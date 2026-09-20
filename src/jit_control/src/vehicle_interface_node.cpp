// vehicle_interface_node.cpp
//
// The ONLY node in the system that publishes to MAVROS or calls its services.
// Extracted wholesale from the old manual_control_node so that arming, mode
// setting and command arbitration exist exactly once.
//
// ---------------------------------------------------------------------------
// WHY A SINGLE WRITER
// ---------------------------------------------------------------------------
// The alternative is letting every locomotion node keep its own arm/mode logic
// and trusting that only one runs at a time. But "only one runs at a time" is
// exactly the guarantee that does not hold during a mode change, which is
// precisely when two nodes would both be deciding to arm. Locomotion nodes
// therefore publish to cmd/<source>/* and never construct a MAVROS client.
//
// A command arriving from a source that is not the active one is dropped and
// logged as a defect - it means a locomotion node is publishing when the bus
// says it should not be, and that is a bug worth seeing rather than silently
// absorbing.
//
// ---------------------------------------------------------------------------
// DISARM IS ALWAYS RETRIED UNTIL CONFIRMED  (project standing rule)
// ---------------------------------------------------------------------------
// /mavros/cmd/arming can return success without the vehicle actually
// disarming, so the service ACK is never treated as proof. The only acceptable
// confirmation is /mavros/state.armed going false. run_safing() therefore keeps
// re-sending the disarm at disarm_retry_hz until the state confirms it, AND
// sends it at least min_disarm_commands times regardless - never a single
// fire-and-forget disarm.
//
// ---------------------------------------------------------------------------
// SAFING RUNS CONCURRENTLY, NOT IN SEQUENCE
// ---------------------------------------------------------------------------
// An earlier draft disarmed first and only then changed mode out of GUIDED.
// That has a hole: if the disarm needs several seconds of retries, GUIDED keeps
// driving toward its target for that whole time. So safing now does all of
// these on every tick until they take:
//
//   1. request the mode change out of GUIDED immediately - this stops position
//      tracking now rather than after the disarm confirms;
//   2. run the disarm retry loop;
//   3. once out of GUIDED, publish neutral MANUAL_CONTROL so that if the
//      vehicle is still armed for a moment, the commanded motion is zero;
//   4. hold vehicle/ready false.
//
// Step 1 matters more than it looks: an ArduSub GUIDED position target has NO
// staleness timeout (verified in ArduSub/mode_guided.cpp - only velocity
// targets expire). Merely stopping our setpoint stream does not stop the
// vehicle. If the motor rail is later re-energised with a stale target still
// loaded, the vehicle drives back to it. Leaving GUIDED is what actually clears
// the target.
//
// ---------------------------------------------------------------------------
// Assumed external behaviour (verify on the bench):
//   - ArduSub accepts a mode change to GUIDED while disarmed and can arm in it
//     on the surface with the current ARMING_CHECK set.
//   - ArduSub accepts a mode change to `safing_mode` (STABILIZE) while armed.
//   - MAVROS forwards MANUAL_CONTROL unscaled; x/y/r in [-1000,1000],
//     z in [0,1000] with 500 neutral.
//   - A guided position target sent ONCE is retained and flown by ArduSub with
//     no further messages, and setpoint_burst_count repeats are enough to
//     survive MAVLink loss over the BlueOS router. See forward_active_command.
//   - MAV_GCS_SYSID on the vehicle equals MAVROS's system id, or mode changes
//     are silently ignored.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "jit_msgs/msg/health.hpp"
#include "jit_msgs/msg/mode.hpp"

using namespace std::chrono_literals;
using Health = jit_msgs::msg::Health;
using Mode = jit_msgs::msg::Mode;

class VehicleInterfaceNode : public rclcpp::Node
{
public:
  VehicleInterfaceNode()
  : Node("vehicle_interface_node")
  {
    // --- ArduSub mode strings ---
    // Our MANUAL maps to ArduSub STABILIZE by default: auto-levelled roll and
    // pitch with manual throttle and yaw. Set manual_ardusub_mode:=MANUAL for
    // raw passthrough with no attitude stabilisation.
    manual_ardusub_mode_ =
      this->declare_parameter<std::string>("manual_ardusub_mode", "STABILIZE");
    guided_ardusub_mode_ =
      this->declare_parameter<std::string>("guided_ardusub_mode", "GUIDED");
    // What we fall back to when safing. Must not be GUIDED, or the stale
    // position target is never cleared.
    safing_mode_ = this->declare_parameter<std::string>("safing_mode", "STABILIZE");

    // --- Timeouts / pacing ---
    health_timeout_s_ = this->declare_parameter<double>("health_timeout_s", 0.5);
    mode_timeout_s_ = this->declare_parameter<double>("mode_timeout_s", 0.5);
    mavros_timeout_s_ = this->declare_parameter<double>("mavros_timeout_s", 3.0);
    cmd_timeout_s_ = this->declare_parameter<double>("cmd_timeout_s", 0.5);
    arm_hold_s_ = this->declare_parameter<double>("arm_hold_s", 0.75);
    disarm_retry_hz_ = this->declare_parameter<double>("disarm_retry_hz", 5.0);
    mode_arm_retry_s_ = this->declare_parameter<double>("mode_arm_retry_s", 1.0);
    send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 20.0);

    // Standing rule: never a single fire-and-forget disarm. Even if the very
    // first command works, send at least this many.
    min_disarm_commands_ = this->declare_parameter<int>("min_disarm_commands", 2);

    // Arm precondition for manual: both sticks within this of centre, in
    // MANUAL_CONTROL counts.
    arm_stick_epsilon_ = this->declare_parameter<double>("arm_stick_epsilon", 60.0);
    z_neutral_ = this->declare_parameter<double>("z_neutral", 500.0);

    // --- Guided setpoint dedupe (see forward_active_command) ---
    // A guided position target is a ONE-SHOT command to ArduSub, not a stream.
    // Only forward one when the target position actually moves by more than
    // this. Position alone decides - see setpoint_is_new().
    setpoint_epsilon_m_ = this->declare_parameter<double>("setpoint_epsilon_m", 0.05);
    // Each new target is sent this many times, this far apart, then we go quiet.
    // The repeats cover MAVLink loss over the BlueOS router; they are close
    // enough together that the s-curve reset they cause is irrelevant.
    setpoint_burst_count_ = this->declare_parameter<int>("setpoint_burst_count", 5);
    setpoint_burst_interval_s_ =
      this->declare_parameter<double>("setpoint_burst_interval_s", 0.05);

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    health_stamp_ = mode_stamp_ = mavros_stamp_ = t0;
    manual_stamp_ = local_stamp_ = global_stamp_ = t0;
    last_arm_call_ = last_mode_call_ = t0;
    last_sp_send_ = t0;

    // --- Publishers to MAVROS (the only ones in the system) ---
    manual_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
      "/mavros/manual_control/send", 10);
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();
    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("vehicle/ready", latched);

    // --- Bus ---
    health_sub_ = this->create_subscription<Health>(
      "jit/health", latched,
      [this](const Health::SharedPtr m) {health_ = *m; have_health_ = true;
        health_stamp_ = this->now();});
    mode_sub_ = this->create_subscription<Mode>(
      "jit/mode", latched,
      [this](const Mode::SharedPtr m) {granted_mode_ = m->mode; have_mode_ = true;
        mode_stamp_ = this->now();});

    // --- Command sources ---
    manual_sub_ = this->create_subscription<mavros_msgs::msg::ManualControl>(
      "cmd/manual/manual_control", 10,
      [this](const mavros_msgs::msg::ManualControl::SharedPtr m) {
        if (!accept_from(Mode::MANUAL, "manual")) {return;}
        manual_cmd_ = *m;
        manual_stamp_ = this->now();
      });
    local_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "cmd/local_guided/setpoint", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
        if (!accept_from(Mode::LOCAL_GUIDED, "local_guided")) {return;}
        local_cmd_ = *m;
        local_stamp_ = this->now();
      });
    global_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "cmd/global_guided/setpoint", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
        if (!accept_from(Mode::GLOBAL_GUIDED, "global_guided")) {return;}
        global_cmd_ = *m;
        global_stamp_ = this->now();
      });

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&VehicleInterfaceNode::state_cb, this, std::placeholders::_1));

    // --- Service clients ---
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / send_rate_hz_)),
      std::bind(&VehicleInterfaceNode::control_tick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "vehicle_interface_node up. MANUAL -> ArduSub '%s', guided -> '%s', safing -> '%s'. "
      "Loop %.0f Hz. Guided targets go out on setpoint_position/local, deduped. "
      "Disarm retries at %.1f Hz until /mavros/state confirms, minimum %d commands.",
      manual_ardusub_mode_.c_str(), guided_ardusub_mode_.c_str(), safing_mode_.c_str(),
      send_rate_hz_, disarm_retry_hz_, min_disarm_commands_);
  }

private:
  // ---- Inputs --------------------------------------------------------------
  bool accept_from(uint8_t source_mode, const char * name)
  {
    if (granted_mode_ == source_mode) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dropping a command from '%s' - the bus says the mode is %u, not %u. That source "
      "should not be publishing. This is a defect, not a race.",
      name, static_cast<unsigned>(granted_mode_), static_cast<unsigned>(source_mode));
    return false;
  }

  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    if (have_state_ && msg->armed != mavros_armed_) {
      RCLCPP_INFO(this->get_logger(), "ArduSub is now %s.", msg->armed ? "ARMED" : "DISARMED");
    }
    if (!msg->mode.empty() && msg->mode != mavros_mode_) {
      RCLCPP_INFO(this->get_logger(), "Flight mode is now '%s'.", msg->mode.c_str());
    }
    if (have_state_ && msg->connected != mavros_connected_) {
      RCLCPP_WARN(
        this->get_logger(), "MAVROS <-> ArduSub link %s.",
        msg->connected ? "CONNECTED" : "LOST");
    }
    have_state_ = true;
    mavros_connected_ = msg->connected;
    mavros_armed_ = msg->armed;
    mavros_mode_ = msg->mode;
    mavros_stamp_ = this->now();

    // Anything that destroys the destination ArduSub is holding invalidates the
    // dedupe, so the next tick re-sends rather than staying silent about a
    // target the vehicle no longer has. Leaving GUIDED clears it, and while
    // disarmed guided_pos_control_run() re-inits wp_nav every loop.
    if (!mavros_armed_ || mavros_mode_ != guided_ardusub_mode_) {
      invalidate_setpoint_dedupe();
    }
  }

  // Force the next guided tick to transmit, whatever the last sent value was.
  void invalidate_setpoint_dedupe()
  {
    have_sent_sp_ = false;
    sp_burst_remaining_ = 0;
  }

  bool fresh(const rclcpp::Time & stamp, double max_age_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= max_age_s;
  }

  // ---- MAVROS calls --------------------------------------------------------
  void send_set_mode(const std::string & mode)
  {
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "/mavros/set_mode not available yet.");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode = 0;
    req->custom_mode = mode;
    mode_pending_ = true;
    last_mode_call_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Requesting flight mode '%s'.", mode.c_str());
    set_mode_client_->async_send_request(
      req,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        mode_pending_ = false;
        const auto resp = future.get();
        if (!resp->mode_sent) {
          RCLCPP_WARN(this->get_logger(), "Mode '%s' rejected; will retry.", mode.c_str());
        }
      });
  }

  void send_arming(bool arm, const std::string & reason)
  {
    if (!arming_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "/mavros/cmd/arming not available yet.");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    arm_pending_ = true;
    last_arm_call_ = this->now();
    if (!arm) {
      ++disarm_sent_;
      RCLCPP_WARN(
        this->get_logger(), "DISARM command #%d: %s", disarm_sent_, reason.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "ARM command: %s", reason.c_str());
    }
    arming_client_->async_send_request(
      req,
      [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        arm_pending_ = false;
        const auto resp = future.get();
        if (!resp->success) {
          RCLCPP_WARN(
            this->get_logger(), "%s REJECTED (result=%u); will retry.",
            arm ? "ARM" : "DISARM", resp->result);
        }
        // Note: a success ACK is NOT treated as confirmation. Only
        // /mavros/state.armed is.
      });
  }

  bool arm_call_due(const rclcpp::Time & now, double period_s) const
  {
    return !arm_pending_ && (now - last_arm_call_).seconds() >= period_s;
  }
  bool mode_call_due(const rclcpp::Time & now, double period_s) const
  {
    return !mode_pending_ && (now - last_mode_call_).seconds() >= period_s;
  }

  // ---- Main loop -----------------------------------------------------------
  void control_tick()
  {
    const rclcpp::Time now = this->now();

    const bool health_ok = have_health_ && health_.ok && fresh(health_stamp_, health_timeout_s_);
    const bool mode_ok = have_mode_ && fresh(mode_stamp_, mode_timeout_s_);
    const bool mavros_ok = have_state_ && mavros_connected_ &&
      fresh(mavros_stamp_, mavros_timeout_s_);
    const bool gate = health_ok && mode_ok && mavros_ok && granted_mode_ != Mode::SAFE;

    if (!gate) {
      std::string why;
      if (!health_ok) {
        why = have_health_ ? ("health: " + health_.detail) : "health: never received";
      } else if (!mode_ok) {
        why = "jit/mode stale";
      } else if (!mavros_ok) {
        why = "MAVROS link";
      } else {
        why = "granted mode is SAFE";
      }
      run_safing(now, why);
      return;
    }

    // Gate is holding.
    if (!gate_true_since_) {
      gate_true_since_ = now;
    }
    if (safing_active_) {
      RCLCPP_INFO(this->get_logger(), "Gate restored - leaving safing.");
      safing_active_ = false;
      disarm_sent_ = 0;
    }

    const std::string target_mode = ardusub_mode_for(granted_mode_);

    // 1. Flight mode first. Never arm before the mode is confirmed.
    if (mavros_mode_ != target_mode) {
      if (mode_call_due(now, mode_arm_retry_s_)) {
        send_set_mode(target_mode);
      }
      publish_ready(false);
      return;
    }

    // 2. Arm, once the preconditions hold.
    if (!mavros_armed_) {
      std::string blocker;
      if ((now - *gate_true_since_).seconds() < arm_hold_s_) {
        blocker = "gate not held long enough";
      } else if (!active_cmd_fresh(now)) {
        blocker = "no fresh command from the active source";
      } else if (!arm_precondition_ok()) {
        blocker = (granted_mode_ == Mode::MANUAL)
          ? "sticks not neutral" : "no valid first setpoint";
      }
      if (blocker.empty()) {
        if (arm_call_due(now, mode_arm_retry_s_)) {
          send_arming(true, "gate satisfied, in mode '" + target_mode + "'");
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "Not arming: %s.", blocker.c_str());
      }
      publish_ready(false);
      return;
    }

    // 3. Armed and in mode. If the active source has gone quiet, that source
    //    has died - safe rather than coast on a stale command.
    if (!active_cmd_fresh(now)) {
      run_safing(now, "active command source went stale");
      return;
    }

    forward_active_command(now);
    publish_ready(true);
  }

  std::string ardusub_mode_for(uint8_t granted) const
  {
    switch (granted) {
      case Mode::MANUAL: return manual_ardusub_mode_;
      case Mode::LOCAL_GUIDED:
      case Mode::GLOBAL_GUIDED: return guided_ardusub_mode_;
      default: return safing_mode_;
    }
  }

  bool active_cmd_fresh(const rclcpp::Time & now) const
  {
    (void)now;
    switch (granted_mode_) {
      case Mode::MANUAL: return fresh(manual_stamp_, cmd_timeout_s_);
      case Mode::LOCAL_GUIDED: return fresh(local_stamp_, cmd_timeout_s_);
      case Mode::GLOBAL_GUIDED: return fresh(global_stamp_, cmd_timeout_s_);
      default: return false;
    }
  }

  bool arm_precondition_ok() const
  {
    if (granted_mode_ == Mode::MANUAL) {
      return std::fabs(manual_cmd_.x) < arm_stick_epsilon_ &&
             std::fabs(manual_cmd_.r) < arm_stick_epsilon_;
    }
    // Guided: a fresh setpoint having arrived at all is the precondition. The
    // guided node refuses to publish one without a fresh pose, so a setpoint
    // existing already means the position estimate was good when it was built.
    return true;
  }

  // Yaw about Z from a yaw-only quaternion, as the guided nodes build them.
  // Used only to log what heading went out with a target; the change test below
  // deliberately ignores yaw.
  static double yaw_of(const geometry_msgs::msg::PoseStamped & p)
  {
    return 2.0 * std::atan2(p.pose.orientation.z, p.pose.orientation.w);
  }

  // POSITION ONLY, deliberately.
  //
  // A waypoint carries no orientation (see local_guided_node's Waypoint), so
  // the yaw on a setpoint is not an input at all - it is synthesised from the
  // live pose as the bearing to the target. Comparing it here meant estimator
  // noise could declare a target "new" that had not moved, and every false
  // trigger is another AC_WPNav::set_wp_destination() re-anchoring the leg and
  // restarting the s-curve. Position catches every real change on its own:
  // consecutive waypoints are metres apart, and mode exit, disarm or safing
  // clear have_sent_sp_ outright rather than relying on any threshold.
  bool setpoint_is_new(const geometry_msgs::msg::PoseStamped & sp) const
  {
    if (!have_sent_sp_) {
      return true;
    }
    const auto & a = sp.pose.position;
    const auto & b = last_sent_sp_.pose.position;
    return std::fabs(a.x - b.x) > setpoint_epsilon_m_ ||
           std::fabs(a.y - b.y) > setpoint_epsilon_m_ ||
           std::fabs(a.z - b.z) > setpoint_epsilon_m_;
  }

  // MANUAL_CONTROL is a stream and must be sent every tick. A guided position
  // target is the opposite: a ONE-SHOT command.
  //
  // ArduSub routes SET_POSITION_TARGET_LOCAL_NED to guided_set_destination(),
  // which calls AC_WPNav::set_wp_destination(). That is not idempotent - it
  // re-anchors the leg origin to the current position target and recalculates
  // the s-curve from scratch, restarting the acceleration ramp at zero. Sending
  // it at send_rate_hz therefore reset the ramp 20 times a second and the
  // vehicle never accelerated: observed in the water as the sub yawing onto the
  // correct track bearing and then sitting there, with the four horizontal
  // thrusters within 9 us of each other while the verticals held attitude
  // normally. The target not expiring (see the safing notes above) is exactly
  // why it must be sent once, not continuously.
  //
  // So: transmit only when the target actually moves, in a short burst to cover
  // MAVLink loss, then go quiet and let ArduSub fly the leg. The incoming
  // cmd/*/setpoint stream is still consumed every tick - it is what feeds the
  // cmd_timeout_s freshness gate - it is just not forwarded.
  void forward_active_command(const rclcpp::Time & now)
  {
    if (granted_mode_ == Mode::MANUAL) {
      auto mc = manual_cmd_;
      mc.header.stamp = now;
      manual_pub_->publish(mc);
      return;
    }

    auto sp = (granted_mode_ == Mode::LOCAL_GUIDED) ? local_cmd_ : global_cmd_;

    if (setpoint_is_new(sp)) {
      sp_burst_remaining_ = std::max(1, setpoint_burst_count_);
      last_sent_sp_ = sp;
      have_sent_sp_ = true;
      last_sp_send_ = rclcpp::Time(0, 0, now.get_clock_type());
      RCLCPP_INFO(
        this->get_logger(),
        "New guided target E=%.2f N=%.2f z=%.2f yaw=%.1f deg - sending %d time(s), "
        "then holding silent.",
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z,
        yaw_of(sp) * 180.0 / M_PI, sp_burst_remaining_);
    }

    if (sp_burst_remaining_ > 0 &&
      (last_sp_send_.nanoseconds() == 0 ||
      (now - last_sp_send_).seconds() >= setpoint_burst_interval_s_))
    {
      auto out = last_sent_sp_;
      out.header.stamp = now;
      setpoint_pub_->publish(out);
      last_sp_send_ = now;
      --sp_burst_remaining_;
    }
  }

  // ---- Safing --------------------------------------------------------------
  // Every route to a stopped vehicle comes through here: fault-safe and
  // mission-safe differ only in the logged reason and in whether the monitor
  // also opened the relay.
  void run_safing(const rclcpp::Time & now, const std::string & reason)
  {
    gate_true_since_.reset();
    publish_ready(false);
    // Safing leaves GUIDED, which destroys the destination. Re-entry must
    // re-send it rather than dedupe against a target that no longer exists.
    invalidate_setpoint_dedupe();

    if (!safing_active_) {
      safing_active_ = true;
      disarm_sent_ = 0;
      RCLCPP_WARN(this->get_logger(), "SAFING - %s", reason.c_str());
    }

    if (!have_state_) {
      return;  // nothing to command yet
    }

    // 1. Get out of GUIDED immediately. A GUIDED position target has no
    //    staleness timeout, so this is what actually stops position tracking.
    if (mavros_mode_ == guided_ardusub_mode_ && mode_call_due(now, mode_arm_retry_s_)) {
      send_set_mode(safing_mode_);
    }

    // 2. Disarm: retry until /mavros/state confirms, and at least
    //    min_disarm_commands times regardless. Standing project rule.
    const bool need_more = mavros_armed_ || (disarm_sent_ < min_disarm_commands_);
    if (need_more && arm_call_due(now, 1.0 / std::max(0.1, disarm_retry_hz_))) {
      send_arming(false, reason);
    }
    if (!mavros_armed_ && disarm_sent_ >= min_disarm_commands_ && !disarm_confirmed_logged_) {
      disarm_confirmed_logged_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Disarm CONFIRMED by /mavros/state after %d commands.", disarm_sent_);
    }
    if (mavros_armed_) {
      disarm_confirmed_logged_ = false;
    }

    // 3. If still armed but out of GUIDED, command zero motion so nothing is
    //    left driving while the disarm retries.
    if (mavros_armed_ && mavros_mode_ != guided_ardusub_mode_) {
      mavros_msgs::msg::ManualControl neutral;
      neutral.header.stamp = now;
      neutral.x = 0.0f;
      neutral.y = 0.0f;
      neutral.z = static_cast<float>(z_neutral_);
      neutral.r = 0.0f;
      neutral.buttons = 0;
      manual_pub_->publish(neutral);
    }
  }

  void publish_ready(bool ready)
  {
    if (have_ready_ && ready == last_ready_) {
      return;
    }
    have_ready_ = true;
    last_ready_ = ready;
    std_msgs::msg::Bool msg;
    msg.data = ready;
    ready_pub_->publish(msg);
  }

  // --- Config ---
  std::string manual_ardusub_mode_;
  std::string guided_ardusub_mode_;
  std::string safing_mode_;
  double health_timeout_s_ {0.5};
  double mode_timeout_s_ {0.5};
  double mavros_timeout_s_ {3.0};
  double cmd_timeout_s_ {0.5};
  double arm_hold_s_ {0.75};
  double disarm_retry_hz_ {5.0};
  double mode_arm_retry_s_ {1.0};
  double send_rate_hz_ {20.0};
  int min_disarm_commands_ {2};
  double arm_stick_epsilon_ {60.0};
  double z_neutral_ {500.0};
  double setpoint_epsilon_m_ {0.05};
  int setpoint_burst_count_ {5};
  double setpoint_burst_interval_s_ {0.05};

  // --- Bus ---
  bool have_health_ {false};
  Health health_;
  rclcpp::Time health_stamp_;
  bool have_mode_ {false};
  uint8_t granted_mode_ {Mode::SAFE};
  rclcpp::Time mode_stamp_;

  // --- Command sources ---
  mavros_msgs::msg::ManualControl manual_cmd_;
  rclcpp::Time manual_stamp_;
  geometry_msgs::msg::PoseStamped local_cmd_;
  rclcpp::Time local_stamp_;
  geometry_msgs::msg::PoseStamped global_cmd_;
  rclcpp::Time global_stamp_;

  // --- MAVROS state ---
  bool have_state_ {false};
  bool mavros_connected_ {false};
  bool mavros_armed_ {false};
  std::string mavros_mode_;
  rclcpp::Time mavros_stamp_;

  // --- Control state ---
  std::optional<rclcpp::Time> gate_true_since_;

  // --- Guided setpoint dedupe ---
  geometry_msgs::msg::PoseStamped last_sent_sp_;
  bool have_sent_sp_ {false};
  int sp_burst_remaining_ {0};
  rclcpp::Time last_sp_send_;
  bool safing_active_ {true};
  int disarm_sent_ {0};
  bool disarm_confirmed_logged_ {false};
  bool arm_pending_ {false};
  bool mode_pending_ {false};
  rclcpp::Time last_arm_call_;
  rclcpp::Time last_mode_call_;
  bool have_ready_ {false};
  bool last_ready_ {false};

  // --- ROS interfaces ---
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<Health>::SharedPtr health_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
