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

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    health_stamp_ = mode_stamp_ = mavros_stamp_ = t0;
    manual_stamp_ = local_stamp_ = global_stamp_ = t0;
    last_arm_call_ = last_mode_call_ = t0;

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
      "Loop %.0f Hz. Disarm retries at %.1f Hz until /mavros/state confirms, minimum %d "
      "commands.",
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

  void forward_active_command(const rclcpp::Time & now)
  {
    if (granted_mode_ == Mode::MANUAL) {
      auto mc = manual_cmd_;
      mc.header.stamp = now;
      manual_pub_->publish(mc);
    } else {
      auto sp = (granted_mode_ == Mode::LOCAL_GUIDED) ? local_cmd_ : global_cmd_;
      sp.header.stamp = now;
      setpoint_pub_->publish(sp);
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
