// vehicle_interface_raw_node.cpp
//
// VARIANT B of the vehicle interface. Identical to vehicle_interface_node in
// every respect except the transport used for guided position targets:
//
//   variant A  /mavros/setpoint_position/local   geometry_msgs/PoseStamped
//   variant B  /mavros/setpoint_raw/local        mavros_msgs/PositionTarget   <- this file
//
// Both plugins emit the SAME MAVLink message, SET_POSITION_TARGET_LOCAL_NED,
// and both perform the ENU->NED conversion inside MAVROS, so the values we put
// in are ENU exactly as the guided nodes produce them. What changes is that
// setpoint_position hard-codes its type_mask and derives yaw from the pose
// quaternion, whereas setpoint_raw exposes both. That is the entire reason this
// variant exists.
//
// ---------------------------------------------------------------------------
// WHY type_mask IS WORTH TESTING
// ---------------------------------------------------------------------------
// The observed failure was the sub yawing onto the correct track bearing and
// then sitting there. The leading explanation is the s-curve reset described in
// vehicle_interface_node.cpp, but a commanded yaw fighting the position
// controller is also consistent with that symptom, and the two are separable
// only by trying them. With `type_mask` as a parameter you can switch between:
//
//   2552  position + yaw       (IGNORE_V* | IGNORE_AF* | IGNORE_YAW_RATE)
//   3576  position only        (the above | IGNORE_YAW - ArduSub picks heading)
//
// without a rebuild, which matters when the test site has a slow link.
//
// ---------------------------------------------------------------------------
// EXPERIMENT DESIGN
// ---------------------------------------------------------------------------
// `stream_mode` is present here and in variant A so the two questions can be
// separated instead of confounded:
//
//   A/dedupe  vs  A/continuous   isolates the RATE question
//   A/dedupe  vs  B/dedupe       isolates the TRANSPORT / type_mask question
//
// Changing transport and rate together would leave a good result unattributable.
//
// ---------------------------------------------------------------------------
// UNCHANGED FROM VARIANT A - do not let these drift
// ---------------------------------------------------------------------------
//   - the single-writer rule: this is the only node publishing to MAVROS when
//     it is the one launched;
//   - the health gate, the arm preconditions and the mode-before-arm ordering;
//   - DISARM IS ALWAYS RETRIED UNTIL /mavros/state.armed READS FALSE, and sent
//     at least min_disarm_commands times regardless. Standing project rule: the
//     service ACK is never proof.
//   - safing runs concurrently, leaving the guided mode immediately rather than
//     after the disarm confirms, because a GUIDED position target has no
//     staleness timeout.
//
// Assumed external behaviour (verify on the bench):
//   - MAVROS's setpoint_raw plugin converts position ENU->NED and yaw ENU->NED
//     for coordinate_frame FRAME_LOCAL_NED, the same as setpoint_position does.
//   - ArduSub honours the type_mask rather than ignoring it and reading every
//     field.
//   - A guided position target sent ONCE is retained and flown with no further
//     messages.

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
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "jit_msgs/msg/health.hpp"
#include "jit_msgs/msg/mode.hpp"

using namespace std::chrono_literals;
using Health = jit_msgs::msg::Health;
using Mode = jit_msgs::msg::Mode;
using PositionTarget = mavros_msgs::msg::PositionTarget;

namespace
{
// Position + yaw: ignore velocity, acceleration and yaw rate.
constexpr uint16_t MASK_POS_YAW =
  PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ |
  PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ |
  PositionTarget::IGNORE_YAW_RATE;
}  // namespace

class VehicleInterfaceRawNode : public rclcpp::Node
{
public:
  VehicleInterfaceRawNode()
  : Node("vehicle_interface_raw_node")
  {
    manual_ardusub_mode_ =
      this->declare_parameter<std::string>("manual_ardusub_mode", "STABILIZE");
    guided_ardusub_mode_ =
      this->declare_parameter<std::string>("guided_ardusub_mode", "GUIDED");
    safing_mode_ = this->declare_parameter<std::string>("safing_mode", "STABILIZE");

    health_timeout_s_ = this->declare_parameter<double>("health_timeout_s", 0.5);
    mode_timeout_s_ = this->declare_parameter<double>("mode_timeout_s", 0.5);
    mavros_timeout_s_ = this->declare_parameter<double>("mavros_timeout_s", 3.0);
    cmd_timeout_s_ = this->declare_parameter<double>("cmd_timeout_s", 0.5);
    arm_hold_s_ = this->declare_parameter<double>("arm_hold_s", 0.75);
    disarm_retry_hz_ = this->declare_parameter<double>("disarm_retry_hz", 5.0);
    mode_arm_retry_s_ = this->declare_parameter<double>("mode_arm_retry_s", 1.0);
    send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 20.0);

    min_disarm_commands_ = this->declare_parameter<int>("min_disarm_commands", 2);
    arm_stick_epsilon_ = this->declare_parameter<double>("arm_stick_epsilon", 60.0);
    z_neutral_ = this->declare_parameter<double>("z_neutral", 500.0);

    setpoint_epsilon_m_ = this->declare_parameter<double>("setpoint_epsilon_m", 0.05);
    setpoint_yaw_epsilon_rad_ =
      this->declare_parameter<double>("setpoint_yaw_epsilon_rad", 0.02);
    setpoint_burst_count_ = this->declare_parameter<int>("setpoint_burst_count", 5);
    setpoint_burst_interval_s_ =
      this->declare_parameter<double>("setpoint_burst_interval_s", 0.05);

    stream_mode_ = this->declare_parameter<std::string>("stream_mode", "dedupe");
    if (stream_mode_ != "dedupe" && stream_mode_ != "continuous") {
      RCLCPP_WARN(
        this->get_logger(), "stream_mode '%s' is not recognised; using 'dedupe'.",
        stream_mode_.c_str());
      stream_mode_ = "dedupe";
    }

    // --- The reason this variant exists ---
    // Default 2552 = position + yaw, matching what setpoint_position/local
    // sends. Set 3576 to add IGNORE_YAW and let ArduSub choose the heading.
    type_mask_ = static_cast<uint16_t>(
      this->declare_parameter<int>("type_mask", static_cast<int>(MASK_POS_YAW)));
    coordinate_frame_ = static_cast<uint8_t>(
      this->declare_parameter<int>(
        "coordinate_frame", static_cast<int>(PositionTarget::FRAME_LOCAL_NED)));

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    health_stamp_ = mode_stamp_ = mavros_stamp_ = t0;
    manual_stamp_ = local_stamp_ = global_stamp_ = t0;
    last_arm_call_ = last_mode_call_ = t0;
    last_sp_send_ = t0;

    manual_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
      "/mavros/manual_control/send", 10);
    // The one line that differs from variant A.
    setpoint_pub_ = this->create_publisher<PositionTarget>("/mavros/setpoint_raw/local", 10);

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();
    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("vehicle/ready", latched);

    health_sub_ = this->create_subscription<Health>(
      "jit/health", latched,
      [this](const Health::SharedPtr m) {health_ = *m; have_health_ = true;
        health_stamp_ = this->now();});
    mode_sub_ = this->create_subscription<Mode>(
      "jit/mode", latched,
      [this](const Mode::SharedPtr m) {granted_mode_ = m->mode; have_mode_ = true;
        mode_stamp_ = this->now();});

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
      std::bind(&VehicleInterfaceRawNode::state_cb, this, std::placeholders::_1));

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / send_rate_hz_)),
      std::bind(&VehicleInterfaceRawNode::control_tick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "vehicle_interface_raw_node (VARIANT B) up. MANUAL -> '%s', guided -> '%s', "
      "safing -> '%s'. Loop %.0f Hz. Guided transport: setpoint_raw/local, "
      "type_mask=%u (%s), frame=%u, stream_mode '%s'. Disarm retries at %.1f Hz until "
      "/mavros/state confirms, minimum %d commands.",
      manual_ardusub_mode_.c_str(), guided_ardusub_mode_.c_str(), safing_mode_.c_str(),
      send_rate_hz_, static_cast<unsigned>(type_mask_),
      (type_mask_ & PositionTarget::IGNORE_YAW) ? "position only" : "position + yaw",
      static_cast<unsigned>(coordinate_frame_), stream_mode_.c_str(),
      disarm_retry_hz_, min_disarm_commands_);
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

    if (!mavros_armed_ || mavros_mode_ != guided_ardusub_mode_) {
      invalidate_setpoint_dedupe();
    }
  }

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
      RCLCPP_WARN(this->get_logger(), "DISARM command #%d: %s", disarm_sent_, reason.c_str());
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
        // A success ACK is NOT confirmation. Only /mavros/state.armed is.
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

    if (!gate_true_since_) {
      gate_true_since_ = now;
    }
    if (safing_active_) {
      RCLCPP_INFO(this->get_logger(), "Gate restored - leaving safing.");
      safing_active_ = false;
      disarm_sent_ = 0;
    }

    const std::string target_mode = ardusub_mode_for(granted_mode_);

    if (mavros_mode_ != target_mode) {
      if (mode_call_due(now, mode_arm_retry_s_)) {
        send_set_mode(target_mode);
      }
      publish_ready(false);
      return;
    }

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
    return true;
  }

  static double yaw_of(const geometry_msgs::msg::PoseStamped & p)
  {
    return 2.0 * std::atan2(p.pose.orientation.z, p.pose.orientation.w);
  }

  // Repack the guided node's ENU PoseStamped into a PositionTarget. MAVROS's
  // setpoint_raw plugin does the ENU->NED conversion, so the values go in as
  // the guided node produced them.
  PositionTarget to_target(const geometry_msgs::msg::PoseStamped & sp) const
  {
    PositionTarget t;
    t.header = sp.header;
    t.coordinate_frame = coordinate_frame_;
    t.type_mask = type_mask_;
    t.position = sp.pose.position;
    t.yaw = static_cast<float>(yaw_of(sp));
    t.yaw_rate = 0.0f;
    return t;
  }

  bool setpoint_is_new(const geometry_msgs::msg::PoseStamped & sp) const
  {
    if (!have_sent_sp_) {
      return true;
    }
    const auto & a = sp.pose.position;
    const auto & b = last_sent_sp_.pose.position;
    if (std::fabs(a.x - b.x) > setpoint_epsilon_m_ ||
      std::fabs(a.y - b.y) > setpoint_epsilon_m_ ||
      std::fabs(a.z - b.z) > setpoint_epsilon_m_)
    {
      return true;
    }
    // Yaw cannot make a target "new" when we are not commanding yaw at all.
    if (type_mask_ & PositionTarget::IGNORE_YAW) {
      return false;
    }
    double dyaw = yaw_of(sp) - yaw_of(last_sent_sp_);
    while (dyaw > M_PI) {dyaw -= 2.0 * M_PI;}
    while (dyaw < -M_PI) {dyaw += 2.0 * M_PI;}
    return std::fabs(dyaw) > setpoint_yaw_epsilon_rad_;
  }

  void forward_active_command(const rclcpp::Time & now)
  {
    if (granted_mode_ == Mode::MANUAL) {
      auto mc = manual_cmd_;
      mc.header.stamp = now;
      manual_pub_->publish(mc);
      return;
    }

    auto sp = (granted_mode_ == Mode::LOCAL_GUIDED) ? local_cmd_ : global_cmd_;

    if (stream_mode_ == "continuous") {
      auto t = to_target(sp);
      t.header.stamp = now;
      setpoint_pub_->publish(t);
      last_sent_sp_ = sp;
      have_sent_sp_ = true;
      return;
    }

    if (setpoint_is_new(sp)) {
      sp_burst_remaining_ = std::max(1, setpoint_burst_count_);
      last_sent_sp_ = sp;
      have_sent_sp_ = true;
      last_sp_send_ = rclcpp::Time(0, 0, now.get_clock_type());
      RCLCPP_INFO(
        this->get_logger(),
        "New guided target E=%.2f N=%.2f z=%.2f yaw=%.1f deg (mask %u) - sending %d "
        "time(s), then holding silent.",
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z,
        yaw_of(sp) * 180.0 / M_PI, static_cast<unsigned>(type_mask_), sp_burst_remaining_);
    }

    if (sp_burst_remaining_ > 0 &&
      (last_sp_send_.nanoseconds() == 0 ||
      (now - last_sp_send_).seconds() >= setpoint_burst_interval_s_))
    {
      auto t = to_target(last_sent_sp_);
      t.header.stamp = now;
      setpoint_pub_->publish(t);
      last_sp_send_ = now;
      --sp_burst_remaining_;
    }
  }

  // ---- Safing --------------------------------------------------------------
  void run_safing(const rclcpp::Time & now, const std::string & reason)
  {
    gate_true_since_.reset();
    publish_ready(false);
    invalidate_setpoint_dedupe();

    if (!safing_active_) {
      safing_active_ = true;
      disarm_sent_ = 0;
      RCLCPP_WARN(this->get_logger(), "SAFING - %s", reason.c_str());
    }

    if (!have_state_) {
      return;
    }

    // 1. Out of GUIDED first - the position target has no staleness timeout.
    if (mavros_mode_ == guided_ardusub_mode_ && mode_call_due(now, mode_arm_retry_s_)) {
      send_set_mode(safing_mode_);
    }

    // 2. Disarm, retried until /mavros/state confirms. Standing project rule.
    const bool need_more = mavros_armed_ || (disarm_sent_ < min_disarm_commands_);
    if (need_more && arm_call_due(now, 1.0 / std::max(0.1, disarm_retry_hz_))) {
      send_arming(false, reason);
    }
    if (!mavros_armed_ && disarm_sent_ >= min_disarm_commands_ && !disarm_confirmed_logged_) {
      disarm_confirmed_logged_ = true;
      RCLCPP_INFO(
        this->get_logger(), "Disarm CONFIRMED by /mavros/state after %d commands.",
        disarm_sent_);
    }
    if (mavros_armed_) {
      disarm_confirmed_logged_ = false;
    }

    // 3. Still armed but out of GUIDED: command zero motion.
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
  double setpoint_yaw_epsilon_rad_ {0.02};
  int setpoint_burst_count_ {5};
  double setpoint_burst_interval_s_ {0.05};
  std::string stream_mode_ {"dedupe"};
  uint16_t type_mask_ {MASK_POS_YAW};
  uint8_t coordinate_frame_ {PositionTarget::FRAME_LOCAL_NED};

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
  rclcpp::Publisher<PositionTarget>::SharedPtr setpoint_pub_;
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
  rclcpp::spin(std::make_shared<VehicleInterfaceRawNode>());
  rclcpp::shutdown();
  return 0;
}
