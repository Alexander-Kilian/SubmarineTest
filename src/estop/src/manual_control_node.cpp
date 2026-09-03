// manual_control_node.cpp
//
// Surface-only manual driving for the BlueROV2. Reads two CRSF stick channels
// and streams ArduSub MANUAL_CONTROL (MAVLink #69) over MAVROS, "driven like a
// ground vehicle": one axis for forward/reverse, one for yaw. No lateral, no
// vertical, no roll/pitch input - the vehicle is expected to stay on the
// surface.
//
// Channel mapping (0-based CRSF indices; "channel N" = index N-1 on the Tx):
//   index 2 (channel 3) -> MANUAL_CONTROL x : forward (+) / reverse (-)
//   index 3 (channel 4) -> MANUAL_CONTROL r : yaw right (+) / yaw left (-)
//   MANUAL_CONTROL y = 0            (no lateral)
//   MANUAL_CONTROL z = z_neutral    (default 500 = neutral vertical; ArduSub's
//                                    legacy vertical axis is [0, 1000])
//
// Raw CRSF -> command scaling (per axis, all parameters):
//   raw is mapped through a 0..2000 "stick scale" (raw_min->0, raw_center->1000,
//   raw_max->2000, each half scaled independently so an asymmetric raw range
//   still centres correctly), then shifted to the MANUAL_CONTROL range by
//   subtracting 1000, giving -1000..+1000. Values are clamped to that range,
//   which is exactly MANUAL_CONTROL full scale - so the mapping itself bounds
//   the command. A deadband (in command counts) is applied around centre, and
//   an optional per-axis invert flips the sign.
//   Observed raw values: min 191, centre 997, max 1792 (indices 2, 3 and 7).
//
// Safety gate - MANUAL_CONTROL is streamed (and the vehicle is armed) ONLY when
// ALL of the following hold:
//   1. crsf/channel_threshold == true and fresh (< enable_timeout_s). This is
//      the crsf_channel_node output: receiver link healthy AND the 3-position
//      switch in its centre band.
//   2. estop/status == true and fresh (< estop_timeout_s). The gpio_estop_node
//      relay state - the node will not command through a cut motor circuit.
//   3. /mavros/state.connected == true and fresh. MAVROS <-> ArduSub link live.
//
// When the gate holds (auto-arm + set mode):
//   - if the flight mode is not `target_mode` (default MANUAL; set
//     target_mode:=STABILIZE for attitude-stabilised driving), request it;
//   - once in mode, if the gate has held continuously for arm_hold_s AND the RC
//     is fresh AND both sticks are within arm_stick_epsilon of centre, arm;
//   - once armed and in mode, stream MANUAL_CONTROL at send_rate_hz. If the RC
//     channels go stale (> axes_timeout_s) the commanded x and r fall to 0.
//
// When the gate fails:
//   - stop streaming MANUAL_CONTROL immediately;
//   - if the vehicle is armed, run a disarm loop: call /mavros/cmd/arming(false)
//     at disarm_retry_hz and keep retrying until /mavros/state.armed is false
//     (retry + ACK - never a single fire-and-forget disarm).
//
// The hardware e-stop relay (gpio_estop_node) reacts to the SAME
// crsf/channel_threshold signal independently, so a switch flip or a link loss
// both cuts motor power in hardware AND disarms in software.
//
// Assumed external behaviour (verify on the bench - see the project brief):
//   - ArduSub: MANUAL_CONTROL acts only when armed and in MANUAL/STABILIZE/
//     DEPTH_HOLD; x/y/r in [-1000,1000], z in [0,1000] (500 neutral). MAVROS
//     forwards these values unscaled.
//   - ArduSub accepts a mode change to `target_mode` while disarmed and can arm
//     in that mode on the surface with the current ARMING_CHECK set.
//   - /mavros/cmd/arming and /mavros/set_mode may report success but not take
//     effect; this node verifies against /mavros/state, never the service ACK
//     alone.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

using namespace std::chrono_literals;

namespace
{
struct AxisCal
{
  int index {0};
  double raw_min {191.0};
  double raw_center {997.0};
  double raw_max {1792.0};
  double deadband {40.0};   // in command counts (-1000..1000 domain)
  bool invert {false};
};

// Map one raw CRSF value to a MANUAL_CONTROL command in [-1000, 1000].
double map_axis(uint16_t raw, const AxisCal & c)
{
  const double r = static_cast<double>(raw);
  double stick;  // 0..2000, 1000 = centre
  if (r >= c.raw_center) {
    const double span = std::max(1.0, c.raw_max - c.raw_center);
    stick = 1000.0 + (r - c.raw_center) * 1000.0 / span;
  } else {
    const double span = std::max(1.0, c.raw_center - c.raw_min);
    stick = 1000.0 - (c.raw_center - r) * 1000.0 / span;
  }
  stick = std::clamp(stick, 0.0, 2000.0);

  double v = stick - 1000.0;  // -1000..+1000
  if (std::fabs(v) < c.deadband) {
    v = 0.0;
  }
  if (c.invert) {
    v = -v;
  }
  return std::clamp(v, -1000.0, 1000.0);
}
}  // namespace

class ManualControlNode : public rclcpp::Node
{
public:
  ManualControlNode()
  : Node("manual_control_node")
  {
    // --- Axis calibration parameters ---
    fwd_cal_.index = this->declare_parameter<int>("fwd_channel_index", 2);
    fwd_cal_.raw_min = this->declare_parameter<double>("fwd_raw_min", 191.0);
    fwd_cal_.raw_center = this->declare_parameter<double>("fwd_raw_center", 997.0);
    fwd_cal_.raw_max = this->declare_parameter<double>("fwd_raw_max", 1792.0);
    fwd_cal_.deadband = this->declare_parameter<double>("fwd_deadband", 40.0);
    fwd_cal_.invert = this->declare_parameter<bool>("fwd_invert", false);

    yaw_cal_.index = this->declare_parameter<int>("yaw_channel_index", 3);
    yaw_cal_.raw_min = this->declare_parameter<double>("yaw_raw_min", 191.0);
    yaw_cal_.raw_center = this->declare_parameter<double>("yaw_raw_center", 997.0);
    yaw_cal_.raw_max = this->declare_parameter<double>("yaw_raw_max", 1792.0);
    yaw_cal_.deadband = this->declare_parameter<double>("yaw_deadband", 40.0);
    yaw_cal_.invert = this->declare_parameter<bool>("yaw_invert", false);

    // --- Command shaping ---
    z_neutral_ = this->declare_parameter<double>("z_neutral", 500.0);
    send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 20.0);
    // Slew-rate limit in command counts per second (0 = disabled). Caps how
    // fast x/r may change between ticks, turning a stick slam into a ramp.
    slew_max_per_s_ = this->declare_parameter<double>("slew_max_per_s", 0.0);

    // --- Timeouts / pacing ---
    enable_timeout_s_ = this->declare_parameter<double>("enable_timeout_s", 0.4);
    estop_timeout_s_ = this->declare_parameter<double>("estop_timeout_s", 1.5);
    axes_timeout_s_ = this->declare_parameter<double>("axes_timeout_s", 0.4);
    mavros_timeout_s_ = this->declare_parameter<double>("mavros_timeout_s", 3.0);
    arm_hold_s_ = this->declare_parameter<double>("arm_hold_s", 0.75);
    disarm_retry_hz_ = this->declare_parameter<double>("disarm_retry_hz", 5.0);
    mode_arm_retry_s_ = this->declare_parameter<double>("mode_arm_retry_s", 1.0);
    arm_stick_epsilon_ = this->declare_parameter<double>("arm_stick_epsilon", 60.0);
    // ArduSub flight mode the node commands when the gate holds. Default MANUAL
    // (raw passthrough, no attitude stabilisation). Set target_mode:=STABILIZE
    // for auto-levelled roll/pitch with manual throttle and yaw.
    target_mode_ = this->declare_parameter<std::string>("target_mode", "MANUAL");

    // --- ROS time init (so freshness checks start "stale") ---
    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    enable_stamp_ = estop_stamp_ = channels_stamp_ = mavros_stamp_ = last_service_call_ = t0;

    // --- Publishers ---
    manual_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
      "/mavros/manual_control/send", 10);
    diag_pub_ = this->create_publisher<std_msgs::msg::String>("manual_control/status", 10);

    // --- Subscriptions ---
    rclcpp::QoS latched_qos(1);
    latched_qos.reliable();
    latched_qos.transient_local();

    channels_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "crsf/channels", 10,
      std::bind(&ManualControlNode::channels_callback, this, std::placeholders::_1));
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "crsf/channel_threshold", 10,
      std::bind(&ManualControlNode::enable_callback, this, std::placeholders::_1));
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "estop/status", latched_qos,
      std::bind(&ManualControlNode::estop_callback, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&ManualControlNode::state_callback, this, std::placeholders::_1));

    // --- Service clients ---
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    // --- Control loop ---
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / send_rate_hz_)),
      std::bind(&ManualControlNode::control_tick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "manual_control_node up. fwd=CRSF index %d, yaw=CRSF index %d, target mode '%s', "
      "send %.1f Hz. Streaming MANUAL_CONTROL only when the enable switch, e-stop and "
      "MAVROS gate is satisfied.",
      fwd_cal_.index, yaw_cal_.index, target_mode_.c_str(), send_rate_hz_);
    if (slew_max_per_s_ > 0.0) {
      RCLCPP_INFO(
        this->get_logger(), "Slew-rate limit active: %.0f command counts/s.", slew_max_per_s_);
    }
  }

private:
  enum class Phase { HOLD_SAFE, WAIT_MODE, WAIT_ARM, DRIVING };

  // ---- Subscription callbacks -------------------------------------------------
  void channels_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    const int need = std::max(fwd_cal_.index, yaw_cal_.index);
    if (need < 0 || static_cast<int>(msg->data.size()) <= need) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "crsf/channels has %zu entries but need index %d; ignoring.", msg->data.size(), need);
      channels_valid_ = false;
      return;
    }
    last_channels_.assign(msg->data.begin(), msg->data.end());
    channels_valid_ = true;
    channels_stamp_ = this->now();
  }

  void enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev = enable_value_;
    enable_value_ = msg->data;
    enable_stamp_ = this->now();
    if (prev && !enable_value_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Enable signal dropped (switch left centre, or receiver link lost).");
    }
  }

  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (estop_seen_ && msg->data != estop_value_) {
      RCLCPP_WARN(
        this->get_logger(), "estop/status -> %s.",
        msg->data ? "true (motor circuit permitted)" : "false (motor circuit CUT)");
    }
    estop_seen_ = true;
    estop_value_ = msg->data;
    estop_stamp_ = this->now();
  }

  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    if (state_seen_ && msg->armed != mavros_armed_) {
      RCLCPP_INFO(this->get_logger(), "ArduSub is now %s.", msg->armed ? "ARMED" : "DISARMED");
    }
    if (!msg->mode.empty() && msg->mode != mavros_mode_) {
      RCLCPP_INFO(this->get_logger(), "Flight mode is now '%s'.", msg->mode.c_str());
    }
    if (state_seen_ && msg->connected != mavros_connected_) {
      RCLCPP_WARN(
        this->get_logger(), "MAVROS <-> ArduSub link %s.",
        msg->connected ? "CONNECTED" : "LOST");
    }
    state_seen_ = true;
    mavros_connected_ = msg->connected;
    mavros_armed_ = msg->armed;
    mavros_mode_ = msg->mode;
    mavros_stamp_ = this->now();
  }

  // ---- Helpers --------------------------------------------------------------
  bool fresh(const rclcpp::Time & stamp, double max_age_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= max_age_s;
  }

  bool service_call_due(const rclcpp::Time & now, double period_s) const
  {
    return !service_pending_ && (now - last_service_call_).seconds() >= period_s;
  }

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
    service_pending_ = true;
    last_service_call_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Requesting flight mode '%s'.", mode.c_str());
    set_mode_client_->async_send_request(
      req,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        service_pending_ = false;
        const auto resp = future.get();
        if (resp->mode_sent) {
          RCLCPP_INFO(
            this->get_logger(),
            "Mode '%s' request accepted by MAVROS; awaiting /mavros/state confirmation.",
            mode.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "Mode '%s' request rejected; will retry.", mode.c_str());
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
    service_pending_ = true;
    last_service_call_ = this->now();
    RCLCPP_INFO(
      this->get_logger(), "Commanding %s: %s", arm ? "ARM" : "DISARM", reason.c_str());
    arming_client_->async_send_request(
      req,
      [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        service_pending_ = false;
        const auto resp = future.get();
        if (resp->success) {
          RCLCPP_INFO(
            this->get_logger(), "%s command accepted (result=%u); confirming via /mavros/state.",
            arm ? "ARM" : "DISARM", resp->result);
        } else {
          RCLCPP_WARN(
            this->get_logger(), "%s command REJECTED (result=%u); will retry.",
            arm ? "ARM" : "DISARM", resp->result);
        }
      });
  }

  void set_phase(Phase p, const std::string & detail)
  {
    if (p == phase_) {
      return;
    }
    phase_ = p;
    switch (p) {
      case Phase::HOLD_SAFE:
        RCLCPP_WARN(
          this->get_logger(), "HOLD / SAFE - not streaming MANUAL_CONTROL. %s", detail.c_str());
        break;
      case Phase::WAIT_MODE:
        RCLCPP_INFO(this->get_logger(), "Gate satisfied; setting flight mode. %s", detail.c_str());
        break;
      case Phase::WAIT_ARM:
        RCLCPP_INFO(this->get_logger(), "In mode; preparing to arm. %s", detail.c_str());
        break;
      case Phase::DRIVING:
        RCLCPP_INFO(this->get_logger(), "DRIVING - streaming MANUAL_CONTROL to ArduSub.");
        break;
    }
  }

  void publish_diag(bool gate, bool axes_ok, double x, double r)
  {
    if (++diag_counter_ % 4 != 0) {  // ~5 Hz at a 20 Hz loop
      return;
    }
    std_msgs::msg::String msg;
    const char * phase_str =
      phase_ == Phase::HOLD_SAFE ? "HOLD_SAFE" :
      phase_ == Phase::WAIT_MODE ? "WAIT_MODE" :
      phase_ == Phase::WAIT_ARM ? "WAIT_ARM" : "DRIVING";
    char buf[256];
    std::snprintf(
      buf, sizeof(buf),
      "phase=%s gate=%d enable=%d estop=%d mavros(conn=%d armed=%d mode=%s) axes=%d x=%.0f r=%.0f",
      phase_str, gate ? 1 : 0, enable_value_ ? 1 : 0, estop_value_ ? 1 : 0,
      mavros_connected_ ? 1 : 0, mavros_armed_ ? 1 : 0,
      mavros_mode_.empty() ? "?" : mavros_mode_.c_str(), axes_ok ? 1 : 0, x, r);
    msg.data = buf;
    diag_pub_->publish(msg);
  }

  // ---- Control loop -------------------------------------------------------
  void control_tick()
  {
    const rclcpp::Time now = this->now();
    const double dt =
      (last_tick_.nanoseconds() > 0) ? (now - last_tick_).seconds() : (1.0 / send_rate_hz_);
    last_tick_ = now;

    const bool enable_ok = enable_value_ && fresh(enable_stamp_, enable_timeout_s_);
    const bool estop_ok = estop_seen_ && estop_value_ && fresh(estop_stamp_, estop_timeout_s_);
    const bool mavros_ok = mavros_connected_ && fresh(mavros_stamp_, mavros_timeout_s_);
    const bool axes_ok = channels_valid_ && fresh(channels_stamp_, axes_timeout_s_);
    const bool gate = enable_ok && estop_ok && mavros_ok;

    // ---- Gate failed: stop, and disarm (retry + ACK) ----
    if (!gate) {
      gate_true_since_.reset();
      prev_x_ = prev_r_ = 0.0;

      std::string why;
      if (!enable_ok) {why += "enable ";}
      if (!estop_ok) {why += "estop ";}
      if (!mavros_ok) {why += "mavros ";}
      set_phase(Phase::HOLD_SAFE, "missing: " + why);

      if (mavros_armed_ && service_call_due(now, 1.0 / std::max(0.1, disarm_retry_hz_))) {
        send_arming(false, "gate lost (" + why + ")");
      }
      publish_diag(gate, axes_ok, 0.0, 0.0);
      return;
    }

    // ---- Gate holding ----
    if (!gate_true_since_) {
      gate_true_since_ = now;
    }
    const bool gate_held = (now - *gate_true_since_).seconds() >= arm_hold_s_;

    double x = axes_ok ? map_axis(last_channels_[fwd_cal_.index], fwd_cal_) : 0.0;
    double r = axes_ok ? map_axis(last_channels_[yaw_cal_.index], yaw_cal_) : 0.0;
    const bool sticks_neutral =
      std::fabs(x) < arm_stick_epsilon_ && std::fabs(r) < arm_stick_epsilon_;

    // 1) Flight mode.
    if (mavros_mode_ != target_mode_) {
      set_phase(Phase::WAIT_MODE, "have '" + mavros_mode_ + "', want '" + target_mode_ + "'");
      if (service_call_due(now, mode_arm_retry_s_)) {
        send_set_mode(target_mode_);
      }
      publish_diag(gate, axes_ok, 0.0, 0.0);
      return;
    }

    // 2) Arm.
    if (!mavros_armed_) {
      std::string blocker;
      if (!gate_held) {
        blocker = "gate not held long enough";
      } else if (!axes_ok) {
        blocker = "RC channels stale";
      } else if (!sticks_neutral) {
        blocker = "sticks not neutral";
      }
      set_phase(Phase::WAIT_ARM, blocker.empty() ? "arming now" : ("blocked: " + blocker));
      if (blocker.empty()) {
        if (service_call_due(now, mode_arm_retry_s_)) {
          send_arming(true, "gate satisfied, in mode, sticks neutral");
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "Not arming: %s.", blocker.c_str());
      }
      publish_diag(gate, axes_ok, 0.0, 0.0);
      return;
    }

    // 3) Armed and in mode: stream MANUAL_CONTROL.
    if (slew_max_per_s_ > 0.0) {
      const double max_delta = slew_max_per_s_ * dt;
      x = std::clamp(x, prev_x_ - max_delta, prev_x_ + max_delta);
      r = std::clamp(r, prev_r_ - max_delta, prev_r_ + max_delta);
    }
    prev_x_ = x;
    prev_r_ = r;

    mavros_msgs::msg::ManualControl mc;
    mc.header.stamp = now;
    mc.x = static_cast<float>(x);
    mc.y = 0.0f;
    mc.z = static_cast<float>(z_neutral_);
    mc.r = static_cast<float>(r);
    mc.buttons = 0;
    manual_pub_->publish(mc);

    set_phase(Phase::DRIVING, "");
    if (!axes_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "RC channels stale while driving - commanding x=0, r=0.");
    }
    publish_diag(gate, axes_ok, x, r);
  }

  // ---- Parameters / calibration ----
  AxisCal fwd_cal_;
  AxisCal yaw_cal_;
  double z_neutral_ {500.0};
  double send_rate_hz_ {20.0};
  double slew_max_per_s_ {0.0};
  double enable_timeout_s_ {0.4};
  double estop_timeout_s_ {1.5};
  double axes_timeout_s_ {0.4};
  double mavros_timeout_s_ {3.0};
  double arm_hold_s_ {0.75};
  double disarm_retry_hz_ {5.0};
  double mode_arm_retry_s_ {1.0};
  double arm_stick_epsilon_ {60.0};
  std::string target_mode_ {"MANUAL"};

  // ---- Cached inputs ----
  std::vector<uint16_t> last_channels_;
  bool channels_valid_ {false};
  rclcpp::Time channels_stamp_;

  bool enable_value_ {false};
  rclcpp::Time enable_stamp_;

  bool estop_seen_ {false};
  bool estop_value_ {false};
  rclcpp::Time estop_stamp_;

  bool state_seen_ {false};
  bool mavros_connected_ {false};
  bool mavros_armed_ {false};
  std::string mavros_mode_;
  rclcpp::Time mavros_stamp_;

  // ---- Control state ----
  Phase phase_ {Phase::HOLD_SAFE};
  std::optional<rclcpp::Time> gate_true_since_;
  rclcpp::Time last_tick_;
  double prev_x_ {0.0};
  double prev_r_ {0.0};

  bool service_pending_ {false};
  rclcpp::Time last_service_call_;
  uint64_t diag_counter_ {0};

  // ---- ROS interfaces ----
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr channels_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}
