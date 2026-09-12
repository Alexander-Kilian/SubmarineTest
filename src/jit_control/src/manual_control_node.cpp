// manual_control_node.cpp
//
// Surface-only manual driving for the BlueROV2, "driven like a ground vehicle":
// one axis for forward/reverse, one for yaw. No lateral, no vertical, no
// roll/pitch input - the vehicle is expected to stay on the surface.
//
// WHAT THIS NODE NO LONGER DOES. It used to own the safety gate, the arming
// retry loop, the flight-mode requests and the MAVROS clients. All of that
// moved to vehicle_interface_node so that it exists exactly once for every
// locomotion source. What is left here is the axis calibration and the
// mapping - which was always this node's actual job.
//
// It publishes cmd/manual/manual_control and nothing else. It holds no MAVROS
// publisher and no service client, and it cannot arm anything.
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
//   Observed raw values: min 191, centre 997, max 1792.
//
// Publishing rules:
//   - jit/mode must be MANUAL, or this node publishes nothing at all.
//     Publishing while not the active source is treated as a defect by
//     vehicle_interface_node, so don't.
//   - It must NOT wait for vehicle/ready. That gate deadlocks the two nodes
//     against each other: vehicle_interface_node only publishes
//     vehicle/ready=true once it has ARMED, and it will not arm until a fresh
//     command from the active source already exists. Each ends up waiting for
//     the other, and the symptom is an endless "Not arming: no fresh command
//     from the active source" with this node silent. (Seen on the first
//     in-water test.) local_guided_node has the same constraint and the same
//     comment.
//     Publishing while disarmed is harmless: cmd/manual/manual_control is an
//     internal topic, and vehicle_interface_node only forwards it to MAVROS
//     once it is armed and in mode. vehicle/ready is kept as a subscription
//     for diagnostics only.
//   - If crsf/channels goes stale, x and r fall to zero but the node KEEPS
//     publishing. Going silent would make vehicle_interface_node safe the
//     vehicle, which is right when this node dies but wrong for a momentary
//     gap in RC frames - the e-stop path already covers a real link loss.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

#include "mavros_msgs/msg/manual_control.hpp"

#include "jit_msgs/msg/mode.hpp"

using namespace std::chrono_literals;
using Mode = jit_msgs::msg::Mode;

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
    fwd_cal_.index = this->declare_parameter<int>("fwd_channel_index", 2);
    fwd_cal_.raw_min = this->declare_parameter<double>("fwd_raw_min", 191.0);
    fwd_cal_.raw_center = this->declare_parameter<double>("fwd_raw_center", 997.0);
    fwd_cal_.raw_max = this->declare_parameter<double>("fwd_raw_max", 1792.0);
    fwd_cal_.deadband = this->declare_parameter<double>("fwd_deadband", 150.0);
    fwd_cal_.invert = this->declare_parameter<bool>("fwd_invert", false);

    yaw_cal_.index = this->declare_parameter<int>("yaw_channel_index", 3);
    yaw_cal_.raw_min = this->declare_parameter<double>("yaw_raw_min", 191.0);
    yaw_cal_.raw_center = this->declare_parameter<double>("yaw_raw_center", 997.0);
    yaw_cal_.raw_max = this->declare_parameter<double>("yaw_raw_max", 1792.0);
    yaw_cal_.deadband = this->declare_parameter<double>("yaw_deadband", 40.0);
    yaw_cal_.invert = this->declare_parameter<bool>("yaw_invert", false);

    z_neutral_ = this->declare_parameter<double>("z_neutral", 500.0);
    send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 20.0);
    // Slew-rate limit in command counts per second (0 = disabled). Caps how
    // fast x/r may change between ticks, turning a stick slam into a ramp.
    slew_max_per_s_ = this->declare_parameter<double>("slew_max_per_s", 0.0);
    axes_timeout_s_ = this->declare_parameter<double>("axes_timeout_s", 0.4);
    bus_timeout_s_ = this->declare_parameter<double>("bus_timeout_s", 0.5);

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    channels_stamp_ = mode_stamp_ = ready_stamp_ = t0;

    cmd_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
      "cmd/manual/manual_control", 10);

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    channels_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "crsf/channels", 10,
      std::bind(&ManualControlNode::channels_cb, this, std::placeholders::_1));
    mode_sub_ = this->create_subscription<Mode>(
      "jit/mode", latched,
      [this](const Mode::SharedPtr m) {granted_mode_ = m->mode; mode_stamp_ = this->now();});
    ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "vehicle/ready", latched,
      [this](const std_msgs::msg::Bool::SharedPtr m) {ready_ = m->data;
        ready_stamp_ = this->now();});

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / send_rate_hz_)),
      std::bind(&ManualControlNode::tick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "manual_control_node up. fwd=CRSF index %d, yaw=CRSF index %d, %.0f Hz. Publishes "
      "cmd/manual/manual_control only while jit/mode is MANUAL.",
      fwd_cal_.index, yaw_cal_.index, send_rate_hz_);
  }

private:
  void channels_cb(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
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

  bool fresh(const rclcpp::Time & stamp, double max_age_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= max_age_s;
  }

  void tick()
  {
    const rclcpp::Time now = this->now();
    const double dt =
      (last_tick_.nanoseconds() > 0) ? (now - last_tick_).seconds() : (1.0 / send_rate_hz_);
    last_tick_ = now;

    const bool mode_ok = (granted_mode_ == Mode::MANUAL) && fresh(mode_stamp_, bus_timeout_s_);
    // Diagnostics only - deliberately NOT part of the gate. See the header.
    const bool ready_ok = ready_ && fresh(ready_stamp_, bus_timeout_s_);

    if (!mode_ok) {
      // Not the active source. Publish nothing and reset the slew memory so a
      // later re-entry ramps from zero rather than from wherever we left off.
      prev_x_ = prev_r_ = 0.0;
      if (was_publishing_) {
        was_publishing_ = false;
        RCLCPP_INFO(this->get_logger(), "Stopped publishing - not the active source.");
      }
      return;
    }

    if (!was_publishing_) {
      was_publishing_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Active source - publishing MANUAL_CONTROL commands (vehicle/ready = %s; "
        "vehicle_interface_node forwards them once it has armed).",
        ready_ok ? "true" : "false");
    }

    const bool axes_ok = channels_valid_ && fresh(channels_stamp_, axes_timeout_s_);
    double x = axes_ok ? map_axis(last_channels_[fwd_cal_.index], fwd_cal_) : 0.0;
    double r = axes_ok ? map_axis(last_channels_[yaw_cal_.index], yaw_cal_) : 0.0;

    if (!axes_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "crsf/channels stale - commanding x=0, r=0.");
    }

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
    cmd_pub_->publish(mc);
  }

  // --- Calibration / config ---
  AxisCal fwd_cal_;
  AxisCal yaw_cal_;
  double z_neutral_ {500.0};
  double send_rate_hz_ {20.0};
  double slew_max_per_s_ {0.0};
  double axes_timeout_s_ {0.4};
  double bus_timeout_s_ {0.5};

  // --- Inputs ---
  std::vector<uint16_t> last_channels_;
  bool channels_valid_ {false};
  rclcpp::Time channels_stamp_;
  uint8_t granted_mode_ {Mode::SAFE};
  rclcpp::Time mode_stamp_;
  bool ready_ {false};
  rclcpp::Time ready_stamp_;

  // --- State ---
  rclcpp::Time last_tick_;
  double prev_x_ {0.0};
  double prev_r_ {0.0};
  bool was_publishing_ {false};

  // --- ROS interfaces ---
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr channels_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}
