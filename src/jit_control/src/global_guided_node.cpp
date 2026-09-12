// global_guided_node.cpp
//
// SKELETON. This node contains no navigation and deliberately cannot move the
// vehicle.
//
// It exists so that the health bus and the command mux in
// vehicle_interface_node are exercised with THREE consumers rather than two.
// Adding a third source later should be a matter of filling this in, not of
// discovering that the architecture only ever worked for two.
//
// The GPS is not connected and GLOBAL_GUIDED has no switch detent (see
// crsf_channel_node), so this node should never become the active source on the
// current vehicle. If it somehow does, it publishes nothing, the active command
// source goes stale, and vehicle_interface_node safes the vehicle. That is the
// correct outcome and is worth testing on the bench by temporarily remapping
// the UP detent.
//
// What it does today:
//   - subscribes to the bus and to the GPS fix, so the topics exist and the
//     graph is complete;
//   - publishes nav/global/status = IDLE forever. system_monitor_node ignores
//     the status of any source that is not the active one, so this never
//     affects health;
//   - never publishes cmd/global_guided/setpoint.
//
// When it is filled in it should mirror local_guided_node: capture a datum on
// mode entry, treat waypoints as offsets, mirror ArduPilot's radius-then-dwell
// acceptance, and publish setpoints continuously while active.

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "jit_msgs/msg/mode.hpp"

using namespace std::chrono_literals;
using Mode = jit_msgs::msg::Mode;

class GlobalGuidedNode : public rclcpp::Node
{
public:
  GlobalGuidedNode()
  : Node("global_guided_node")
  {
    const double rate_hz = this->declare_parameter<double>("status_rate_hz", 10.0);

    status_pub_ = this->create_publisher<std_msgs::msg::String>("nav/global/status", 10);

    // Declared but never published to. Advertising it keeps the graph honest
    // and lets `ros2 topic info` show the mux wiring even before this node
    // does anything.
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "cmd/global_guided/setpoint", 10);

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    mode_sub_ = this->create_subscription<Mode>(
      "jit/mode", latched,
      [this](const Mode::SharedPtr m) {
        if (m->mode == Mode::GLOBAL_GUIDED && !warned_) {
          warned_ = true;
          RCLCPP_ERROR(
            this->get_logger(),
            "GLOBAL_GUIDED is the granted mode, but this node is a skeleton and will "
            "not publish setpoints. vehicle_interface_node will safe the vehicle when "
            "the active command source goes stale. This is expected.");
        }
        if (m->mode != Mode::GLOBAL_GUIDED) {
          warned_ = false;
        }
      });

    // Best-effort, to match MAVROS's sensor-topic QoS. See local_guided_node.
    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr) {
        if (!seen_fix_) {
          seen_fix_ = true;
          RCLCPP_INFO(this->get_logger(), "First GPS fix seen (not used - skeleton node).");
        }
      });

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / rate_hz)),
      [this]() {
        std_msgs::msg::String msg;
        msg.data = "IDLE";
        status_pub_->publish(msg);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "global_guided_node up as a SKELETON - no navigation, no setpoints. Present so "
      "the bus and the command mux are exercised with three consumers.");
  }

private:
  bool warned_ {false};
  bool seen_fix_ {false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalGuidedNode>());
  rclcpp::shutdown();
  return 0;
}
