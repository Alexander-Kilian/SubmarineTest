// led_driver_node.cpp
//
// STUB. Accepts the indicator-panel command and logs it. No hardware access.
//
// The panel's drive mechanism is not yet specified - PWM, I2C, addressable
// serial and a plain GPIO-per-colour are all still on the table - so this node
// exists to pin down the INTERFACE now and defer the hardware. When the panel
// is chosen, this is the only file that has to change; nothing upstream knows
// or cares how the light is produced.
//
// The panel shows the physical state of the motor rail first and the mode
// second, because "the rail is dead" is what an operator needs to read from
// across a pool deck:
//
//   ESTOP  -> RED     the relay is open (whatever the switch says)
//   MANUAL -> YELLOW  rail live, manual control
//   AUTO   -> GREEN   rail live, a guided mode is active
//
// That decision is made entirely in system_monitor_node. This node must never
// re-derive it from health or mode topics - a panel that can disagree with the
// monitor about the vehicle's state is worse than no panel.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "jit_msgs/msg/led_command.hpp"

class LedDriverNode : public rclcpp::Node
{
public:
  LedDriverNode()
  : Node("led_driver_node")
  {
    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    sub_ = this->create_subscription<jit_msgs::msg::LedCommand>(
      "led/command", latched,
      std::bind(&LedDriverNode::led_cb, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "led_driver_node up as a STUB - logging led/command, driving no hardware. "
      "TODO: implement the panel driver once the mechanism is specified.");
  }

private:
  void led_cb(const jit_msgs::msg::LedCommand::SharedPtr msg)
  {
    if (msg->pattern == pattern_) {
      return;
    }
    pattern_ = msg->pattern;

    // TODO(panel): drive the real hardware here.
    const char * colour =
      (pattern_ == jit_msgs::msg::LedCommand::ESTOP) ? "RED" :
      (pattern_ == jit_msgs::msg::LedCommand::MANUAL) ? "YELLOW" :
      (pattern_ == jit_msgs::msg::LedCommand::AUTO) ? "GREEN" : "UNKNOWN";

    RCLCPP_INFO(
      this->get_logger(), "LED panel would show %s (pattern '%s').", colour, pattern_.c_str());
  }

  std::string pattern_;
  rclcpp::Subscription<jit_msgs::msg::LedCommand>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedDriverNode>());
  rclcpp::shutdown();
  return 0;
}
