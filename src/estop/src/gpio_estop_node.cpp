// gpio_estop_node.cpp
//
// Subscribes to crsf/channel_threshold (published by crsf_channel_node) and
// drives a single GPIO pin as the hardware E-stop for the motor rail.
//
// Signal / relay convention (ASSUMED - confirm against the bench wiring):
//   pin HIGH -> relay coil energized -> normally-open relay CLOSED -> motor rail LIVE
//   pin LOW  -> relay coil de-energized -> relay OPEN -> motor rail CUT (fail-safe)
// i.e. the coil must be actively held energized for the motors to have power,
// so any loss of drive (node dead, pin released, power-on) falls to "cut".
//
//   - On startup: the pin is driven LOW -> motor circuit OPEN. The vehicle
//     powers up safed and stays safed until a fresh crsf/channel_threshold=true
//     is received. (Earlier versions came up HIGH / circuit closed - that was
//     unsafe and has been changed.)
//   - crsf/channel_threshold == true  -> pin HIGH -> circuit closed (motors allowed).
//   - crsf/channel_threshold == false -> pin LOW  -> circuit open (motors cut).
//   - Watchdog: if no crsf/channel_threshold message arrives for
//     `watchdog_timeout_ms`, the pin is forced LOW. This covers crsf_channel_node
//     crashing or being killed (that node otherwise publishes false on every
//     failure mode, so a genuine link loss already drives the pin LOW directly).
//   - On shutdown: the pin is driven LOW and released.
//
// Current state is published on estop/status (std_msgs/Bool) with a
// TRANSIENT_LOCAL ("latched") QoS AND re-published as a ~2 Hz heartbeat, so a
// late subscriber gets the state immediately and a live subscriber can apply
// its own staleness timeout to this topic.
//   estop/status == true  -> motor circuit permitted / closed
//   estop/status == false -> motor circuit cut / e-stop asserted
//
// GPIO is done via libgpiod (the modern kernel gpio-cdev interface), not the
// deprecated sysfs /sys/class/gpio approach.
//
// gpio_line_offset defaults to 23 (BCM GPIO23 / 40-pin header physical pin 16).
// On a Pi 4 gpiochip0, the line offset equals the BCM number. Set the parameter
// to whatever pin the relay driver is actually wired to. Do NOT use offset 0 or
// 1 - those are the ID_SD / ID_SC EEPROM pins, not general-purpose.

#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

extern "C" {
#include <gpiod.h>
}

using namespace std::chrono_literals;

class GpioEstopNode : public rclcpp::Node
{
public:
  GpioEstopNode()
  : Node("gpio_estop_node"), chip_(nullptr), line_(nullptr), circuit_closed_(false)
  {
    chip_name_ = this->declare_parameter<std::string>("gpio_chip", "gpiochip0");
    line_offset_ = this->declare_parameter<int>("gpio_line_offset", 23);
    watchdog_timeout_ms_ = this->declare_parameter<int>("watchdog_timeout_ms", 500);
    const double heartbeat_hz = this->declare_parameter<double>("status_heartbeat_hz", 2.0);
    const double watchdog_check_hz = this->declare_parameter<double>("watchdog_check_hz", 20.0);

    // --- Open the GPIO chip and request the line as an output, driven LOW ---
    chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
    if (!chip_) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to open GPIO chip '%s'. Is libgpiod set up correctly?",
        chip_name_.c_str());
      throw std::runtime_error("gpiod_chip_open_by_name failed");
    }

    line_ = gpiod_chip_get_line(chip_, line_offset_);
    if (!line_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get GPIO line offset %d.", line_offset_);
      throw std::runtime_error("gpiod_chip_get_line failed");
    }

    // Request as output with initial value 0 (LOW) -> motor circuit OPEN at startup.
    const int request_result = gpiod_line_request_output(line_, "gpio_estop_node", 0);
    if (request_result < 0) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Failed to request GPIO line %d as output. Is another process using it, "
        "or do you need to be in the 'gpio' group / run with sudo?",
        line_offset_);
      throw std::runtime_error("gpiod_line_request_output failed");
    }

    circuit_closed_ = false;
    last_threshold_msg_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "GPIO chip '%s' line %d (BCM%d, header pin 16) requested as output and driven LOW "
      "at startup: motor circuit OPEN, e-stop asserted. Waiting for crsf/channel_threshold.",
      chip_name_.c_str(), line_offset_, line_offset_);

    // --- Status publisher: latched + heartbeat ---
    rclcpp::QoS status_qos(1);
    status_qos.reliable();
    status_qos.transient_local();
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>("estop/status", status_qos);
    publish_status();

    // --- Subscribe to the "manual control permitted" bool from crsf_channel_node ---
    threshold_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "crsf/channel_threshold", 10,
      std::bind(&GpioEstopNode::threshold_callback, this, std::placeholders::_1));

    // --- Timers: watchdog + status heartbeat ---
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / watchdog_check_hz)),
      std::bind(&GpioEstopNode::watchdog_tick, this));

    heartbeat_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / heartbeat_hz)),
      std::bind(&GpioEstopNode::publish_status, this));
  }

  ~GpioEstopNode() override
  {
    if (line_) {
      // Drive LOW (motor circuit OPEN) on shutdown rather than leaving the pin
      // in whatever state it last held.
      gpiod_line_set_value(line_, 0);
      gpiod_line_release(line_);
    }
    if (chip_) {
      gpiod_chip_close(chip_);
    }
  }

private:
  void threshold_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    last_threshold_msg_time_ = this->now();
    set_circuit(
      msg->data,
      msg->data ? "crsf/channel_threshold=true (link healthy, enable switch centered)"
                : "crsf/channel_threshold=false (switch out of band, link down, or port closed)");
  }

  void watchdog_tick()
  {
    if (!circuit_closed_) {
      return;  // already open - nothing to protect against
    }
    const double age_ms = (this->now() - last_threshold_msg_time_).seconds() * 1000.0;
    if (age_ms > static_cast<double>(watchdog_timeout_ms_)) {
      char reason[128];
      std::snprintf(
        reason, sizeof(reason),
        "watchdog: no crsf/channel_threshold message for %.0f ms (limit %d ms) - "
        "crsf_channel_node may have died", age_ms, watchdog_timeout_ms_);
      set_circuit(false, reason);
    }
  }

  // Drives the relay pin and publishes status. No-op if the requested state
  // matches the current state.
  void set_circuit(bool close_circuit, const std::string & reason)
  {
    if (close_circuit == circuit_closed_) {
      return;
    }
    circuit_closed_ = close_circuit;

    const int pin_value = circuit_closed_ ? 1 : 0;  // HIGH closes, LOW opens
    if (gpiod_line_set_value(line_, pin_value) < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to set GPIO line %d to %s.",
        line_offset_, circuit_closed_ ? "HIGH" : "LOW");
    }

    if (circuit_closed_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Motor circuit CLOSED - GPIO %d driven HIGH (motors permitted). Reason: %s",
        line_offset_, reason.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "E-STOP ASSERTED - motor circuit OPEN, GPIO %d driven LOW. Reason: %s",
        line_offset_, reason.c_str());
    }

    publish_status();
  }

  void publish_status()
  {
    std_msgs::msg::Bool status_msg;
    status_msg.data = circuit_closed_;  // true = permitted/closed, false = cut/open
    status_pub_->publish(status_msg);
  }

  // --- GPIO ---
  std::string chip_name_;
  int line_offset_;
  gpiod_chip * chip_;
  gpiod_line * line_;

  // --- State ---
  bool circuit_closed_;   // true = pin HIGH / motor rail live; false = pin LOW / cut
  int watchdog_timeout_ms_;
  rclcpp::Time last_threshold_msg_time_;

  // --- ROS interfaces ---
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr threshold_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GpioEstopNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gpio_estop_node"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
