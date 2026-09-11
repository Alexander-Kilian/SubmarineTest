// gpio_estop_node.cpp
//
// Drives the motor-rail relay and reads back whether the rail actually went
// live. This node holds no policy of its own beyond an AND, two watchdogs and a
// comparison - which is exactly why it can be trusted.
//
// ---------------------------------------------------------------------------
// TWO PERMITS, BOTH REQUIRED
// ---------------------------------------------------------------------------
//   crsf/relay_permit  (std_msgs/Bool)  - from crsf_channel_node, direct from
//                                         the switch and the link FSM
//   sys/health_ok      (std_msgs/Bool)  - from system_monitor_node
//
// The relay is closed only when BOTH are true AND both are fresh. Each input
// gets its own independent staleness check, so a dead crsf_channel_node and a
// dead system_monitor_node each open the relay on their own, without either
// needing to notice the other. There is deliberately no node between the
// operator's switch and this one.
//
// ---------------------------------------------------------------------------
// RELAY FEEDBACK
// ---------------------------------------------------------------------------
// An LDO fed from the switched (post-relay) battery rail drives an input pin:
//   feedback HIGH -> relay closed -> motor rail LIVE
//   feedback LOW  -> relay open   -> motor rail DEAD
//
// The pin is requested as an input with an internal PULL-DOWN bias, so a broken
// or disconnected feedback wire floats to LOW and is read as "rail dead". The
// safe interpretation of "I don't know" is that the rail is not live.
//
// Comparing commanded against feedback gives two faults the system could not
// otherwise see:
//
//   commanded closed, feedback open -> RELAY_NOT_CLOSED. Not dangerous (the
//     motors are dead either way) but it distinguishes "the sub won't move"
//     from "the sub won't move AND here is why": blown fuse, battery
//     disconnected, open relay coil, dead LDO.
//
//   commanded open, feedback CLOSED -> RELAY_WELDED. The hardware e-stop is not
//     working and the only remaining stop is the software disarm path. Loud,
//     latched, and it forces a disarm through system_monitor_node.
//
// SETTLE WINDOW. The relay is mechanical and the LDO output has capacitance to
// discharge, so the feedback lags a commanded change - and lags further on the
// falling edge than the rising one. Comparisons are suppressed for
// relay_settle_ms after any commanded change; without that, every legitimate
// open would throw a spurious RELAY_NOT_CLOSED.
//
// BOOT SELF-TEST. The constructor drives the output LOW before anything else,
// waits out the settle window, and reads the feedback. If it reads HIGH the
// relay is welded, this node throws, and the launch aborts. A vehicle whose
// e-stop is welded shut should not finish booting. Set boot_selftest:=false for
// bench work where the relay or the LDO is not connected.
//
// A welded relay detected at RUNTIME is NOT fatal. Killing this process would
// release the pin and stop estop/status, which makes the situation worse; the
// correct response is the one the monitor already takes - raise RELAY_WELDED
// and disarm through vehicle_interface_node.
//
// ---------------------------------------------------------------------------
// SIGNAL CONVENTION (confirmed against the bench wiring)
//   pin HIGH -> relay coil energized -> relay CLOSED -> motor rail LIVE
//   pin LOW  -> coil de-energized    -> relay OPEN   -> motor rail CUT
// The coil must be actively held energized for the motors to have power, so any
// loss of drive (node dead, pin released, power-on) falls to "cut".
//
// GPIO is done via libgpiod (the modern kernel gpio-cdev interface), not the
// deprecated sysfs /sys/class/gpio approach. On a Pi 4 gpiochip0 the line
// offset equals the BCM number.
//   relay command  : BCM23 / header pin 16
//   relay feedback : BCM18 / header pin 12

#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "jit_msgs/msg/estop_status.hpp"

extern "C" {
#include <gpiod.h>
}

using namespace std::chrono_literals;

class GpioEstopNode : public rclcpp::Node
{
public:
  GpioEstopNode()
  : Node("gpio_estop_node"), chip_(nullptr), cmd_line_(nullptr), fb_line_(nullptr),
    commanded_(false)
  {
    chip_name_ = this->declare_parameter<std::string>("gpio_chip", "gpiochip0");
    cmd_offset_ = this->declare_parameter<int>("relay_cmd_line_offset", 23);
    fb_offset_ = this->declare_parameter<int>("relay_feedback_line_offset", 18);
    watchdog_timeout_ms_ = this->declare_parameter<int>("watchdog_timeout_ms", 500);
    relay_settle_ms_ = this->declare_parameter<int>("relay_settle_ms", 300);
    boot_selftest_ = this->declare_parameter<bool>("boot_selftest", true);
    const double heartbeat_hz = this->declare_parameter<double>("status_heartbeat_hz", 2.0);
    const double evaluate_hz = this->declare_parameter<double>("evaluate_hz", 20.0);

    // Inputs start stale (stamp of zero), which reads as "not fresh", which
    // denies the permit. The vehicle comes up safed and stays safed until both
    // publishers have actually said yes.
    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    permit_stamp_ = health_stamp_ = t0;
    last_cmd_change_ = this->now();

    open_gpio();
    run_boot_selftest();

    // --- Status publisher: latched + heartbeat ---
    rclcpp::QoS status_qos(1);
    status_qos.reliable();
    status_qos.transient_local();
    status_pub_ = this->create_publisher<jit_msgs::msg::EstopStatus>("estop/status", status_qos);
    publish_status();

    // --- The two permits ---
    permit_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "crsf/relay_permit", 10,
      std::bind(&GpioEstopNode::permit_callback, this, std::placeholders::_1));
    health_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "sys/health_ok", 10,
      std::bind(&GpioEstopNode::health_callback, this, std::placeholders::_1));

    // --- Timers ---
    evaluate_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / evaluate_hz)),
      std::bind(&GpioEstopNode::evaluate, this));

    heartbeat_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / heartbeat_hz)),
      std::bind(&GpioEstopNode::publish_status, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Relay command = BCM%d (driven LOW at startup, rail CUT). Feedback = BCM%d "
      "(pull-down bias). Settle %d ms, watchdog %d ms per input. Waiting for "
      "crsf/relay_permit AND sys/health_ok.",
      cmd_offset_, fb_offset_, relay_settle_ms_, watchdog_timeout_ms_);
  }

  ~GpioEstopNode() override
  {
    if (cmd_line_) {
      // Drive LOW (rail CUT) on shutdown rather than leaving the pin in
      // whatever state it last held.
      gpiod_line_set_value(cmd_line_, 0);
      gpiod_line_release(cmd_line_);
    }
    if (fb_line_) {
      gpiod_line_release(fb_line_);
    }
    if (chip_) {
      gpiod_chip_close(chip_);
    }
  }

private:
  // ---- Setup ---------------------------------------------------------------
  void open_gpio()
  {
    chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
    if (!chip_) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to open GPIO chip '%s'. Is libgpiod set up correctly?",
        chip_name_.c_str());
      throw std::runtime_error("gpiod_chip_open_by_name failed");
    }

    cmd_line_ = gpiod_chip_get_line(chip_, cmd_offset_);
    if (!cmd_line_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get relay command line %d.", cmd_offset_);
      throw std::runtime_error("gpiod_chip_get_line (command) failed");
    }
    // Request as output with initial value 0 (LOW) -> motor rail CUT at startup.
    if (gpiod_line_request_output(cmd_line_, "gpio_estop_node", 0) < 0) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Failed to request GPIO line %d as output. Is another process using it, or do "
        "you need to be in the 'gpio' group / run with sudo?", cmd_offset_);
      throw std::runtime_error("gpiod_line_request_output failed");
    }
    commanded_ = false;
    last_cmd_change_ = this->now();

    fb_line_ = gpiod_chip_get_line(chip_, fb_offset_);
    if (!fb_line_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get relay feedback line %d.", fb_offset_);
      throw std::runtime_error("gpiod_chip_get_line (feedback) failed");
    }

    // Internal pull-down: there is no external resistor on this pin, so bias it
    // in software. A disconnected feedback wire then reads LOW = "rail dead",
    // which is the pessimistic and therefore correct default.
    int fb_flags = 0;
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN
    fb_flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
#endif
    if (gpiod_line_request_input_flags(fb_line_, "gpio_estop_node", fb_flags) < 0) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to request GPIO line %d as input.", fb_offset_);
      throw std::runtime_error("gpiod_line_request_input_flags failed");
    }
    if (fb_flags == 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "This libgpiod is too old for GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN, so the "
        "feedback pin has NO bias. Add an external ~100k pull-down, or a broken "
        "feedback wire will float and may read as a live rail.");
    }
  }

  // Drive LOW, wait out the settle window, and require the feedback to agree.
  // A relay that reads closed while commanded open is welded, and the hardware
  // e-stop is therefore non-functional.
  void run_boot_selftest()
  {
    if (!boot_selftest_) {
      RCLCPP_WARN(
        this->get_logger(),
        "boot_selftest:=false - skipping the welded-relay check. Do not fly like this.");
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(relay_settle_ms_));

    const int value = gpiod_line_get_value(fb_line_);
    if (value < 0) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Boot self-test: cannot read the relay feedback line %d. Refusing to start - "
        "without feedback there is no way to know the e-stop works.", fb_offset_);
      throw std::runtime_error("relay feedback read failed at boot");
    }
    if (value != 0) {
      RCLCPP_FATAL(
        this->get_logger(),
        "BOOT SELF-TEST FAILED: relay command is LOW but the feedback on BCM%d reads "
        "HIGH after %d ms. The motor rail is live while the e-stop is asserted - the "
        "relay is WELDED, or the feedback pin is miswired. Refusing to start.",
        fb_offset_, relay_settle_ms_);
      throw std::runtime_error("welded relay detected at boot");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Boot self-test passed: rail confirmed dead with the e-stop asserted.");
  }

  // ---- Inputs --------------------------------------------------------------
  void permit_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (have_permit_ && msg->data != permit_value_) {
      RCLCPP_INFO(
        this->get_logger(), "crsf/relay_permit -> %s.", msg->data ? "true" : "false");
    }
    have_permit_ = true;
    permit_value_ = msg->data;
    permit_stamp_ = this->now();
  }

  void health_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (have_health_ && msg->data != health_value_) {
      RCLCPP_INFO(
        this->get_logger(), "sys/health_ok -> %s.", msg->data ? "true" : "false");
    }
    have_health_ = true;
    health_value_ = msg->data;
    health_stamp_ = this->now();
  }

  bool fresh(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() * 1000.0 <= static_cast<double>(watchdog_timeout_ms_);
  }

  // ---- The only place the output pin is written ----------------------------
  void evaluate()
  {
    const rclcpp::Time now = this->now();

    const bool permit_ok = have_permit_ && permit_value_ && fresh(permit_stamp_);
    const bool health_ok = have_health_ && health_value_ && fresh(health_stamp_);
    const bool want_closed = permit_ok && health_ok;

    if (want_closed != commanded_) {
      std::string reason;
      if (!want_closed) {
        reason = "denied by:";
        if (!have_permit_ || !fresh(permit_stamp_)) {
          reason += " crsf/relay_permit stale/absent";
        } else if (!permit_value_) {
          reason += " crsf/relay_permit false";
        }
        if (!have_health_ || !fresh(health_stamp_)) {
          reason += " sys/health_ok stale/absent";
        } else if (!health_value_) {
          reason += " sys/health_ok false";
        }
      } else {
        reason = "both permits true and fresh";
      }
      set_relay(want_closed, reason);
    }

    // --- Feedback comparison -------------------------------------------------
    const int raw = gpiod_line_get_value(fb_line_);
    if (raw < 0) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to read the relay feedback line %d.", fb_offset_);
      // Unknown feedback is treated as "rail dead" - the pessimistic reading.
      feedback_ = false;
    } else {
      feedback_ = (raw != 0);
    }

    settled_ =
      (now - last_cmd_change_).seconds() * 1000.0 >= static_cast<double>(relay_settle_ms_);
    const bool mismatch = settled_ && (feedback_ != commanded_);

    if (mismatch != mismatch_) {
      mismatch_ = mismatch;
      if (mismatch_ && !commanded_ && feedback_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "RELAY WELDED: commanded OPEN but the rail is still LIVE after %d ms. The "
          "hardware e-stop is NOT working. The software disarm path is now the only "
          "stop - system_monitor_node will force a disarm.", relay_settle_ms_);
      } else if (mismatch_ && commanded_ && !feedback_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "RELAY NOT CLOSED: commanded CLOSED but the rail is dead after %d ms. Check "
          "the fuse, the battery connection, the relay coil and the LDO.", relay_settle_ms_);
      } else {
        RCLCPP_INFO(this->get_logger(), "Relay feedback now agrees with the command.");
      }
      publish_status();
    }
  }

  void set_relay(bool close_circuit, const std::string & reason)
  {
    commanded_ = close_circuit;
    last_cmd_change_ = this->now();
    settled_ = false;
    mismatch_ = false;  // no verdict until the new state has settled

    if (gpiod_line_set_value(cmd_line_, commanded_ ? 1 : 0) < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to set GPIO line %d to %s.",
        cmd_offset_, commanded_ ? "HIGH" : "LOW");
    }

    if (commanded_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Motor rail COMMANDED LIVE - BCM%d HIGH. Reason: %s", cmd_offset_, reason.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "E-STOP ASSERTED - motor rail CUT, BCM%d LOW. Reason: %s", cmd_offset_, reason.c_str());
    }

    publish_status();
  }

  void publish_status()
  {
    jit_msgs::msg::EstopStatus msg;
    msg.stamp = this->now();
    msg.commanded = commanded_;
    msg.feedback = feedback_;
    msg.mismatch = mismatch_;
    msg.settled = settled_;
    status_pub_->publish(msg);
  }

  // --- GPIO ---
  std::string chip_name_;
  int cmd_offset_;
  int fb_offset_;
  gpiod_chip * chip_;
  gpiod_line * cmd_line_;
  gpiod_line * fb_line_;

  // --- Config ---
  int watchdog_timeout_ms_;
  int relay_settle_ms_;
  bool boot_selftest_;

  // --- Permit inputs ---
  bool have_permit_ {false};
  bool permit_value_ {false};
  rclcpp::Time permit_stamp_;
  bool have_health_ {false};
  bool health_value_ {false};
  rclcpp::Time health_stamp_;

  // --- Relay state ---
  bool commanded_;          // what we drove the pin to
  bool feedback_ {false};   // what the LDO says the rail is doing
  bool mismatch_ {false};
  bool settled_ {false};
  rclcpp::Time last_cmd_change_;

  // --- ROS interfaces ---
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permit_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;
  rclcpp::Publisher<jit_msgs::msg::EstopStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr evaluate_timer_;
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
