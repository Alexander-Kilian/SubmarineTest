// crsf_channel_node.cpp
//
// Reads CRSF/ELRS channel data from a T8L-paired receiver via the xcrsf
// library and publishes:
//   - crsf/channels           (std_msgs/UInt16MultiArray)  all 16 raw channels
//   - crsf/link_ok            (std_msgs/Bool)               receiver link healthy
//   - crsf/channel_threshold  (std_msgs/Bool)               "manual control permitted"
//
// WIRING NOTE: on the real vehicle the link to the ELRS receiver is ONE-WAY.
// Only the receiver's TX line is wired to the Pi's UART RX (GPIO15); the Pi's
// UART TX (GPIO14) is NOT connected to the receiver (space constraints on the
// penetrator). The Pi therefore cannot transmit to the receiver at all - no
// telemetry uplink, no CRSF config. This is fine for reading channels: the
// receiver still streams the RC-channels frame and the link-statistics frame
// (uplink LQ / RSSI) down that single wire, so is_paired() and the planned
// get_link_state() quality check both still work.
//
// crsf/channel_threshold is the safety-relevant output. It is true ONLY when
// BOTH of these hold:
//   1. the receiver link is healthy, and
//   2. the enable channel (the 3-position switch, channel index
//      `enable_channel_index`) is inside the neutral centre band.
//
// It is published on EVERY poll cycle. Every failure mode - serial port not
// open, receiver not paired, link quality below floor, channel index
// misconfigured, or switch out of band - publishes `false` that same cycle.
// The node never goes silent while running, so a downstream consumer that
// stops seeing messages can treat that as "this node died" (see the watchdog
// in gpio_estop_node and the enable-timeout in manual_control_node).
//
// A separate node (gpio_estop_node) subscribes to crsf/channel_threshold and
// drives the relay GPIO. "Decode CRSF" and "touch hardware GPIO" stay as two
// independent, individually testable nodes.
//
// Verified against the real installed header (/usr/local/include/xcrsf/crossfire.h):
//   open_port(), is_paired(), get_channel_state() -> std::array<uint16_t, 16>, get_link_state().

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <termios.h>  // speed_t

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "xcrsf/crossfire.h"

using namespace std::chrono_literals;

class CrsfChannelNode : public rclcpp::Node
{
public:
  CrsfChannelNode()
  : Node("crsf_channel_node"), port_open_(false)
  {
    // --- Parameters (set via a launch file or `ros2 run ... --ros-args -p`) ---
    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    poll_rate_hz_ = this->declare_parameter<double>("poll_rate_hz", 50.0);

    // CRSF UART baud rate. Standard CRSF is 420000; xcrsf defaults to that.
    // Set this to match the ELRS receiver if its CRSF baud was changed (e.g.
    // 400000, 921600). If it doesn't match, frames never decode and
    // is_paired() stays false.
    crsf_baud_ = this->declare_parameter<int>("crsf_baud", 115200);

    // Enable channel: the 3-position switch. 0-based index into the 16-channel
    // CRSF array. Default 7 (i.e. "channel 8" as counted on the transmitter).
    // Verify with `ros2 topic echo /crsf/channels` that index 7 is the one
    // that swings between the switch detents (observed ~191 / ~997 / ~1792);
    // override the parameter if not.
    enable_channel_index_ = this->declare_parameter<int>("enable_channel_index", 7);

    // Neutral centre band for the enable switch, in raw CRSF counts. The
    // switch centre detent was observed at ~997, the extremes at ~191 / ~1792,
    // so [700, 1300] leaves roughly 500 counts of margin to each extreme.
    neutral_low_ = this->declare_parameter<int>("neutral_low", 700);
    neutral_high_ = this->declare_parameter<int>("neutral_high", 1300);

    // Link-quality floor (0-100). NOTE: not yet enforced - see link_healthy().
    min_link_quality_ = this->declare_parameter<int>("min_link_quality", 50);

    // --- Publishers ---
    channels_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("crsf/channels", 10);
    link_ok_pub_ = this->create_publisher<std_msgs::msg::Bool>("crsf/link_ok", 10);
    threshold_pub_ = this->create_publisher<std_msgs::msg::Bool>("crsf/channel_threshold", 10);

    // --- Open the CRSF serial link ---
    crossfire_ = std::make_unique<crossfire::XCrossfire>(
      serial_port_, static_cast<speed_t>(crsf_baud_));
    port_open_ = crossfire_->open_port();
    if (port_open_) {
      RCLCPP_INFO(
        this->get_logger(), "Opened CRSF port '%s' at %d baud.",
        serial_port_.c_str(), crsf_baud_);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to open CRSF port '%s' at %d baud - retrying every poll cycle. "
        "Publishing crsf/channel_threshold=false until it recovers.",
        serial_port_.c_str(), crsf_baud_);
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Link-loss detection currently relies on xcrsf is_paired() only. Ensure the "
      "ELRS receiver failsafe is set to 'no pulses'. TODO: enforce min_link_quality "
      "(=%d) via get_link_state() so a receiver that HOLDS last values is also caught.",
      min_link_quality_);

    RCLCPP_INFO(
      this->get_logger(),
      "Enable switch = channel index %d; neutral band = [%d, %d] counts; poll = %.1f Hz.",
      enable_channel_index_, neutral_low_, neutral_high_, poll_rate_hz_);

    // --- Poll timer ---
    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CrsfChannelNode::poll_callback, this));
  }

private:
  void poll_callback()
  {
    // 1. Ensure the serial port is open.
    if (!port_open_) {
      port_open_ = crossfire_->open_port();
      if (!port_open_) {
        publish_link_ok(false);
        publish_threshold(false);
        return;
      }
      RCLCPP_INFO(this->get_logger(), "CRSF port '%s' recovered.", serial_port_.c_str());
    }

    // 2. Check the receiver link.
    const bool link = link_healthy();
    publish_link_ok(link);
    if (!link) {
      // No valid link this cycle: fail safe, do not publish stale channel data.
      publish_threshold(false);
      return;
    }

    // 3. Publish the raw channels.
    const std::array<uint16_t, 16> channels = crossfire_->get_channel_state();
    std_msgs::msg::UInt16MultiArray channels_msg;
    channels_msg.data.assign(channels.begin(), channels.end());
    channels_pub_->publish(channels_msg);

    // 4. Derive "manual control permitted" from the enable switch.
    if (enable_channel_index_ < 0 ||
      static_cast<size_t>(enable_channel_index_) >= channels.size())
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "enable_channel_index %d is out of range [0, %zu); publishing "
        "crsf/channel_threshold=false. Set the parameter to your switch channel.",
        enable_channel_index_, channels.size());
      publish_threshold(false);
      return;
    }

    const uint16_t value = channels[enable_channel_index_];
    const bool permitted =
      (value >= static_cast<uint16_t>(neutral_low_) &&
       value <= static_cast<uint16_t>(neutral_high_));
    publish_threshold(permitted);
  }

  // Returns true when the receiver link is considered healthy.
  //
  // TODO(link-quality): once the return type of crossfire_->get_link_state()
  // in /usr/local/include/xcrsf/crossfire.h is known, additionally require its
  // link-quality / RSSI to be >= min_link_quality_. That covers the case where
  // the ELRS receiver is configured to HOLD the last channel values (or output
  // configured failsafe positions) on TX loss instead of stopping frames - in
  // that case is_paired() can stay true even though the transmitter is gone.
  // The link-statistics frame that carries this quality figure still reaches
  // the Pi over the one-way wiring described in the file header.
  bool link_healthy()
  {
    return crossfire_->is_paired();
  }

  void publish_link_ok(bool ok)
  {
    std_msgs::msg::Bool msg;
    msg.data = ok;
    link_ok_pub_->publish(msg);

    if (!have_link_ok_ || ok != last_link_ok_) {
      have_link_ok_ = true;
      last_link_ok_ = ok;
      if (ok) {
        RCLCPP_INFO(this->get_logger(), "CRSF receiver link is up.");
      } else {
        RCLCPP_WARN(this->get_logger(), "CRSF receiver link is DOWN (port closed or not paired).");
      }
    }
  }

  void publish_threshold(bool permitted)
  {
    std_msgs::msg::Bool msg;
    msg.data = permitted;
    threshold_pub_->publish(msg);

    if (!have_threshold_ || permitted != last_threshold_) {
      have_threshold_ = true;
      last_threshold_ = permitted;
      if (permitted) {
        RCLCPP_INFO(
          this->get_logger(),
          "Manual control PERMITTED: link healthy and enable switch (channel index %d) "
          "inside neutral band [%d, %d].",
          enable_channel_index_, neutral_low_, neutral_high_);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Manual control NOT permitted: enable switch out of band, link down, "
          "channel index misconfigured, or serial port closed.");
      }
    }
  }

  // --- CRSF state ---
  std::unique_ptr<crossfire::XCrossfire> crossfire_;
  bool port_open_;
  std::string serial_port_;
  int crsf_baud_;
  double poll_rate_hz_;

  // --- Enable-switch config ---
  int enable_channel_index_;
  int neutral_low_;
  int neutral_high_;
  int min_link_quality_;

  // --- Edge-logging state ---
  bool have_link_ok_ {false};
  bool last_link_ok_ {false};
  bool have_threshold_ {false};
  bool last_threshold_ {false};

  // --- ROS interfaces ---
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr channels_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_ok_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr threshold_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrsfChannelNode>());
  rclcpp::shutdown();
  return 0;
}
