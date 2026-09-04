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
//   1. the link state machine is OK, and
//   2. the enable channel (the 3-position switch, channel index
//      `enable_channel_index`) is inside the neutral centre band.
//
// LINK LOSS / RECONNECT (trip fast, recover slow):
//   - Any loss of link -> crsf/channel_threshold goes false immediately.
//   - xcrsf does not self-heal after a serial dropout (is_paired() stays
//     false), so on a sustained loss the node destroys and recreates the
//     XCrossfire object and re-opens the port, ~1 Hz, until the link returns.
//   - LinkFsm::LOST_TRANSIENT: link is back is not enough - it must be healthy
//     continuously for RECOVER_HOLD_S before the permit is restored. This
//     debounces a flapping link.
//   - LinkFsm::LATCHED: if the link stays down longer than LATCH_TIMEOUT_S the
//     permit is latched off. Clearing it requires the operator to move the
//     enable switch to an extreme (e-stop) detent and back to centre, with a
//     live link - a deliberate acknowledgement. (A crash+respawn loses this
//     latch; that edge case is accepted and handled by the operator.)
//
// Moving the switch out of the centre band by hand is the normal manual
// e-stop: instant false, instant clear, NO latch. Only link loss latches.
//
// crsf/channel_threshold is published on EVERY poll cycle - the node never
// goes silent while running, so a downstream consumer that stops seeing
// messages can treat that as "this node died" (see the watchdog in
// gpio_estop_node and the enable-timeout in manual_control_node).
//
// A separate node (gpio_estop_node) subscribes to crsf/channel_threshold and
// drives the relay GPIO. "Decode CRSF" and "touch hardware GPIO" stay as two
// independent, individually testable nodes. The link FSM / latch policy will
// move to a dedicated health-monitor node in a later refactor.
//
// Verified against the real installed header (/usr/local/include/xcrsf/crossfire.h):
//   XCrossfire(uart_path, speed_t baud=420000), open_port(), close_port(),
//   is_paired(), get_channel_state() -> std::array<uint16_t, 16>, get_link_state().

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "xcrsf/crossfire.h"

using namespace std::chrono_literals;

namespace
{
// Link-loss / reconnect timing. Hardcoded on purpose - tune here and rebuild.
constexpr double RECONNECT_GRACE_S = 0.5;   // link bad this long before the first object-recreate
constexpr double RECONNECT_PERIOD_S = 1.0;  // between recreate attempts
constexpr double RECOVER_HOLD_S = 1.0;      // link healthy continuously before the permit is restored
constexpr double LATCH_TIMEOUT_S = 5.0;     // link bad continuously -> LATCHED (operator switch toggle to clear)
}  // namespace

class CrsfChannelNode : public rclcpp::Node
{
public:
  CrsfChannelNode()
  : Node("crsf_channel_node"), port_open_(false)
  {
    // --- Parameters (set via a launch file or `ros2 run ... --ros-args -p`) ---
    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    poll_rate_hz_ = this->declare_parameter<double>("poll_rate_hz", 50.0);

    // CRSF UART baud rate. Passed to XCrossfire (whose own default is 420000,
    // the CRSF standard). This default matches the rate currently configured
    // on the ELRS receiver - keep the two in sync, or frames never decode and
    // is_paired() stays false. xcrsf sets non-standard rates via termios2.
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

    // --- ROS time init (so age checks are well-defined on the first poll) ---
    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_reconnect_attempt_ = t0;
    link_lost_at_ = t0;

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
        "Failed to open CRSF port '%s' at %d baud - will recreate and retry ~%.1f Hz. "
        "Publishing crsf/channel_threshold=false until it recovers.",
        serial_port_.c_str(), crsf_baud_, 1.0 / RECONNECT_PERIOD_S);
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Link-loss detection currently relies on xcrsf is_paired() only. Ensure the "
      "ELRS receiver failsafe is set to 'no pulses'. TODO: enforce min_link_quality "
      "(=%d) via get_link_state() so a receiver that HOLDS last values is also caught.",
      min_link_quality_);

    RCLCPP_INFO(
      this->get_logger(),
      "Enable switch = channel index %d; neutral band = [%d, %d] counts; poll = %.1f Hz. "
      "Link loss latches after %.0f s; clear by toggling the switch to an extreme and back.",
      enable_channel_index_, neutral_low_, neutral_high_, poll_rate_hz_, LATCH_TIMEOUT_S);

    // --- Poll timer ---
    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CrsfChannelNode::poll_callback, this));
  }

private:
  enum class LinkFsm { OK, LOST_TRANSIENT, LATCHED };
  enum class SwitchZone { UNKNOWN, CENTER, EXTREME };

  static const char * fsm_name(LinkFsm s)
  {
    switch (s) {
      case LinkFsm::OK: return "OK";
      case LinkFsm::LOST_TRANSIENT: return "LOST_TRANSIENT";
      case LinkFsm::LATCHED: return "LATCHED";
    }
    return "?";
  }
  static const char * zone_name(SwitchZone z)
  {
    switch (z) {
      case SwitchZone::UNKNOWN: return "unknown";
      case SwitchZone::CENTER: return "centre";
      case SwitchZone::EXTREME: return "extreme";
    }
    return "?";
  }

  void poll_callback()
  {
    const rclcpp::Time now = this->now();

    // 1. Evaluate the link, recreating the xcrsf object on a sustained loss.
    bool link = port_open_ && link_healthy();
    if (!link) {
      if (!link_bad_since_) {
        link_bad_since_ = now;
      }
      const double bad_for = (now - *link_bad_since_).seconds();
      const double since_attempt = (now - last_reconnect_attempt_).seconds();
      if (bad_for >= RECONNECT_GRACE_S && since_attempt >= RECONNECT_PERIOD_S) {
        attempt_reconnect(now);
        link = port_open_ && link_healthy();
      }
    } else {
      link_bad_since_.reset();
    }
    publish_link_ok(link);

    // 2. Read the switch position (only meaningful with a live link).
    SwitchZone zone = SwitchZone::UNKNOWN;
    if (link) {
      const std::array<uint16_t, 16> channels = crossfire_->get_channel_state();
      std_msgs::msg::UInt16MultiArray channels_msg;
      channels_msg.data.assign(channels.begin(), channels.end());
      channels_pub_->publish(channels_msg);

      if (enable_channel_index_ < 0 ||
        static_cast<size_t>(enable_channel_index_) >= channels.size())
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "enable_channel_index %d is out of range [0, %zu); treating the switch as "
          "not centred. Set the parameter to your switch channel.",
          enable_channel_index_, channels.size());
      } else {
        const uint16_t value = channels[enable_channel_index_];
        zone = (value >= static_cast<uint16_t>(neutral_low_) &&
                value <= static_cast<uint16_t>(neutral_high_))
          ? SwitchZone::CENTER
          : SwitchZone::EXTREME;
      }
    }

    // 3. Run the link state machine and publish the permit.
    update_link_fsm(now, link, zone);
    const bool permitted = (link_fsm_ == LinkFsm::OK) && (zone == SwitchZone::CENTER);
    publish_threshold(permitted, zone);
  }

  void update_link_fsm(const rclcpp::Time & now, bool link, SwitchZone zone)
  {
    // Continuous-healthy streak, for the "recover slow" debounce.
    if (link) {
      if (!link_good_since_) {
        link_good_since_ = now;
      }
    } else {
      link_good_since_.reset();
    }
    const bool link_stable = link_good_since_ &&
      (now - *link_good_since_).seconds() >= RECOVER_HOLD_S;

    const LinkFsm prev = link_fsm_;

    switch (link_fsm_) {
      case LinkFsm::OK:
        if (!link) {
          link_lost_at_ = now;
          ack_extreme_seen_ = false;
          link_fsm_ = LinkFsm::LOST_TRANSIENT;
        }
        break;

      case LinkFsm::LOST_TRANSIENT:
        if (link_stable) {
          link_fsm_ = LinkFsm::OK;
        } else if ((now - link_lost_at_).seconds() >= LATCH_TIMEOUT_S) {
          link_fsm_ = LinkFsm::LATCHED;
        }
        break;

      case LinkFsm::LATCHED:
        // Operator acknowledgement: switch to an extreme detent, then back to
        // centre, with a live and stable link.
        if (link && zone == SwitchZone::EXTREME) {
          ack_extreme_seen_ = true;
        }
        if (link_stable && ack_extreme_seen_ && zone == SwitchZone::CENTER) {
          link_fsm_ = LinkFsm::OK;
        }
        break;
    }

    if (link_fsm_ != prev) {
      switch (link_fsm_) {
        case LinkFsm::OK:
          RCLCPP_INFO(
            this->get_logger(), "Link FSM: %s -> OK (link stable %.1f s). Permit restored.",
            fsm_name(prev), RECOVER_HOLD_S);
          break;
        case LinkFsm::LOST_TRANSIENT:
          RCLCPP_WARN(
            this->get_logger(),
            "Link FSM: OK -> LOST_TRANSIENT. E-stop asserted; recreating the CRSF link. "
            "Auto-recovers if healthy again within %.0f s, otherwise latches.",
            LATCH_TIMEOUT_S);
          break;
        case LinkFsm::LATCHED:
          RCLCPP_ERROR(
            this->get_logger(),
            "Link FSM: LOST_TRANSIENT -> LATCHED (link down > %.0f s). E-stop held. "
            "Toggle the enable switch to an extreme detent and back to centre to re-enable.",
            LATCH_TIMEOUT_S);
          break;
      }
    }
  }

  void attempt_reconnect(const rclcpp::Time & now)
  {
    last_reconnect_attempt_ = now;
    ++reconnect_count_;
    RCLCPP_WARN(
      this->get_logger(),
      "Recreating XCrossfire on '%s' @ %d baud (reconnect attempt %lu).",
      serial_port_.c_str(), crsf_baud_, reconnect_count_);

    // Assumption (bench-verify): close_port() is safe to call here and
    // ~XCrossfire() releases the fd and stops any internal reader thread, so a
    // freshly constructed object + open_port() fully re-initialises the library.
    if (crossfire_) {
      (void)crossfire_->close_port();
    }
    crossfire_.reset();
    crossfire_ = std::make_unique<crossfire::XCrossfire>(
      serial_port_, static_cast<speed_t>(crsf_baud_));
    port_open_ = crossfire_->open_port();

    if (port_open_) {
      RCLCPP_INFO(this->get_logger(), "CRSF port reopened; waiting for the receiver link.");
    } else {
      RCLCPP_WARN(
        this->get_logger(), "CRSF port reopen failed; retrying in ~%.1f s.", RECONNECT_PERIOD_S);
    }
  }

  // Returns true when the receiver link is considered healthy.
  //
  // TODO(link-quality): once the return type of crossfire_->get_link_state()
  // in /usr/local/include/xcrsf/crossfire.h is known, additionally require its
  // link-quality / RSSI to be >= min_link_quality_. That covers the case where
  // the ELRS receiver is configured to HOLD the last channel values (or output
  // configured failsafe positions) on TX loss instead of stopping frames - in
  // that case is_paired() can stay true even though the transmitter is gone.
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

  void publish_threshold(bool permitted, SwitchZone zone)
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
          "Manual control PERMITTED: link FSM OK and enable switch (index %d) in "
          "neutral band [%d, %d].",
          enable_channel_index_, neutral_low_, neutral_high_);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Manual control NOT permitted: link FSM = %s, switch = %s.",
          fsm_name(link_fsm_), zone_name(zone));
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

  // --- Reconnect / link FSM state ---
  // Start OK: the first bad poll drives OK -> LOST_TRANSIENT cleanly (and stamps
  // link_lost_at_). Starting in LOST_TRANSIENT would need link_lost_at_ primed
  // or it would latch immediately.
  LinkFsm link_fsm_ {LinkFsm::OK};
  std::optional<rclcpp::Time> link_good_since_;
  std::optional<rclcpp::Time> link_bad_since_;
  rclcpp::Time link_lost_at_;
  rclcpp::Time last_reconnect_attempt_;
  unsigned long reconnect_count_ {0};
  bool ack_extreme_seen_ {false};

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
