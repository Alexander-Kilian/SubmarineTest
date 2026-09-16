// crsf_channel_node.cpp
//
// Reads CRSF/ELRS channel data from a T8L-paired receiver via the xcrsf
// library. This node is the sensor at the top of the stack and it publishes
// four things, every poll cycle, without exception:
//
//   - crsf/channels      (std_msgs/UInt16MultiArray)  all 16 raw channels
//   - crsf/link_ok       (std_msgs/Bool)              receiver link healthy
//   - crsf/relay_permit  (std_msgs/Bool)              hardware e-stop permit
//   - mode/request       (jit_msgs/ModeRequest)       requested operating mode
//
// PUBLISH-ALWAYS CONTRACT. Every one of those topics is published on every
// timer tick regardless of link state. Two consumers depend on that:
// gpio_estop_node treats stale crsf/relay_permit as "open the relay", and
// system_monitor_node treats stale crsf/link_ok as CRSF_NODE_DEAD. Silence from
// this node must always mean "this node died", never "nothing to report".
//
// WHY THE LINK LATCH LIVES HERE. crsf/relay_permit goes straight to
// gpio_estop_node with no node in between, because the path from the operator's
// switch to the motor relay should be as short as the software can make it. The
// link state machine that decides the permit therefore has to live in this node
// too - moving it to the health monitor would put a second process in that
// path. system_monitor_node consumes crsf/link_ok raw and applies its own,
// separate policy for the software gate.
//
// CHANNEL 8 - THE 3-POSITION SWITCH. One switch, three detents, and it is
// currently doing double duty as both the mode selector and the e-stop:
//
//   DOWN  (raw ~191)   -> ESTOP        : relay permit false, mode SAFE
//   MID   (raw ~997)   -> MANUAL       : relay permit true
//   UP    (raw ~1792)  -> LOCAL_GUIDED : relay permit true
//
// GLOBAL_GUIDED has no detent. It is reachable only by editing kDetentModes
// below, which is deliberate: the GPS is not connected and that mode must not
// be selectable by accident at the pool.
//
// WIRING NOTE: on the real vehicle the link to the ELRS receiver is ONE-WAY.
// Only the receiver's TX line is wired to the Pi's UART RX (GPIO15); the Pi's
// UART TX (GPIO14) is NOT connected (space constraints on the penetrator). The
// Pi cannot transmit to the receiver at all - no telemetry uplink, no CRSF
// config. That is fine for reading channels: the receiver still streams the
// RC-channels frame and the link-statistics frame down that single wire.
//
// LINK LOSS / RECONNECT (trip fast, recover slow):
//   - Any loss of link -> crsf/relay_permit goes false immediately, no debounce.
//   - xcrsf does not self-heal after a serial dropout (is_paired() stays false),
//     so on a sustained loss the node destroys and recreates the XCrossfire
//     object and re-opens the port, ~1 Hz, until the link returns.
//   - LOST_TRANSIENT: the link being back is not enough - it must be healthy
//     continuously for RECOVER_HOLD_S before the permit is restored. This
//     debounces a flapping link.
//   - LATCHED: if the link stays down longer than LATCH_TIMEOUT_S the permit is
//     latched off. Clearing it requires the operator to move the switch to the
//     ESTOP detent and back, with a live link - a deliberate acknowledgement.
//     (A crash + respawn loses this latch; that edge case is accepted and
//     handled by the operator.)
//
// Moving the switch to the ESTOP detent by hand is the normal manual e-stop:
// instant false, instant clear, NO latch. Only link loss latches.
//
// Verified against the installed header (/usr/local/include/xcrsf/crossfire.h):
//   XCrossfire(uart_path, speed_t baud=420000), open_port(), close_port(),
//   is_paired(), get_channel_state() -> std::array<uint16_t, 16>, get_link_state().

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

#include "jit_msgs/msg/mode.hpp"
#include "jit_msgs/msg/mode_request.hpp"

#include "xcrsf/crossfire.h"

using namespace std::chrono_literals;

namespace
{
// Link-loss / reconnect timing. Hardcoded on purpose - tune here and rebuild.
constexpr double RECONNECT_GRACE_S = 0.5;   // link bad this long before the first object-recreate
constexpr double RECONNECT_PERIOD_S = 1.0;  // between recreate attempts
constexpr double RECOVER_HOLD_S = 1.0;      // link healthy continuously before the permit is restored
constexpr double LATCH_TIMEOUT_S = 5.0;     // link bad continuously -> LATCHED
}  // namespace

class CrsfChannelNode : public rclcpp::Node
{
public:
  CrsfChannelNode()
  : Node("crsf_channel_node"), port_open_(false)
  {
    // --- Parameters ---
    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    poll_rate_hz_ = this->declare_parameter<double>("poll_rate_hz", 50.0);

    // CRSF UART baud rate. Passed to XCrossfire (whose own default is 420000,
    // the CRSF standard). This default matches the rate currently configured on
    // the ELRS receiver - keep the two in sync, or frames never decode and
    // is_paired() stays false. xcrsf sets non-standard rates via termios2.
    crsf_baud_ = this->declare_parameter<int>("crsf_baud", 115200);

    // Mode switch: the 3-position switch. 0-based index into the 16-channel
    // CRSF array. Default 7 (i.e. "channel 8" as counted on the transmitter).
    mode_channel_index_ = this->declare_parameter<int>("mode_channel_index", 7);

    // Detent bands, in raw CRSF counts. Detents observed at ~191 / ~997 / ~1792,
    // so [700, 1300] for MID leaves roughly 500 counts of margin either side.
    // Below neutral_low is DOWN, above neutral_high is UP.
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
    permit_pub_ = this->create_publisher<std_msgs::msg::Bool>("crsf/relay_permit", 10);
    mode_pub_ = this->create_publisher<jit_msgs::msg::ModeRequest>("mode/request", 10);

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
        "Publishing crsf/relay_permit=false until it recovers.",
        serial_port_.c_str(), crsf_baud_, 1.0 / RECONNECT_PERIOD_S);
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Link-loss detection relies on xcrsf is_paired() plus a first-frame-decoded "
      "check. Ensure the ELRS receiver failsafe is set to 'no pulses'. TODO: enforce "
      "min_link_quality (=%d) via get_link_state() so a receiver that HOLDS last "
      "values is also caught.",
      min_link_quality_);

    RCLCPP_INFO(
      this->get_logger(),
      "Mode switch = channel index %d. DOWN(<%d)=ESTOP, MID=MANUAL, UP(>%d)=LOCAL_GUIDED. "
      "Poll = %.1f Hz. Link loss latches after %.0f s; clear by moving the switch to the "
      "ESTOP detent and back.",
      mode_channel_index_, neutral_low_, neutral_high_, poll_rate_hz_, LATCH_TIMEOUT_S);

    // --- Poll timer ---
    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CrsfChannelNode::poll_callback, this));
  }

private:
  enum class LinkFsm { OK, LOST_TRANSIENT, LATCHED };
  enum class Detent { UNKNOWN, DOWN, MID, UP };

  static const char * fsm_name(LinkFsm s)
  {
    switch (s) {
      case LinkFsm::OK: return "OK";
      case LinkFsm::LOST_TRANSIENT: return "LOST_TRANSIENT";
      case LinkFsm::LATCHED: return "LATCHED";
    }
    return "?";
  }

  static const char * detent_name(Detent d)
  {
    switch (d) {
      case Detent::UNKNOWN: return "unknown";
      case Detent::DOWN: return "DOWN/ESTOP";
      case Detent::MID: return "MID/MANUAL";
      case Detent::UP: return "UP/LOCAL_GUIDED";
    }
    return "?";
  }

  // The detent -> mode table. GLOBAL_GUIDED is deliberately absent: change the
  // UP entry here and rebuild if you want to bench-test it. See the header.
  //
  // The values come from Mode.msg, which is the single definition of the mode
  // enum; ModeRequest carries one of them in its `mode` field.
  static uint8_t detent_mode(Detent d)
  {
    switch (d) {
      case Detent::MID: return jit_msgs::msg::Mode::MANUAL;
      case Detent::UP: return jit_msgs::msg::Mode::LOCAL_GUIDED;
      case Detent::DOWN:
      case Detent::UNKNOWN:
      default: return jit_msgs::msg::Mode::SAFE;
    }
  }

  void poll_callback()
  {
    const rclcpp::Time now = this->now();

    // 1. Evaluate the link, recreating the xcrsf object on a sustained loss.
    bool link = link_up();
    if (!link) {
      if (!link_bad_since_) {
        link_bad_since_ = now;
      }
      const double bad_for = (now - *link_bad_since_).seconds();
      const double since_attempt = (now - last_reconnect_attempt_).seconds();
      if (bad_for >= RECONNECT_GRACE_S && since_attempt >= RECONNECT_PERIOD_S) {
        attempt_reconnect(now);
        link = link_up();
      }
    } else {
      link_bad_since_.reset();
    }
    publish_link_ok(link);

    // 2. Read the switch detent (only meaningful with a live link). channels_
    // was refreshed by link_up(), which only returns true once a real frame has
    // been decoded, so these values are never the zero-initialised array.
    Detent detent = Detent::UNKNOWN;
    if (link) {
      std_msgs::msg::UInt16MultiArray channels_msg;
      channels_msg.data.assign(channels_.begin(), channels_.end());
      channels_pub_->publish(channels_msg);

      if (mode_channel_index_ < 0 ||
        static_cast<size_t>(mode_channel_index_) >= channels_.size())
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "mode_channel_index %d is out of range [0, %zu); treating the switch as "
          "unknown, which denies the relay permit. Set the parameter to your switch channel.",
          mode_channel_index_, channels_.size());
      } else {
        detent = classify(channels_[mode_channel_index_]);
      }
    }

    // 3. Run the link state machine, then publish the permit and the mode request.
    update_link_fsm(now, link, detent);

    // The entire permit expression. It deliberately ignores mode, MAVROS,
    // mission state and jit/health: flipping the switch must cut power without
    // consulting anything that could be busy or dead.
    const bool permitted = (link_fsm_ == LinkFsm::OK) && (detent != Detent::DOWN) &&
      (detent != Detent::UNKNOWN);

    publish_permit(permitted, detent);
    publish_mode_request(now, detent);
  }

  Detent classify(uint16_t raw) const
  {
    if (raw < static_cast<uint16_t>(neutral_low_)) {
      return Detent::DOWN;
    }
    if (raw > static_cast<uint16_t>(neutral_high_)) {
      return Detent::UP;
    }
    return Detent::MID;
  }

  void update_link_fsm(const rclcpp::Time & now, bool link, Detent detent)
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
          ack_estop_seen_ = false;
          link_fsm_ = LinkFsm::LOST_TRANSIENT;
        }
        break;

      case LinkFsm::LOST_TRANSIENT:
        if (link_stable) {
          link_fsm_ = LinkFsm::OK;
        } else if ((now - link_lost_at_).seconds() >= LATCH_TIMEOUT_S) {
          // Clear the acknowledgement on ENTRY to LATCHED, not only on the
          // OK -> LOST_TRANSIENT edge. The flag is sticky, so anything that set
          // it earlier in this loss event would otherwise carry in and leave the
          // latch pre-cleared the instant it engaged.
          ack_estop_seen_ = false;
          link_fsm_ = LinkFsm::LATCHED;
        }
        break;

      case LinkFsm::LATCHED:
        // Operator acknowledgement: move to the ESTOP detent, then back off it,
        // with a live and stable link. The ESTOP detent specifically - not any
        // extreme - because acknowledging a fault by selecting a guided mode
        // would be a genuinely bad gesture to build in.
        if (link && detent == Detent::DOWN) {
          ack_estop_seen_ = true;
        }
        if (link_stable && ack_estop_seen_ && detent == Detent::MID) {
          link_fsm_ = LinkFsm::OK;
        }
        break;
    }

    if (link_fsm_ != prev) {
      switch (link_fsm_) {
        case LinkFsm::OK:
          RCLCPP_INFO(
            this->get_logger(), "Link FSM: %s -> OK (link stable %.1f s). Relay permit restored.",
            fsm_name(prev), RECOVER_HOLD_S);
          break;
        case LinkFsm::LOST_TRANSIENT:
          RCLCPP_WARN(
            this->get_logger(),
            "Link FSM: OK -> LOST_TRANSIENT. Relay permit withdrawn; recreating the CRSF "
            "link. Auto-recovers if healthy again within %.0f s, otherwise latches.",
            LATCH_TIMEOUT_S);
          break;
        case LinkFsm::LATCHED:
          RCLCPP_ERROR(
            this->get_logger(),
            "Link FSM: LOST_TRANSIENT -> LATCHED (link down > %.0f s). Relay permit held "
            "off. Move the switch to the ESTOP detent (DOWN) and back to MID to re-enable.",
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

    // New object, so nothing has been decoded through it yet. Until a frame
    // arrives its channel array is all zeros, which must not be read as a
    // detent - see link_up().
    frame_seen_ = false;
    channels_.fill(0);

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
  // the ELRS receiver is configured to HOLD the last channel values on TX loss
  // instead of stopping frames - is_paired() can stay true even though the
  // transmitter is gone.
  bool link_healthy()
  {
    return crossfire_->is_paired();
  }

  // The full link test: port open, xcrsf reports paired, AND at least one RC
  // frame has actually been decoded since the current XCrossfire was built.
  // Refreshes channels_ as a side effect, so poll_callback() does not re-read.
  //
  // That last clause is the one that matters. open_port() sets the paired flag
  // true immediately, before any frame arrives, so on the poll tick right after
  // attempt_reconnect() the link looks healthy while get_channel_state() still
  // returns the zero-initialised array - and classify(0) is DOWN. Every
  // reconnect attempt therefore used to synthesise a phantom ESTOP detent,
  // which forged the operator acknowledgement that clears LinkFsm::LATCHED: a
  // 5 s link loss would latch and then release itself ~0.6 s later, with the
  // relay closing again the moment the transmitter returned. Observed on the
  // bench 2026-09-16.
  //
  // An all-zero array therefore means "no frames yet", not "sticks at zero".
  // frame_seen_ sticks once set (cleared only by attempt_reconnect), so a
  // legitimate all-zero reading later can never be misread as a dead link.
  // Note this cannot be done by watching the channels for CHANGE: with the
  // transmitter idle the values are stable, so change-detection would
  // false-trip on a healthy link.
  bool link_up()
  {
    if (!port_open_ || !link_healthy()) {
      return false;
    }

    channels_ = crossfire_->get_channel_state();

    if (!frame_seen_) {
      frame_seen_ = std::any_of(
        channels_.begin(), channels_.end(), [](uint16_t v) {return v != 0;});
      if (!frame_seen_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "CRSF port open and xcrsf reports paired, but no RC frame decoded yet - "
          "holding the link DOWN and the switch UNKNOWN.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "First CRSF frame decoded; link is real.");
    }

    return true;
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

  void publish_permit(bool permitted, Detent detent)
  {
    std_msgs::msg::Bool msg;
    msg.data = permitted;
    permit_pub_->publish(msg);

    if (!have_permit_ || permitted != last_permit_) {
      have_permit_ = true;
      last_permit_ = permitted;
      if (permitted) {
        RCLCPP_INFO(
          this->get_logger(),
          "Relay permit GRANTED: link FSM OK and switch at %s.", detent_name(detent));
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Relay permit WITHDRAWN: link FSM = %s, switch = %s.",
          fsm_name(link_fsm_), detent_name(detent));
      }
    }
  }

  void publish_mode_request(const rclcpp::Time & now, Detent detent)
  {
    jit_msgs::msg::ModeRequest msg;
    msg.stamp = now;
    msg.mode = detent_mode(detent);
    msg.source = jit_msgs::msg::ModeRequest::SOURCE_RC;
    mode_pub_->publish(msg);

    if (!have_detent_ || detent != last_detent_) {
      have_detent_ = true;
      last_detent_ = detent;
      RCLCPP_INFO(this->get_logger(), "Switch detent -> %s.", detent_name(detent));
    }
  }

  // --- CRSF state ---
  std::unique_ptr<crossfire::XCrossfire> crossfire_;
  bool port_open_;
  std::string serial_port_;
  int crsf_baud_;
  double poll_rate_hz_;

  // Last channel array read by link_up(), and whether any frame has been
  // decoded through the current XCrossfire at all. See link_up().
  std::array<uint16_t, 16> channels_ {};
  bool frame_seen_ {false};

  // --- Switch config ---
  int mode_channel_index_;
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
  bool ack_estop_seen_ {false};

  // --- Edge-logging state ---
  bool have_link_ok_ {false};
  bool last_link_ok_ {false};
  bool have_permit_ {false};
  bool last_permit_ {false};
  bool have_detent_ {false};
  Detent last_detent_ {Detent::UNKNOWN};

  // --- ROS interfaces ---
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr channels_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_ok_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permit_pub_;
  rclcpp::Publisher<jit_msgs::msg::ModeRequest>::SharedPtr mode_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrsfChannelNode>());
  rclcpp::shutdown();
  return 0;
}
