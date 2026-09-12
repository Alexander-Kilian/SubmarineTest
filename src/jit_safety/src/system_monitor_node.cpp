// system_monitor_node.cpp
//
// The single arbiter. Every other node's opinion about safety arrives here and
// leaves as one fault bitfield and one granted mode.
//
// ---------------------------------------------------------------------------
// WHAT IT PUBLISHES
// ---------------------------------------------------------------------------
//   jit/health      (jit_msgs/Health)  10 Hz - the ground truth. Anything that
//                                      can move the vehicle reads this.
//   jit/mode        (jit_msgs/Mode)    10 Hz - the GRANTED mode.
//   sys/health_ok   (std_msgs/Bool)    20 Hz - the relay permit, on its own
//                                      callback group. See below.
//   led/command     (jit_msgs/LedCommand)   - indicator panel.
//
// ---------------------------------------------------------------------------
// THE TWO SAFING CLASSES
// ---------------------------------------------------------------------------
// The fault set is split by Health::FAULT_SAFE_MASK:
//
//   fault-safe   (in the mask)   -> sys/health_ok goes false -> relay OPENS,
//                                   and vehicle_interface_node disarms
//   mission-safe (outside)       -> relay untouched, vehicle_interface_node
//                                   disarms anyway
//
// Membership of the mask is not a severity judgement. Faults derived from the
// relay's own feedback are excluded because including them deadlocks the
// system: relay open -> feedback low -> ESTOP_OPEN -> sys/health_ok false ->
// relay commanded open -> feedback stays low, forever. Since the vehicle boots
// with the relay open it would never become drivable at all. MISSION_COMPLETE
// is excluded so that a successful run does not cycle the hardware e-stop.
// Everything excluded still sets `ok` false and still forces a disarm.
//
// ---------------------------------------------------------------------------
// WHY sys/health_ok GETS ITS OWN CALLBACK GROUP
// ---------------------------------------------------------------------------
// It is the relay permit. It runs on a MutuallyExclusive callback group of its
// own under a MultiThreadedExecutor, so no amount of work anywhere else in this
// node can delay it. If evaluate_health() has not run for several of its own
// periods, this timer publishes FALSE rather than repeating a stale verdict -
// the monitor reports its own sickness rather than hiding it.
//
// ---------------------------------------------------------------------------
// crsf/link_ok IS USED FOR TWO DIFFERENT THINGS
// ---------------------------------------------------------------------------
//   the VALUE        -> false raises RC_LINK_LOST
//   the ARRIVAL TIME -> crsf_channel_node publishes this topic every poll cycle
//                       regardless of link state, so silence beyond
//                       crsf_heartbeat_timeout_s can only mean that process
//                       died or its executor stalled. That raises CRSF_NODE_DEAD.
//
// What this node explicitly does NOT do is re-run the link latch. That lives in
// crsf_channel_node, next to the relay it gates. This node's use of link_ok is
// only for the software gate and for fault reporting.

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "mavros_msgs/msg/state.hpp"

#include "jit_msgs/msg/estop_status.hpp"
#include "jit_msgs/msg/health.hpp"
#include "jit_msgs/msg/led_command.hpp"
#include "jit_msgs/msg/mode.hpp"
#include "jit_msgs/msg/mode_request.hpp"

using namespace std::chrono_literals;
using Health = jit_msgs::msg::Health;
using Mode = jit_msgs::msg::Mode;
using ModeRequest = jit_msgs::msg::ModeRequest;

namespace
{
const char * mode_name(uint8_t m)
{
  switch (m) {
    case Mode::SAFE: return "SAFE";
    case Mode::MANUAL: return "MANUAL";
    case Mode::LOCAL_GUIDED: return "LOCAL_GUIDED";
    case Mode::GLOBAL_GUIDED: return "GLOBAL_GUIDED";
    default: return "?";
  }
}

// Appends "name " to detail when bit is set. Keeps the string build in one place.
void note(std::string & detail, uint32_t faults, uint32_t bit, const char * name)
{
  if (faults & bit) {
    if (!detail.empty()) {
      detail += " ";
    }
    detail += name;
  }
}
}  // namespace

class SystemMonitorNode : public rclcpp::Node
{
public:
  SystemMonitorNode()
  : Node("system_monitor_node")
  {
    // --- Timeouts ---
    crsf_heartbeat_timeout_s_ =
      this->declare_parameter<double>("crsf_heartbeat_timeout_s", 0.5);
    estop_timeout_s_ = this->declare_parameter<double>("estop_timeout_s", 1.5);
    mavros_timeout_s_ = this->declare_parameter<double>("mavros_timeout_s", 3.0);
    mode_request_timeout_s_ = this->declare_parameter<double>("mode_request_timeout_s", 0.5);

    // How long the active guided mission must sit at COMPLETE before the
    // MISSION_COMPLETE fault is raised and the vehicle mission-safes.
    hold_timeout_s_ = this->declare_parameter<double>("hold_timeout_s", 5.0);

    // --- Rates ---
    evaluate_hz_ = this->declare_parameter<double>("evaluate_hz", 10.0);
    safety_hz_ = this->declare_parameter<double>("safety_hz", 20.0);

    // NOTE-1. Only DISARM is implemented. HOLD and SURFACE are declared so the
    // knob exists, is greppable and shows up in the params file next to
    // everything else, rather than being a bare bool buried in an if.
    //
    // Relaxing this alone changes nothing: a link loss also withdraws
    // crsf/relay_permit inside crsf_channel_node, which opens the relay in
    // hardware. A future autonomy mode that survives link loss needs BOTH this
    // policy and the relay permit relaxed, and only one of them is in software.
    link_loss_action_ = this->declare_parameter<std::string>("link_loss_action", "DISARM");
    if (link_loss_action_ != "DISARM") {
      RCLCPP_ERROR(
        this->get_logger(),
        "link_loss_action:=%s is NOT implemented (only DISARM is). RC_LINK_LOST will "
        "not raise a fault, but crsf_channel_node still opens the relay on link loss, "
        "so the vehicle still loses power. See NOTE-1.",
        link_loss_action_.c_str());
    }

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    link_ok_stamp_ = estop_stamp_ = mavros_stamp_ = mode_req_stamp_ = last_eval_ = t0;

    // --- Callback groups -----------------------------------------------------
    // The safety timer is alone in its group so it cannot be blocked by the
    // evaluation timer or by any subscription callback.
    safety_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    default_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = default_group_;

    // --- Publishers ---
    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    health_pub_ = this->create_publisher<Health>("jit/health", latched);
    mode_pub_ = this->create_publisher<Mode>("jit/mode", latched);
    safety_pub_ = this->create_publisher<std_msgs::msg::Bool>("sys/health_ok", 10);
    led_pub_ = this->create_publisher<jit_msgs::msg::LedCommand>("led/command", latched);

    // --- Subscriptions ---
    link_ok_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "crsf/link_ok", 10,
      std::bind(&SystemMonitorNode::link_ok_cb, this, std::placeholders::_1), sub_opts);

    mode_req_sub_ = this->create_subscription<ModeRequest>(
      "mode/request", 10,
      std::bind(&SystemMonitorNode::mode_request_cb, this, std::placeholders::_1), sub_opts);

    estop_sub_ = this->create_subscription<jit_msgs::msg::EstopStatus>(
      "estop/status", latched,
      std::bind(&SystemMonitorNode::estop_cb, this, std::placeholders::_1), sub_opts);

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&SystemMonitorNode::state_cb, this, std::placeholders::_1), sub_opts);

    nav_local_sub_ = this->create_subscription<std_msgs::msg::String>(
      "nav/local/status", 10,
      [this](const std_msgs::msg::String::SharedPtr m) {nav_local_ = m->data;}, sub_opts);

    nav_global_sub_ = this->create_subscription<std_msgs::msg::String>(
      "nav/global/status", 10,
      [this](const std_msgs::msg::String::SharedPtr m) {nav_global_ = m->data;}, sub_opts);

    // --- Timers ---
    evaluate_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / evaluate_hz_)),
      std::bind(&SystemMonitorNode::evaluate_health, this), default_group_);

    safety_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / safety_hz_)),
      std::bind(&SystemMonitorNode::publish_safety, this), safety_group_);

    RCLCPP_INFO(
      this->get_logger(),
      "system_monitor_node up. Health @ %.0f Hz, relay permit @ %.0f Hz on an isolated "
      "callback group. Mission hold timeout %.1f s. FAULT_SAFE_MASK = 0x%X.",
      evaluate_hz_, safety_hz_, hold_timeout_s_,
      static_cast<unsigned>(Health::FAULT_SAFE_MASK));
  }

private:
  // ---- Subscriptions -------------------------------------------------------

  // Two independent uses of one topic - see the header.
  void link_ok_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    have_link_ok_ = true;
    link_ok_value_ = msg->data;   // used for RC_LINK_LOST
    link_ok_stamp_ = this->now(); // used as crsf_channel_node's heartbeat
  }

  void mode_request_cb(const ModeRequest::SharedPtr msg)
  {
    // NOTE-2: single publisher assumed. msg->source is ignored; when autonomy
    // starts publishing here this needs priority and expiry.
    requested_mode_ = msg->mode;
    mode_req_stamp_ = this->now();
  }

  void estop_cb(const jit_msgs::msg::EstopStatus::SharedPtr msg)
  {
    estop_ = *msg;
    have_estop_ = true;
    estop_stamp_ = this->now();
  }

  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    mavros_connected_ = msg->connected;
    mavros_armed_ = msg->armed;
    mavros_mode_ = msg->mode;
    have_state_ = true;
    mavros_stamp_ = this->now();
  }

  bool fresh(const rclcpp::Time & stamp, double max_age_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= max_age_s;
  }

  // Which nav status topic matters right now. An inactive source is ignored
  // entirely - global_guided_node publishes IDLE forever and must not be able
  // to affect anything.
  const std::string * active_nav_status() const
  {
    switch (granted_mode_) {
      case Mode::LOCAL_GUIDED: return &nav_local_;
      case Mode::GLOBAL_GUIDED: return &nav_global_;
      default: return nullptr;
    }
  }

  // ---- The evaluation tick -------------------------------------------------
  void evaluate_health()
  {
    const rclcpp::Time now = this->now();
    uint32_t faults = 0;

    // --- RC link and the CRSF node's liveness ---
    const bool link_fresh = fresh(link_ok_stamp_, crsf_heartbeat_timeout_s_);
    if (!link_fresh) {
      faults |= Health::CRSF_NODE_DEAD;
    }
    if (!have_link_ok_ || !link_fresh || !link_ok_value_) {
      // A stale topic counts as a lost link too: we cannot claim the link is up
      // on the strength of a value nobody has refreshed.
      if (link_loss_action_ == "DISARM") {
        faults |= Health::RC_LINK_LOST;
      }
    }

    // --- Relay / e-stop ---
    const bool estop_fresh = fresh(estop_stamp_, estop_timeout_s_);
    if (!have_estop_ || !estop_fresh) {
      faults |= Health::ESTOP_NODE_DEAD;
      faults |= Health::ESTOP_OPEN;  // pessimistic: unknown rail state is a dead rail
    } else {
      if (!estop_.feedback) {
        faults |= Health::ESTOP_OPEN;
      }
      if (estop_.mismatch && estop_.commanded && !estop_.feedback) {
        faults |= Health::RELAY_NOT_CLOSED;
      }
      if (estop_.mismatch && !estop_.commanded && estop_.feedback) {
        faults |= Health::RELAY_WELDED;
      }
    }

    // --- MAVROS ---
    if (!have_state_ || !mavros_connected_ || !fresh(mavros_stamp_, mavros_timeout_s_)) {
      faults |= Health::MAVROS_LOST;
    }

    // --- Mission completion (mission-safe, not fault-safe) ---
    const std::string * nav = active_nav_status();
    if (nav != nullptr && *nav == "COMPLETE") {
      if (!mission_complete_since_) {
        mission_complete_since_ = now;
        RCLCPP_INFO(
          this->get_logger(),
          "Guided mission reported COMPLETE. Station-keeping for %.1f s, then mission-safe.",
          hold_timeout_s_);
      }
      if (hold_timeout_s_ > 0.0 &&
        (now - *mission_complete_since_).seconds() >= hold_timeout_s_)
      {
        faults |= Health::MISSION_COMPLETE;
      }
    } else if (nav == nullptr) {
      // Self-clearing: leaving the guided mode clears the stamp and the fault.
      // Without this the first completed mission would latch the vehicle off
      // until reboot.
      if (mission_complete_since_) {
        RCLCPP_INFO(this->get_logger(), "Left the guided mode; MISSION_COMPLETE cleared.");
      }
      mission_complete_since_.reset();
    }

    // --- Split, publish ---
    const uint32_t fault_class = faults & Health::FAULT_SAFE_MASK;
    faults_ = faults;
    last_fault_class_ = fault_class;
    last_eval_ = now;

    Health h;
    h.stamp = now;
    h.faults = faults;
    h.ok = (faults == 0);
    h.detail = describe(faults);
    health_pub_->publish(h);

    if (faults != prev_faults_) {
      if (faults == 0) {
        RCLCPP_INFO(this->get_logger(), "All faults cleared - health OK.");
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Faults: %s%s", h.detail.c_str(),
          (fault_class != 0)
            ? "  [FAULT-SAFE: relay opens]"
            : "  [outside FAULT_SAFE_MASK: relay stays closed, disarm only]");
      }
      prev_faults_ = faults;
    }

    arbitrate_mode(now, fault_class);
    publish_led();
  }

  std::string describe(uint32_t f) const
  {
    if (f == 0) {
      return "ok";
    }
    std::string d;
    note(d, f, Health::RC_LINK_LOST, "RC_LINK_LOST");
    note(d, f, Health::CRSF_NODE_DEAD, "CRSF_NODE_DEAD");
    note(d, f, Health::MAVROS_LOST, "MAVROS_LOST");
    note(d, f, Health::ESTOP_NODE_DEAD, "ESTOP_NODE_DEAD");
    note(d, f, Health::ESTOP_OPEN, "ESTOP_OPEN");
    note(d, f, Health::RELAY_NOT_CLOSED, "RELAY_NOT_CLOSED");
    note(d, f, Health::RELAY_WELDED, "RELAY_WELDED");
    note(d, f, Health::MISSION_COMPLETE, "MISSION_COMPLETE");
    return d;
  }

  void arbitrate_mode(const rclcpp::Time & now, uint32_t fault_class)
  {
    // NOTE-2: one publisher today, so the request is granted as-is unless a
    // fault-safe fault forces SAFE.
    uint8_t requested = Mode::SAFE;
    if (fresh(mode_req_stamp_, mode_request_timeout_s_)) {
      requested = requested_mode_;
    }

    const uint8_t granted = (fault_class != 0) ? Mode::SAFE : requested;

    // MISSION_COMPLETE deliberately does NOT force SAFE here. The switch still
    // describes the mode, so the LED and the operator's mental model stay
    // consistent; the mission-safe action is taken by vehicle_interface_node
    // reading the fault bit.

    if (granted != granted_mode_) {
      RCLCPP_INFO(
        this->get_logger(), "Granted mode: %s -> %s (requested %s).",
        mode_name(granted_mode_), mode_name(granted), mode_name(requested));
      granted_mode_ = granted;
    }

    Mode m;
    m.stamp = now;
    m.mode = granted_mode_;
    mode_pub_->publish(m);
  }

  // The panel shows the physical state of the motor rail first, and the mode
  // second. Red therefore means "the rail is dead" whatever the switch says -
  // which is the thing an operator needs to read from across a pool deck.
  void publish_led()
  {
    std::string pattern;
    if (faults_ & Health::ESTOP_OPEN) {
      pattern = jit_msgs::msg::LedCommand::ESTOP;
    } else if (granted_mode_ == Mode::MANUAL) {
      pattern = jit_msgs::msg::LedCommand::MANUAL;
    } else if (granted_mode_ == Mode::LOCAL_GUIDED || granted_mode_ == Mode::GLOBAL_GUIDED) {
      pattern = jit_msgs::msg::LedCommand::AUTO;
    } else {
      pattern = jit_msgs::msg::LedCommand::ESTOP;
    }

    if (pattern == last_pattern_) {
      return;
    }
    last_pattern_ = pattern;

    jit_msgs::msg::LedCommand msg;
    msg.stamp = this->now();
    msg.pattern = pattern;
    led_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "LED panel -> %s.", pattern.c_str());
  }

  // ---- The relay permit, isolated -----------------------------------------
  void publish_safety()
  {
    std_msgs::msg::Bool msg;

    // If the evaluation tick has stopped, do not keep repeating its last
    // verdict. Report the monitor's own sickness instead.
    const double stale_limit = 4.0 / evaluate_hz_;
    if (!fresh(last_eval_, stale_limit)) {
      msg.data = false;
      safety_pub_->publish(msg);
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "evaluate_health() has not run for > %.2f s - publishing sys/health_ok=false. "
        "The relay will open.", stale_limit);
      return;
    }

    msg.data = (last_fault_class_ == 0);
    safety_pub_->publish(msg);
  }

  // --- Config ---
  double crsf_heartbeat_timeout_s_ {0.5};
  double estop_timeout_s_ {1.5};
  double mavros_timeout_s_ {3.0};
  double mode_request_timeout_s_ {0.5};
  double hold_timeout_s_ {5.0};
  double evaluate_hz_ {10.0};
  double safety_hz_ {20.0};
  std::string link_loss_action_ {"DISARM"};

  // --- Inputs ---
  bool have_link_ok_ {false};
  bool link_ok_value_ {false};
  rclcpp::Time link_ok_stamp_;

  uint8_t requested_mode_ {Mode::SAFE};
  rclcpp::Time mode_req_stamp_;

  bool have_estop_ {false};
  jit_msgs::msg::EstopStatus estop_;
  rclcpp::Time estop_stamp_;

  bool have_state_ {false};
  bool mavros_connected_ {false};
  bool mavros_armed_ {false};
  std::string mavros_mode_;
  rclcpp::Time mavros_stamp_;

  std::string nav_local_ {"IDLE"};
  std::string nav_global_ {"IDLE"};

  // --- Derived state ---
  uint32_t faults_ {0};
  uint32_t prev_faults_ {0xFFFFFFFFu};   // force a log on the first evaluation
  uint32_t last_fault_class_ {Health::FAULT_SAFE_MASK};  // start denied
  rclcpp::Time last_eval_;
  uint8_t granted_mode_ {Mode::SAFE};
  std::optional<rclcpp::Time> mission_complete_since_;
  std::string last_pattern_;

  // --- ROS interfaces ---
  rclcpp::CallbackGroup::SharedPtr safety_group_;
  rclcpp::CallbackGroup::SharedPtr default_group_;
  rclcpp::Publisher<Health>::SharedPtr health_pub_;
  rclcpp::Publisher<Mode>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_pub_;
  rclcpp::Publisher<jit_msgs::msg::LedCommand>::SharedPtr led_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr link_ok_sub_;
  rclcpp::Subscription<ModeRequest>::SharedPtr mode_req_sub_;
  rclcpp::Subscription<jit_msgs::msg::EstopStatus>::SharedPtr estop_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_local_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_global_sub_;
  rclcpp::TimerBase::SharedPtr evaluate_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemMonitorNode>();
  // MultiThreadedExecutor so the isolated safety callback group actually gets
  // its own thread. On a SingleThreadedExecutor the group buys nothing.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
