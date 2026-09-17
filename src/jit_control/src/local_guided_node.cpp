// local_guided_node.cpp
//
// Waypoint tracking against the DVL-backed local pose, for ArduSub GUIDED mode.
// Publishes cmd/local_guided/setpoint (geometry_msgs/PoseStamped), which
// vehicle_interface_node forwards to /mavros/setpoint_position/local. This node
// holds no MAVROS client and cannot arm anything.
//
// ---------------------------------------------------------------------------
// WHY ACCEPTANCE LOGIC LIVES HERE
// ---------------------------------------------------------------------------
// ArduSub's GUIDED mode never declares a waypoint reached. Verified in the
// firmware source: guided_pos_control_run() calls wp_nav.update_wpnav() and
// never checks reached_wp_destination(). A position target in GUIDED is a
// station to hold, not a task to complete - the vehicle drives to it and holds
// there indefinitely. Only AUTO mode sequences waypoints, and AUTO needs
// lat/lon mission items and an EKF origin that can be set only once per power
// cycle, which does not fit the capture-a-datum-at-mode-entry approach below.
//
// So we do the acceptance test ourselves, deliberately mirroring ArduPilot's
// verify_nav_wp():
//
//   1. once the horizontal distance to the target is <= radius_m, latch
//      "reached" and stamp the time;
//   2. wait dwell_s from that stamp, then advance.
//
// The distance is NOT re-checked during the dwell - that matches the firmware,
// and it is the reason a tight radius is workable at all: the vehicle only has
// to touch the ball once, not stay inside it.
//
// A second consequence of the missing timeout matters for safety: stopping the
// setpoint stream does NOT stop the vehicle. Only leaving GUIDED does. That is
// vehicle_interface_node's job during safing.
//
// ---------------------------------------------------------------------------
// DATUM, DEPTH AND HEADING
// ---------------------------------------------------------------------------
// On entry to LOCAL_GUIDED the node captures the current pose as the datum and
// the current depth as entry_z. Every waypoint is an OFFSET from that datum, so
// estimator drift accumulated since boot is absorbed at the moment the mission
// starts rather than being baked into absolute coordinates.
//
// Offsets are along the local ENU frame's axes (x = East, y = North), NOT
// rotated into the vehicle's heading at entry. A waypoint of {x: 2, y: 0} is
// two metres East of wherever the sub was, whichever way it was pointing.
//
// z defaults to entry_z - the vehicle holds the depth it started at, so it does
// not fight the trim changes seen during testing. A waypoint may override it
// with its own `z`, which is also an offset (negative = deeper, ENU). That
// override is how the deliberate link-cut test is expressed.
//
// Heading points along track: the bearing from the current position to the
// target. Within yaw_hold_radius_m of the target the bearing becomes noisy, so
// the last heading is held instead.
//
// ---------------------------------------------------------------------------
// RE-ENTRY
// ---------------------------------------------------------------------------
// Leaving and re-entering LOCAL_GUIDED always recaptures the datum and always
// restarts at index 0. The operator's re-run gesture is therefore a single
// switch cycle. This is intentional for single-waypoint testing; it does mean
// that intervening mid-mission and returning walks the whole pattern.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "jit_msgs/msg/mode.hpp"

#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;
using Mode = jit_msgs::msg::Mode;

namespace
{
struct Waypoint
{
  double x {0.0};        // offset East from the datum, metres
  double y {0.0};        // offset North from the datum, metres
  double z {0.0};        // offset Up from entry_z, metres (negative = deeper)
  bool has_z {false};    // false -> hold entry_z
};
}  // namespace

class LocalGuidedNode : public rclcpp::Node
{
public:
  LocalGuidedNode()
  : Node("local_guided_node")
  {
    waypoint_file_ = this->declare_parameter<std::string>("waypoint_file", "");
    radius_m_ = this->declare_parameter<double>("radius_m", 0.3);
    dwell_s_ = this->declare_parameter<double>("dwell_s", 1.0);
    setpoint_rate_hz_ = this->declare_parameter<double>("setpoint_rate_hz", 10.0);
    pose_timeout_s_ = this->declare_parameter<double>("pose_timeout_s", 1.0);
    bus_timeout_s_ = this->declare_parameter<double>("bus_timeout_s", 0.5);
    yaw_hold_radius_m_ = this->declare_parameter<double>("yaw_hold_radius_m", 0.5);

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    pose_stamp_ = mode_stamp_ = t0;

    load_waypoints();

    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "cmd/local_guided/setpoint", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("nav/local/status", 10);

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();

    // SensorDataQoS (best-effort): MAVROS publishes its sensor topics
    // BEST_EFFORT, and a default RELIABLE subscription silently receives
    // NOTHING from it - rmw reports "incompatible QoS" and drops every sample.
    // The mission would then always abort with "no fresh pose", which is the
    // right safety behaviour for entirely the wrong reason.
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
        pose_ = *m;
        have_pose_ = true;
        pose_stamp_ = this->now();
      });
    mode_sub_ = this->create_subscription<Mode>(
      "jit/mode", latched,
      [this](const Mode::SharedPtr m) {granted_mode_ = m->mode; mode_stamp_ = this->now();});

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / setpoint_rate_hz_)),
      std::bind(&LocalGuidedNode::tick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "local_guided_node up. %zu waypoint(s) from '%s'. Acceptance: %.2f m then %.2f s "
      "dwell. Setpoints at %.0f Hz.",
      waypoints_.size(), waypoint_file_.c_str(), radius_m_, dwell_s_, setpoint_rate_hz_);
  }

private:
  enum class State { IDLE, RUNNING, COMPLETE, ABORTED };

  static const char * state_name(State s)
  {
    switch (s) {
      case State::IDLE: return "IDLE";
      case State::RUNNING: return "RUNNING";
      case State::COMPLETE: return "COMPLETE";
      case State::ABORTED: return "ABORTED";
    }
    return "?";
  }

  // ---- Waypoint file -------------------------------------------------------
  //
  // ROS 2 parameters cannot express a list of maps, so the mission lives in its
  // own plain YAML file loaded by path:
  //
  //   waypoints:
  //     - {x: 2.0, y: 0.0}            # holds entry depth
  //     - {x: 2.0, y: 2.0, z: -3.0}   # 3 m below the entry depth
  //
  void load_waypoints()
  {
    if (waypoint_file_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "No waypoint_file parameter set. No mission will run; the node will stay IDLE.");
      return;
    }
    try {
      const YAML::Node root = YAML::LoadFile(waypoint_file_);
      const YAML::Node list = root["waypoints"];
      if (!list || !list.IsSequence()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "'%s' has no 'waypoints:' sequence. No mission will run.", waypoint_file_.c_str());
        return;
      }
      for (const auto & item : list) {
        Waypoint wp;
        wp.x = item["x"] ? item["x"].as<double>() : 0.0;
        wp.y = item["y"] ? item["y"].as<double>() : 0.0;
        if (item["z"]) {
          wp.z = item["z"].as<double>();
          wp.has_z = true;
        }
        waypoints_.push_back(wp);
        const std::string z_desc = wp.has_z
          ? (std::to_string(wp.z) + " (override)")
          : std::string("entry depth");
        RCLCPP_INFO(
          this->get_logger(), "  wp[%zu]  x=%+.2f  y=%+.2f  z=%s",
          waypoints_.size() - 1, wp.x, wp.y, z_desc.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to read waypoint file '%s': %s. No mission will run.",
        waypoint_file_.c_str(), e.what());
      waypoints_.clear();
    }
  }

  bool fresh(const rclcpp::Time & stamp, double max_age_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= max_age_s;
  }

  // ---- Mode transitions ----------------------------------------------------
  void on_mode_entry()
  {
    if (waypoints_.empty()) {
      set_state(State::ABORTED, "no waypoints loaded");
      return;
    }
    if (!have_pose_ || !fresh(pose_stamp_, pose_timeout_s_)) {
      set_state(
        State::ABORTED,
        "no fresh /mavros/local_position/pose - refusing to start a mission against an "
        "unknown position");
      return;
    }

    datum_ = pose_.pose.position;
    entry_z_ = pose_.pose.position.z;
    index_ = 0;
    reached_flag_ = false;
    have_last_yaw_ = false;
    set_state(State::RUNNING, "datum captured");

    RCLCPP_INFO(
      this->get_logger(),
      "Mission start. Datum E=%.2f N=%.2f, entry depth z=%.2f. %zu waypoint(s), "
      "starting at index 0.",
      datum_.x, datum_.y, entry_z_, waypoints_.size());
  }

  void on_mode_exit()
  {
    if (state_ != State::IDLE) {
      set_state(State::IDLE, "left LOCAL_GUIDED");
    }
    index_ = 0;
    reached_flag_ = false;
  }

  void set_state(State s, const std::string & why)
  {
    if (s == state_) {
      return;
    }
    const State prev = state_;
    state_ = s;
    if (s == State::ABORTED) {
      RCLCPP_ERROR(
        this->get_logger(), "Mission %s -> ABORTED: %s", state_name(prev), why.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(), "Mission %s -> %s: %s", state_name(prev), state_name(s),
        why.c_str());
    }
  }

  // ---- Target construction -------------------------------------------------
  geometry_msgs::msg::PoseStamped current_target(const rclcpp::Time & now)
  {
    const Waypoint & wp = waypoints_[index_];

    geometry_msgs::msg::PoseStamped sp;
    sp.header.stamp = now;
    sp.header.frame_id = "map";
    sp.pose.position.x = datum_.x + wp.x;
    sp.pose.position.y = datum_.y + wp.y;
    sp.pose.position.z = wp.has_z ? (entry_z_ + wp.z) : entry_z_;

    // Point along track. Close in, the bearing is dominated by estimator noise,
    // so hold whatever heading we last commanded.
    const double dx = sp.pose.position.x - pose_.pose.position.x;
    const double dy = sp.pose.position.y - pose_.pose.position.y;
    const double planar = std::hypot(dx, dy);
    if (planar >= yaw_hold_radius_m_ || !have_last_yaw_) {
      last_yaw_ = std::atan2(dy, dx);   // ENU: CCW from East
      have_last_yaw_ = true;
    }
    sp.pose.orientation.x = 0.0;
    sp.pose.orientation.y = 0.0;
    sp.pose.orientation.z = std::sin(last_yaw_ * 0.5);
    sp.pose.orientation.w = std::cos(last_yaw_ * 0.5);
    return sp;
  }

  double horizontal_distance(const geometry_msgs::msg::PoseStamped & target) const
  {
    return std::hypot(
      target.pose.position.x - pose_.pose.position.x,
      target.pose.position.y - pose_.pose.position.y);
  }

  // Mirrors ArduPilot's verify_nav_wp(): latch on first touch, then run the
  // dwell clock without re-checking distance.
  void check_reached(const rclcpp::Time & now, const geometry_msgs::msg::PoseStamped & target)
  {
    const double d = horizontal_distance(target);
    last_distance_ = d;

    if (!reached_flag_) {
      if (d <= radius_m_) {
        reached_flag_ = true;
        reached_at_ = now;
        RCLCPP_INFO(
          this->get_logger(),
          "Waypoint %zu reached (%.2f m <= %.2f m). Dwelling %.2f s.",
          index_, d, radius_m_, dwell_s_);
      }
      return;
    }

    if ((now - reached_at_).seconds() >= dwell_s_) {
      advance();
    }
  }

  void advance()
  {
    reached_flag_ = false;
    if (index_ + 1 < waypoints_.size()) {
      ++index_;
      have_last_yaw_ = false;
      RCLCPP_INFO(this->get_logger(), "Advancing to waypoint %zu.", index_);
      return;
    }
    // Last waypoint. Hold the final target so the vehicle station-keeps there;
    // system_monitor_node starts the hold_timeout_s clock that leads to
    // mission-safe.
    set_state(State::COMPLETE, "final waypoint reached; holding station");
  }

  // ---- Main loop -----------------------------------------------------------
  void tick()
  {
    const rclcpp::Time now = this->now();
    const bool mode_active =
      (granted_mode_ == Mode::LOCAL_GUIDED) && fresh(mode_stamp_, bus_timeout_s_);

    if (mode_active && !was_active_) {
      on_mode_entry();
    } else if (!mode_active && was_active_) {
      on_mode_exit();
    }
    was_active_ = mode_active;

    if (!mode_active || (state_ != State::RUNNING && state_ != State::COMPLETE)) {
      publish_status();
      return;
    }

    // A mission cannot continue without a position estimate.
    if (!fresh(pose_stamp_, pose_timeout_s_)) {
      set_state(State::ABORTED, "/mavros/local_position/pose went stale mid-mission");
      publish_status();
      return;
    }

    const auto target = current_target(now);

    if (state_ == State::RUNNING) {
      check_reached(now, target);
    }

    // Republish continuously, but note this stream does NOT reach ArduSub at
    // this rate. vehicle_interface_node dedupes it and forwards a target only
    // when it moves, because each SET_POSITION_TARGET_LOCAL_NED restarts
    // AC_WPNav's s-curve and streaming them stops the vehicle accelerating.
    // What this rate is for is the cmd_timeout_s liveness gate over there.
    // Note this is NOT gated on vehicle/ready: vehicle_interface_node needs a
    // fresh setpoint to exist before it will arm, so waiting for ready here
    // would deadlock the two nodes against each other.
    setpoint_pub_->publish(target);
    publish_status();
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    status_pub_->publish(msg);
  }

  // --- Config ---
  std::string waypoint_file_;
  double radius_m_ {0.3};
  double dwell_s_ {1.0};
  double setpoint_rate_hz_ {10.0};
  double pose_timeout_s_ {1.0};
  double bus_timeout_s_ {0.5};
  double yaw_hold_radius_m_ {0.5};
  std::vector<Waypoint> waypoints_;

  // --- Inputs ---
  geometry_msgs::msg::PoseStamped pose_;
  bool have_pose_ {false};
  rclcpp::Time pose_stamp_;
  uint8_t granted_mode_ {Mode::SAFE};
  rclcpp::Time mode_stamp_;

  // --- Mission state ---
  State state_ {State::IDLE};
  bool was_active_ {false};
  geometry_msgs::msg::Point datum_;
  double entry_z_ {0.0};
  size_t index_ {0};
  bool reached_flag_ {false};
  rclcpp::Time reached_at_;
  double last_distance_ {0.0};
  double last_yaw_ {0.0};
  bool have_last_yaw_ {false};

  // --- ROS interfaces ---
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalGuidedNode>());
  rclcpp::shutdown();
  return 0;
}
