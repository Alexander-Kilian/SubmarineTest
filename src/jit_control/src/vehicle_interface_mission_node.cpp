// vehicle_interface_mission_node.cpp
//
// VARIANT C of the vehicle interface. Instead of streaming guided position
// targets, this node uploads the whole waypoint list as an ArduPilot MISSION of
// MAV_CMD_NAV_WAYPOINT items and flies it in AUTO.
//
// ---------------------------------------------------------------------------
// WHY THIS IS THE ONLY LEGITIMATE WAY TO USE NAV_WAYPOINT
// ---------------------------------------------------------------------------
// MAV_CMD_NAV_WAYPOINT is a MISSION ITEM, not a command. Sending it through
// /mavros/cmd/command (COMMAND_LONG) does not work: ArduPilot implements a
// specific subset of MAV_CMDs in its command handler and NAV_WAYPOINT is not
// among them, so it answers COMMAND_ACK ... UNSUPPORTED. The fact that
// mavros_msgs/CommandCode lists NAV_WAYPOINT means only that the constant
// exists in the MAVLink enum - that message is a transcription of MAV_CMD, not
// a statement of what any autopilot accepts.
//
// The real route is the mission protocol: upload with /mavros/mission/push,
// then switch to AUTO. That is what this node does.
//
// ---------------------------------------------------------------------------
// WHAT THIS COSTS, RELATIVE TO VARIANTS A AND B
// ---------------------------------------------------------------------------
//  1. NAV_WAYPOINT is inherently GLOBAL - its params 5/6/7 are lat/lon/alt.
//     The mission design is datum-relative ENU offsets specifically so that
//     estimator drift is absorbed, so every waypoint has to be converted.
//  2. That conversion needs an origin, and there is no GPS. See the origin
//     section below for why this works anyway.
//  3. Acceptance moves into the firmware (verify_nav_wp()) and out of
//     local_guided_node's check_reached().
//
// ---------------------------------------------------------------------------
// THE ORIGIN CANCELS OUT
// ---------------------------------------------------------------------------
// We convert datum-relative ENU -> lat/lon using an origin; ArduSub converts
// lat/lon -> its own local frame using the EKF origin. If both are the same
// origin the round trip is the identity and the absolute position on Earth is
// irrelevant. That is what makes a pool test possible with no GPS: any
// plausible origin works as long as it is the one ArduSub is using.
//
// So: read /mavros/global_position/gp_origin. If ArduSub has no origin yet and
// set_origin_if_absent is true, publish the configured one to
// /mavros/global_position/set_gp_origin and wait for the echo. No mission is
// ever built from an unconfirmed origin. Note that on Sub the origin can only
// be set ONCE per power cycle - a second attempt is silently ignored, which is
// why this node confirms by readback rather than by assuming its own write took.
//
// ---------------------------------------------------------------------------
// COEXISTENCE WITH local_guided_node  (deliberate, and it has a consequence)
// ---------------------------------------------------------------------------
// local_guided_node keeps running in this variant and keeps owning
// nav/local/status. This node reads the same waypoints.yaml independently and
// ignores local_guided's setpoints entirely - it subscribes to
// cmd/local_guided/setpoint for ONE reason, the cmd_timeout_s liveness gate, so
// that a dead locomotion node still safes the vehicle exactly as in A and B.
//
// The consequence to be aware of when reading the logs: MISSION_COMPLETE, and
// therefore mission-safe, is still triggered by local_guided_node's own
// acceptance logic against its own datum - NOT by ArduSub finishing the
// mission. The two verdicts can disagree. This node logs every
// /mavros/mission/reached and publishes nav/mission/status so the firmware's
// view can be compared against local_guided's side by side; that comparison is
// most of what this variant is for.
//
// ---------------------------------------------------------------------------
// UNCHANGED - do not let these drift from variants A and B
// ---------------------------------------------------------------------------
//   - single MAVROS writer, health gate, mode-before-arm ordering;
//   - DISARM IS ALWAYS RETRIED UNTIL /mavros/state.armed READS FALSE, and sent
//     at least min_disarm_commands times regardless. Standing project rule: the
//     service ACK is never proof;
//   - safing leaves the autonomous mode immediately and concurrently with the
//     disarm, never after it.
//
// ---------------------------------------------------------------------------
// Assumed external behaviour - NONE of this is verified
// ---------------------------------------------------------------------------
//   - ArduSub accepts a mission over the mission protocol and flies it in AUTO.
//     AUTO exists on Sub but is far less exercised than on Copter.
//   - Mission altitude on Sub is depth, and mission_frame (default
//     GLOBAL_RELATIVE_ALT = 3) is the right frame for it. If the vehicle flies
//     the horizontal track but holds the wrong depth, this is the first
//     parameter to change.
//   - Seq 0 is home by ArduPilot convention; real waypoints start at seq 1.
//   - /mavros/mission/push with start_index 0 REPLACES the stored mission.
//   - /mavros/mission/reached reports each seq as it is hit.
//   - A mission in AUTO starts on arming.

#include <algorithm>
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

#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"

#include "jit_msgs/msg/health.hpp"
#include "jit_msgs/msg/mode.hpp"

#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;
using Health = jit_msgs::msg::Health;
using Mode = jit_msgs::msg::Mode;
using Waypoint = mavros_msgs::msg::Waypoint;

namespace
{
// WGS84 equatorial radius. A spherical approximation is far more than good
// enough over the few metres this mission spans.
constexpr double kEarthRadiusM = 6378137.0;
constexpr uint16_t kNavWaypoint = 16;  // MAV_CMD_NAV_WAYPOINT

// One line of waypoints.yaml. Identical semantics to local_guided_node's
// struct so the same file drives both: offsets from the datum, in local ENU.
struct MissionPoint
{
  double x {0.0};        // metres East of the datum
  double y {0.0};        // metres North of the datum
  double z {0.0};        // metres Up from entry depth (negative = deeper)
  bool has_z {false};    // false -> hold the entry depth
};
}  // namespace

class VehicleInterfaceMissionNode : public rclcpp::Node
{
public:
  VehicleInterfaceMissionNode()
  : Node("vehicle_interface_mission_node")
  {
    manual_ardusub_mode_ =
      this->declare_parameter<std::string>("manual_ardusub_mode", "STABILIZE");
    // The one that differs: AUTO, not GUIDED.
    mission_ardusub_mode_ =
      this->declare_parameter<std::string>("mission_ardusub_mode", "AUTO");
    safing_mode_ = this->declare_parameter<std::string>("safing_mode", "STABILIZE");

    health_timeout_s_ = this->declare_parameter<double>("health_timeout_s", 0.5);
    mode_timeout_s_ = this->declare_parameter<double>("mode_timeout_s", 0.5);
    mavros_timeout_s_ = this->declare_parameter<double>("mavros_timeout_s", 3.0);
    cmd_timeout_s_ = this->declare_parameter<double>("cmd_timeout_s", 0.5);
    pose_timeout_s_ = this->declare_parameter<double>("pose_timeout_s", 1.0);
    arm_hold_s_ = this->declare_parameter<double>("arm_hold_s", 0.75);
    disarm_retry_hz_ = this->declare_parameter<double>("disarm_retry_hz", 5.0);
    mode_arm_retry_s_ = this->declare_parameter<double>("mode_arm_retry_s", 1.0);
    send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 20.0);

    min_disarm_commands_ = this->declare_parameter<int>("min_disarm_commands", 2);
    arm_stick_epsilon_ = this->declare_parameter<double>("arm_stick_epsilon", 60.0);
    z_neutral_ = this->declare_parameter<double>("z_neutral", 500.0);

    // --- Mission build ---
    waypoint_file_ = this->declare_parameter<std::string>("waypoint_file", "");
    // Firmware-side acceptance, replacing local_guided_node's radius/dwell.
    // Defaults match jit_params so the two variants are comparable.
    mission_radius_m_ = this->declare_parameter<double>("radius_m", 0.3);
    mission_dwell_s_ = this->declare_parameter<double>("dwell_s", 1.0);
    // MAV_FRAME_GLOBAL_RELATIVE_ALT = 3. See the assumptions block.
    mission_frame_ = static_cast<uint8_t>(this->declare_parameter<int>("mission_frame", 3));
    push_retry_s_ = this->declare_parameter<double>("mission_push_retry_s", 2.0);

    // --- Origin ---
    origin_lat_ = this->declare_parameter<double>("mission_origin_lat", 47.397742);
    origin_lon_ = this->declare_parameter<double>("mission_origin_lon", 8.545594);
    origin_alt_ = this->declare_parameter<double>("mission_origin_alt", 0.0);
    set_origin_if_absent_ =
      this->declare_parameter<bool>("set_origin_if_absent", true);
    origin_wait_s_ = this->declare_parameter<double>("origin_wait_s", 5.0);
    origin_retry_s_ = this->declare_parameter<double>("origin_retry_s", 2.0);

    const auto t0 = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    health_stamp_ = mode_stamp_ = mavros_stamp_ = t0;
    manual_stamp_ = local_stamp_ = pose_stamp_ = t0;
    last_arm_call_ = last_mode_call_ = last_push_call_ = last_origin_write_ = t0;
    node_started_ = this->now();

    load_waypoints();

    manual_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
      "/mavros/manual_control/send", 10);
    set_origin_pub_ = this->create_publisher<geographic_msgs::msg::GeoPointStamped>(
      "/mavros/global_position/set_gp_origin", rclcpp::QoS(1).transient_local());

    rclcpp::QoS latched(1);
    latched.reliable();
    latched.transient_local();
    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("vehicle/ready", latched);
    // Deliberately NOT nav/local/status - local_guided_node owns that topic and
    // two publishers on it would make the monitor's verdict nondeterministic.
    mission_status_pub_ =
      this->create_publisher<std_msgs::msg::String>("nav/mission/status", 10);

    health_sub_ = this->create_subscription<Health>(
      "jit/health", latched,
      [this](const Health::SharedPtr m) {health_ = *m; have_health_ = true;
        health_stamp_ = this->now();});
    mode_sub_ = this->create_subscription<Mode>(
      "jit/mode", latched,
      [this](const Mode::SharedPtr m) {granted_mode_ = m->mode; have_mode_ = true;
        mode_stamp_ = this->now();});

    manual_sub_ = this->create_subscription<mavros_msgs::msg::ManualControl>(
      "cmd/manual/manual_control", 10,
      [this](const mavros_msgs::msg::ManualControl::SharedPtr m) {
        if (granted_mode_ != Mode::MANUAL) {return;}
        manual_cmd_ = *m;
        manual_stamp_ = this->now();
      });

    // Liveness only. The payload is never forwarded in this variant - ArduSub
    // is flying an uploaded mission, not our setpoints - but a dead
    // local_guided_node must still safe the vehicle exactly as in A and B.
    local_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "cmd/local_guided/setpoint", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr) {
        local_stamp_ = this->now();
      });

    // MAVROS publishes its sensor topics BEST_EFFORT; a default RELIABLE
    // subscription silently receives nothing from it.
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
        pose_ = *m;
        have_pose_ = true;
        pose_stamp_ = this->now();
      });

    origin_sub_ = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
      "/mavros/global_position/gp_origin", rclcpp::QoS(1).transient_local(),
      [this](const geographic_msgs::msg::GeoPointStamped::SharedPtr m) {
        if (!have_origin_) {
          RCLCPP_INFO(
            this->get_logger(),
            "EKF origin confirmed by ArduSub: lat %.7f lon %.7f alt %.2f.",
            m->position.latitude, m->position.longitude, m->position.altitude);
        }
        origin_ = m->position;
        have_origin_ = true;
      });

    reached_sub_ = this->create_subscription<mavros_msgs::msg::WaypointReached>(
      "/mavros/mission/reached", 10,
      [this](const mavros_msgs::msg::WaypointReached::SharedPtr m) {
        last_reached_seq_ = m->wp_seq;
        have_reached_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "ArduSub reports mission seq %u reached (last real waypoint is seq %zu). "
          "Compare this against nav/local/status from local_guided_node.",
          static_cast<unsigned>(m->wp_seq), waypoints_.size());
        if (m->wp_seq >= waypoints_.size()) {
          mission_complete_ = true;
          RCLCPP_INFO(this->get_logger(), "ArduSub has finished the mission.");
        }
      });

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&VehicleInterfaceMissionNode::state_cb, this, std::placeholders::_1));

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    push_client_ = this->create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");

    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / send_rate_hz_)),
      std::bind(&VehicleInterfaceMissionNode::control_tick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "vehicle_interface_mission_node (VARIANT C) up. MANUAL -> '%s', mission -> '%s', "
      "safing -> '%s'. %zu waypoint(s) loaded, frame %u, accept %.2f m / %.1f s. "
      "Origin: %s (configured %.7f, %.7f). Disarm retries at %.1f Hz until "
      "/mavros/state confirms, minimum %d commands.",
      manual_ardusub_mode_.c_str(), mission_ardusub_mode_.c_str(), safing_mode_.c_str(),
      waypoints_.size(), static_cast<unsigned>(mission_frame_), mission_radius_m_,
      mission_dwell_s_, set_origin_if_absent_ ? "set if absent" : "read only",
      origin_lat_, origin_lon_, disarm_retry_hz_, min_disarm_commands_);
  }

private:
  // ---- Mission file --------------------------------------------------------
  // Same file, same format, same semantics as local_guided_node. Kept as a
  // separate parser rather than shared code so that this experimental variant
  // cannot break the node that works.
  void load_waypoints()
  {
    if (waypoint_file_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "No waypoint_file parameter. This node cannot build a mission and will never "
        "leave MANUAL.");
      return;
    }
    try {
      const YAML::Node root = YAML::LoadFile(waypoint_file_);
      const YAML::Node list = root["waypoints"];
      if (!list || !list.IsSequence()) {
        RCLCPP_ERROR(
          this->get_logger(), "%s has no 'waypoints' sequence.", waypoint_file_.c_str());
        return;
      }
      for (const auto & item : list) {
        MissionPoint w;
        w.x = item["x"] ? item["x"].as<double>() : 0.0;
        w.y = item["y"] ? item["y"].as<double>() : 0.0;
        if (item["z"]) {
          w.z = item["z"].as<double>();
          w.has_z = true;
        }
        waypoints_.push_back(w);
      }
      RCLCPP_INFO(
        this->get_logger(), "Loaded %zu waypoint(s) from %s.",
        waypoints_.size(), waypoint_file_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to parse %s: %s", waypoint_file_.c_str(), e.what());
      waypoints_.clear();
    }
  }

  // ---- Origin --------------------------------------------------------------
  // Returns true once ArduSub has an origin we have SEEN. Never returns true on
  // the strength of our own write: on Sub the origin can only be set once per
  // power cycle, so a write that appears to succeed may have been ignored.
  bool ensure_origin(const rclcpp::Time & now)
  {
    if (have_origin_) {
      return true;
    }
    const double waited = (now - node_started_).seconds();
    if (waited < origin_wait_s_) {
      return false;  // still giving ArduSub a chance to publish one
    }
    if (!set_origin_if_absent_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "No EKF origin on /mavros/global_position/gp_origin and set_origin_if_absent is "
        "false. No mission can be built.");
      return false;
    }
    if ((now - last_origin_write_).seconds() < origin_retry_s_) {
      return false;
    }
    last_origin_write_ = now;
    geographic_msgs::msg::GeoPointStamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = "map";
    msg.position.latitude = origin_lat_;
    msg.position.longitude = origin_lon_;
    msg.position.altitude = origin_alt_;
    set_origin_pub_->publish(msg);
    RCLCPP_WARN(
      this->get_logger(),
      "No EKF origin yet - publishing the configured one (%.7f, %.7f) to "
      "set_gp_origin. Waiting for gp_origin to echo it back; a write is not proof.",
      origin_lat_, origin_lon_);
    return false;
  }

  // ---- Mission build -------------------------------------------------------
  // ENU offset from the datum -> lat/lon, using the confirmed origin. ArduSub
  // converts back with the same origin, so the round trip is the identity and
  // the absolute location is irrelevant.
  void enu_to_latlon(double east_m, double north_m, double & lat, double & lon) const
  {
    const double lat0_rad = origin_.latitude * M_PI / 180.0;
    lat = origin_.latitude + (north_m / kEarthRadiusM) * (180.0 / M_PI);
    const double cos_lat = std::max(1e-6, std::cos(lat0_rad));
    lon = origin_.longitude + (east_m / (kEarthRadiusM * cos_lat)) * (180.0 / M_PI);
  }

  std::vector<Waypoint> build_mission() const
  {
    std::vector<Waypoint> items;
    items.reserve(waypoints_.size() + 1);

    // Datum is the pose at mode entry: the same trick local_guided_node uses to
    // absorb estimator drift accumulated since boot. Offsets are relative to it.
    const double dx = datum_.position.x;
    const double dy = datum_.position.y;
    const double dz = datum_.position.z;

    // Seq 0 is home by ArduPilot convention. It is not flown.
    Waypoint home;
    home.frame = mission_frame_;
    home.command = kNavWaypoint;
    home.is_current = false;
    home.autocontinue = true;
    enu_to_latlon(dx, dy, home.x_lat, home.y_long);
    home.z_alt = dz;
    items.push_back(home);

    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto & w = waypoints_[i];
      Waypoint item;
      item.frame = mission_frame_;
      item.command = kNavWaypoint;
      item.is_current = (i == 0);   // first real waypoint is the current one
      item.autocontinue = true;
      item.param1 = static_cast<float>(mission_dwell_s_);     // hold time, s
      item.param2 = static_cast<float>(mission_radius_m_);    // acceptance radius, m
      item.param3 = 0.0f;                                     // pass-through radius
      // NaN = "leave the yaw alone". A literal 0 here would command a heading of
      // due north at every waypoint, which is not what the ENU offsets mean.
      item.param4 = std::nanf("");
      enu_to_latlon(dx + w.x, dy + w.y, item.x_lat, item.y_long);
      item.z_alt = w.has_z ? (dz + w.z) : dz;
      items.push_back(item);
    }
    return items;
  }

  void push_mission(const rclcpp::Time & now)
  {
    if (!push_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "/mavros/mission/push not available yet.");
      return;
    }
    if (push_pending_ || (now - last_push_call_).seconds() < push_retry_s_) {
      return;
    }
    const auto items = build_mission();
    auto req = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    req->start_index = 0;   // full replace
    req->waypoints = items;

    push_pending_ = true;
    last_push_call_ = now;
    RCLCPP_INFO(
      this->get_logger(),
      "Pushing %zu mission item(s) (1 home + %zu waypoints). Datum E=%.2f N=%.2f z=%.2f; "
      "first target lat %.7f lon %.7f alt %.2f.",
      items.size(), waypoints_.size(), datum_.position.x, datum_.position.y,
      datum_.position.z,
      items.size() > 1 ? items[1].x_lat : 0.0,
      items.size() > 1 ? items[1].y_long : 0.0,
      items.size() > 1 ? items[1].z_alt : 0.0);

    push_client_->async_send_request(
      req,
      [this](rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture future) {
        push_pending_ = false;
        const auto resp = future.get();
        if (resp->success) {
          mission_pushed_ = true;
          RCLCPP_INFO(
            this->get_logger(), "Mission accepted: %u item(s) transferred.",
            resp->wp_transfered);
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "Mission push REJECTED (%u transferred); will retry. If this never "
            "succeeds, ArduSub is refusing the mission protocol and variant C is a "
            "dead end on this firmware.",
            resp->wp_transfered);
        }
      });
  }

  void invalidate_mission()
  {
    mission_pushed_ = false;
    mission_complete_ = false;
    have_reached_ = false;
    have_datum_ = false;
  }

  // ---- Inputs --------------------------------------------------------------
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    if (have_state_ && msg->armed != mavros_armed_) {
      RCLCPP_INFO(this->get_logger(), "ArduSub is now %s.", msg->armed ? "ARMED" : "DISARMED");
    }
    if (!msg->mode.empty() && msg->mode != mavros_mode_) {
      RCLCPP_INFO(this->get_logger(), "Flight mode is now '%s'.", msg->mode.c_str());
    }
    if (have_state_ && msg->connected != mavros_connected_) {
      RCLCPP_WARN(
        this->get_logger(), "MAVROS <-> ArduSub link %s.",
        msg->connected ? "CONNECTED" : "LOST");
    }
    have_state_ = true;
    mavros_connected_ = msg->connected;
    mavros_armed_ = msg->armed;
    mavros_mode_ = msg->mode;
    mavros_stamp_ = this->now();
  }

  bool fresh(const rclcpp::Time & stamp, double max_age_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= max_age_s;
  }

  // ---- MAVROS calls --------------------------------------------------------
  void send_set_mode(const std::string & mode)
  {
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "/mavros/set_mode not available yet.");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode = 0;
    req->custom_mode = mode;
    mode_pending_ = true;
    last_mode_call_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Requesting flight mode '%s'.", mode.c_str());
    set_mode_client_->async_send_request(
      req,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        mode_pending_ = false;
        const auto resp = future.get();
        if (!resp->mode_sent) {
          RCLCPP_WARN(this->get_logger(), "Mode '%s' rejected; will retry.", mode.c_str());
        }
      });
  }

  void send_arming(bool arm, const std::string & reason)
  {
    if (!arming_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "/mavros/cmd/arming not available yet.");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    arm_pending_ = true;
    last_arm_call_ = this->now();
    if (!arm) {
      ++disarm_sent_;
      RCLCPP_WARN(this->get_logger(), "DISARM command #%d: %s", disarm_sent_, reason.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "ARM command: %s", reason.c_str());
    }
    arming_client_->async_send_request(
      req,
      [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        arm_pending_ = false;
        const auto resp = future.get();
        if (!resp->success) {
          RCLCPP_WARN(
            this->get_logger(), "%s REJECTED (result=%u); will retry.",
            arm ? "ARM" : "DISARM", resp->result);
        }
        // A success ACK is NOT confirmation. Only /mavros/state.armed is.
      });
  }

  bool arm_call_due(const rclcpp::Time & now, double period_s) const
  {
    return !arm_pending_ && (now - last_arm_call_).seconds() >= period_s;
  }
  bool mode_call_due(const rclcpp::Time & now, double period_s) const
  {
    return !mode_pending_ && (now - last_mode_call_).seconds() >= period_s;
  }

  // ---- Main loop -----------------------------------------------------------
  void control_tick()
  {
    const rclcpp::Time now = this->now();

    const bool health_ok = have_health_ && health_.ok && fresh(health_stamp_, health_timeout_s_);
    const bool mode_ok = have_mode_ && fresh(mode_stamp_, mode_timeout_s_);
    const bool mavros_ok = have_state_ && mavros_connected_ &&
      fresh(mavros_stamp_, mavros_timeout_s_);
    const bool gate = health_ok && mode_ok && mavros_ok && granted_mode_ != Mode::SAFE;

    if (!gate) {
      std::string why;
      if (!health_ok) {
        why = have_health_ ? ("health: " + health_.detail) : "health: never received";
      } else if (!mode_ok) {
        why = "jit/mode stale";
      } else if (!mavros_ok) {
        why = "MAVROS link";
      } else {
        why = "granted mode is SAFE";
      }
      run_safing(now, why);
      publish_mission_status();
      return;
    }

    if (!gate_true_since_) {
      gate_true_since_ = now;
    }
    if (safing_active_) {
      RCLCPP_INFO(this->get_logger(), "Gate restored - leaving safing.");
      safing_active_ = false;
      disarm_sent_ = 0;
    }

    const bool mission_mode = (granted_mode_ == Mode::LOCAL_GUIDED);

    // In a mission variant the upload has to happen before the mode change, so
    // this block sits ahead of the mode/arm ladder that A and B share.
    if (mission_mode && !mission_pushed_) {
      publish_ready(false);
      if (!prepare_mission(now)) {
        publish_mission_status();
        return;
      }
      publish_mission_status();
      return;
    }

    const std::string target_mode =
      mission_mode ? mission_ardusub_mode_ : ardusub_mode_for(granted_mode_);

    if (mavros_mode_ != target_mode) {
      if (mode_call_due(now, mode_arm_retry_s_)) {
        send_set_mode(target_mode);
      }
      publish_ready(false);
      publish_mission_status();
      return;
    }

    if (!mavros_armed_) {
      std::string blocker;
      if ((now - *gate_true_since_).seconds() < arm_hold_s_) {
        blocker = "gate not held long enough";
      } else if (!active_cmd_fresh(now)) {
        blocker = "no fresh command from the active source";
      } else if (granted_mode_ == Mode::MANUAL && !sticks_neutral()) {
        blocker = "sticks not neutral";
      } else if (mission_mode && !mission_pushed_) {
        blocker = "mission not uploaded";
      }
      if (blocker.empty()) {
        if (arm_call_due(now, mode_arm_retry_s_)) {
          send_arming(true, "gate satisfied, in mode '" + target_mode + "'");
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "Not arming: %s.", blocker.c_str());
      }
      publish_ready(false);
      publish_mission_status();
      return;
    }

    // A dead locomotion node must still safe the vehicle, exactly as in A and B.
    if (!active_cmd_fresh(now)) {
      run_safing(now, "active command source went stale");
      publish_mission_status();
      return;
    }

    // Armed and in mode. In MANUAL we still forward sticks; in the mission mode
    // there is nothing to forward - ArduSub is flying its own uploaded mission.
    if (granted_mode_ == Mode::MANUAL) {
      auto mc = manual_cmd_;
      mc.header.stamp = now;
      manual_pub_->publish(mc);
    }
    publish_ready(true);
    publish_mission_status();
  }

  // Returns true once a mission is uploaded and accepted.
  bool prepare_mission(const rclcpp::Time & now)
  {
    if (waypoints_.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "No waypoints loaded; nothing to upload.");
      return false;
    }
    if (!ensure_origin(now)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Waiting for a confirmed EKF origin before building the mission.");
      return false;
    }
    // A mission must never be built against an unknown position: the whole
    // point of the datum is that it is a real, current pose.
    if (!fresh(pose_stamp_, pose_timeout_s_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "No fresh /mavros/local_position/pose; refusing to capture a datum.");
      return false;
    }
    if (!have_datum_) {
      datum_ = pose_.pose;
      have_datum_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Datum captured at E=%.2f N=%.2f z=%.2f. local_guided_node captures its own "
        "a tick or so apart; compare the two if the tracks disagree.",
        datum_.position.x, datum_.position.y, datum_.position.z);
    }
    push_mission(now);
    return mission_pushed_;
  }

  std::string ardusub_mode_for(uint8_t granted) const
  {
    switch (granted) {
      case Mode::MANUAL: return manual_ardusub_mode_;
      case Mode::LOCAL_GUIDED:
      case Mode::GLOBAL_GUIDED: return mission_ardusub_mode_;
      default: return safing_mode_;
    }
  }

  bool active_cmd_fresh(const rclcpp::Time & now) const
  {
    (void)now;
    switch (granted_mode_) {
      case Mode::MANUAL: return fresh(manual_stamp_, cmd_timeout_s_);
      case Mode::LOCAL_GUIDED: return fresh(local_stamp_, cmd_timeout_s_);
      default: return false;
    }
  }

  bool sticks_neutral() const
  {
    return std::fabs(manual_cmd_.x) < arm_stick_epsilon_ &&
           std::fabs(manual_cmd_.r) < arm_stick_epsilon_;
  }

  void publish_mission_status()
  {
    std_msgs::msg::String msg;
    if (!mission_pushed_) {
      msg.data = have_datum_ ? "UPLOADING" : "IDLE";
    } else if (mission_complete_) {
      msg.data = "COMPLETE";
    } else if (mavros_armed_ && mavros_mode_ == mission_ardusub_mode_) {
      msg.data = have_reached_
        ? ("RUNNING seq " + std::to_string(last_reached_seq_) + " reached")
        : "RUNNING";
    } else {
      msg.data = "UPLOADED";
    }
    mission_status_pub_->publish(msg);
  }

  // ---- Safing --------------------------------------------------------------
  void run_safing(const rclcpp::Time & now, const std::string & reason)
  {
    gate_true_since_.reset();
    publish_ready(false);
    // Leaving AUTO abandons the mission; a re-entry must re-capture the datum
    // and re-upload rather than assume the stored mission is still meaningful.
    invalidate_mission();

    if (!safing_active_) {
      safing_active_ = true;
      disarm_sent_ = 0;
      RCLCPP_WARN(this->get_logger(), "SAFING - %s", reason.c_str());
    }

    if (!have_state_) {
      return;
    }

    // 1. Out of AUTO immediately, for the same reason variants A and B leave
    //    GUIDED immediately: the vehicle is tracking a destination that our
    //    silence does not clear.
    if (mavros_mode_ == mission_ardusub_mode_ && mode_call_due(now, mode_arm_retry_s_)) {
      send_set_mode(safing_mode_);
    }

    // 2. Disarm, retried until /mavros/state confirms, minimum
    //    min_disarm_commands. Standing project rule - never one command.
    const bool need_more = mavros_armed_ || (disarm_sent_ < min_disarm_commands_);
    if (need_more && arm_call_due(now, 1.0 / std::max(0.1, disarm_retry_hz_))) {
      send_arming(false, reason);
    }
    if (!mavros_armed_ && disarm_sent_ >= min_disarm_commands_ && !disarm_confirmed_logged_) {
      disarm_confirmed_logged_ = true;
      RCLCPP_INFO(
        this->get_logger(), "Disarm CONFIRMED by /mavros/state after %d commands.",
        disarm_sent_);
    }
    if (mavros_armed_) {
      disarm_confirmed_logged_ = false;
    }

    // 3. Still armed but out of AUTO: command zero motion.
    if (mavros_armed_ && mavros_mode_ != mission_ardusub_mode_) {
      mavros_msgs::msg::ManualControl neutral;
      neutral.header.stamp = now;
      neutral.x = 0.0f;
      neutral.y = 0.0f;
      neutral.z = static_cast<float>(z_neutral_);
      neutral.r = 0.0f;
      neutral.buttons = 0;
      manual_pub_->publish(neutral);
    }
  }

  void publish_ready(bool ready)
  {
    if (have_ready_ && ready == last_ready_) {
      return;
    }
    have_ready_ = true;
    last_ready_ = ready;
    std_msgs::msg::Bool msg;
    msg.data = ready;
    ready_pub_->publish(msg);
  }

  // --- Config ---
  std::string manual_ardusub_mode_;
  std::string mission_ardusub_mode_;
  std::string safing_mode_;
  std::string waypoint_file_;
  double health_timeout_s_ {0.5};
  double mode_timeout_s_ {0.5};
  double mavros_timeout_s_ {3.0};
  double cmd_timeout_s_ {0.5};
  double pose_timeout_s_ {1.0};
  double arm_hold_s_ {0.75};
  double disarm_retry_hz_ {5.0};
  double mode_arm_retry_s_ {1.0};
  double send_rate_hz_ {20.0};
  int min_disarm_commands_ {2};
  double arm_stick_epsilon_ {60.0};
  double z_neutral_ {500.0};
  double mission_radius_m_ {0.3};
  double mission_dwell_s_ {1.0};
  uint8_t mission_frame_ {3};
  double push_retry_s_ {2.0};
  double origin_lat_ {0.0};
  double origin_lon_ {0.0};
  double origin_alt_ {0.0};
  bool set_origin_if_absent_ {true};
  double origin_wait_s_ {5.0};
  double origin_retry_s_ {2.0};

  // --- Mission ---
  std::vector<MissionPoint> waypoints_;
  geometry_msgs::msg::Pose datum_;
  bool have_datum_ {false};
  bool mission_pushed_ {false};
  bool push_pending_ {false};
  bool mission_complete_ {false};
  bool have_reached_ {false};
  uint16_t last_reached_seq_ {0};
  rclcpp::Time last_push_call_;

  // --- Origin ---
  geographic_msgs::msg::GeoPoint origin_;
  bool have_origin_ {false};
  rclcpp::Time last_origin_write_;
  rclcpp::Time node_started_;

  // --- Bus ---
  bool have_health_ {false};
  Health health_;
  rclcpp::Time health_stamp_;
  bool have_mode_ {false};
  uint8_t granted_mode_ {Mode::SAFE};
  rclcpp::Time mode_stamp_;

  // --- Command sources ---
  mavros_msgs::msg::ManualControl manual_cmd_;
  rclcpp::Time manual_stamp_;
  rclcpp::Time local_stamp_;

  // --- Vehicle state ---
  geometry_msgs::msg::PoseStamped pose_;
  bool have_pose_ {false};
  rclcpp::Time pose_stamp_;
  bool have_state_ {false};
  bool mavros_connected_ {false};
  bool mavros_armed_ {false};
  std::string mavros_mode_;
  rclcpp::Time mavros_stamp_;

  // --- Control state ---
  std::optional<rclcpp::Time> gate_true_since_;
  bool safing_active_ {true};
  int disarm_sent_ {0};
  bool disarm_confirmed_logged_ {false};
  bool arm_pending_ {false};
  bool mode_pending_ {false};
  rclcpp::Time last_arm_call_;
  rclcpp::Time last_mode_call_;
  bool have_ready_ {false};
  bool last_ready_ {false};

  // --- ROS interfaces ---
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr set_origin_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_status_pub_;
  rclcpp::Subscription<Health>::SharedPtr health_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr reached_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr push_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInterfaceMissionNode>());
  rclcpp::shutdown();
  return 0;
}
