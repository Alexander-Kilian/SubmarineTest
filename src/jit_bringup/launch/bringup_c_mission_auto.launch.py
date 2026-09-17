#!/usr/bin/env python3
"""VARIANT C - the full stack with vehicle_interface_mission_node.

The waypoint list is uploaded as an ArduPilot MISSION of MAV_CMD_NAV_WAYPOINT
items via ``/mavros/mission/push`` and flown in AUTO. This is the only
legitimate way to use NAV_WAYPOINT: it is a mission item, not a command, and
sending it through ``/mavros/cmd/command`` gets COMMAND_ACK ... UNSUPPORTED from
ArduPilot, whose command handler implements a specific subset of MAV_CMDs that
does not include it.

    ros2 launch jit_bringup bringup_c_mission_auto.launch.py

``stream_mode`` does not apply here - nothing is streamed.

WHAT TO WATCH, AND WHAT CAN GO WRONG
------------------------------------
local_guided_node still runs and still owns ``nav/local/status``. This variant
reads the same waypoints.yaml independently and ignores local_guided's
setpoints, subscribing to them only for the ``cmd_timeout_s`` liveness gate so
that a dead locomotion node still safes the vehicle.

The consequence, and it matters when reading the logs: MISSION_COMPLETE and
therefore mission-safe are still triggered by local_guided_node's own acceptance
logic against its own datum - NOT by ArduSub finishing the mission. The two can
disagree. Watch both:

    ros2 topic echo /nav/local/status      local_guided_node's verdict
    ros2 topic echo /nav/mission/status    ArduSub's, from /mavros/mission/reached

Failure modes to expect first, in order of likelihood:

  1. The mission push is REJECTED. ArduSub is refusing the mission protocol and
     this variant is a dead end on this firmware. The log says so explicitly.
  2. No EKF origin. There is no GPS, so the node publishes the configured
     origin (mission_origin_lat/lon in jit_params.yaml) to set_gp_origin and
     waits for gp_origin to echo it back. It never builds a mission from an
     unconfirmed origin. On Sub the origin can only be set ONCE per power
     cycle, so if this fails, power-cycle before retrying.
     The absolute origin is irrelevant: we convert ENU->lat/lon and ArduSub
     converts back with the same origin, so the round trip is the identity.
  3. Right horizontal track, wrong depth. ``mission_frame`` (default 3,
     MAV_FRAME_GLOBAL_RELATIVE_ALT) is the first thing to change - Sub mission
     altitude semantics are the least certain part of this variant.
  4. AUTO refuses to arm. Sub's AUTO is far less exercised than Copter's.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

VARIANT_EXE = "vehicle_interface_mission_node"


def generate_launch_description():
    pkg_share = FindPackageShare("jit_bringup")

    arg_params = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "jit_params.yaml"]),
        description="Node parameters, keyed by node name.",
    )
    arg_waypoints = DeclareLaunchArgument(
        "waypoint_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "waypoints.yaml"]),
        description="Mission file, read by BOTH local_guided_node and this variant.",
    )
    arg_control = DeclareLaunchArgument(
        "enable_control",
        default_value="true",
        description="Start the locomotion and vehicle-I/O nodes.",
    )
    arg_selftest = DeclareLaunchArgument(
        "boot_selftest",
        default_value="true",
        description="Run the welded-relay check at startup. Only disable on the bench.",
    )

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "jit_bringup.launch.py"])
        ),
        launch_arguments={
            "vehicle_interface_exe": VARIANT_EXE,
            "params_file": LaunchConfiguration("params_file"),
            "waypoint_file": LaunchConfiguration("waypoint_file"),
            "enable_control": LaunchConfiguration("enable_control"),
            "boot_selftest": LaunchConfiguration("boot_selftest"),
        }.items(),
    )

    return LaunchDescription([
        arg_params,
        arg_waypoints,
        arg_control,
        arg_selftest,
        LogInfo(
            msg="[bringup] VARIANT C - mission/push + AUTO (NAV_WAYPOINT). Watch "
                "/nav/local/status and /nav/mission/status; they can disagree."
        ),
        base,
    ])
