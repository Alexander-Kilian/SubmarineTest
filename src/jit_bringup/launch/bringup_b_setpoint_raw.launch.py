#!/usr/bin/env python3
"""VARIANT B - the full stack with vehicle_interface_raw_node.

Guided targets go out on ``/mavros/setpoint_raw/local`` as a PositionTarget.
Same MAVLink message as variant A (SET_POSITION_TARGET_LOCAL_NED) and the same
ENU->NED conversion inside MAVROS; what changes is that the ``type_mask`` and
the yaw field are ours to set instead of being fixed by the setpoint_position
plugin.

    ros2 launch jit_bringup bringup_b_setpoint_raw.launch.py

To compare against variant A cleanly, run this with the SAME stream_mode. The
pairs that mean something:

    A dedupe  vs  A continuous   isolates the RATE question
    A dedupe  vs  B dedupe       isolates the TRANSPORT / type_mask question

Changing both at once would leave a good result unattributable to either.

The type_mask itself lives in jit_params.yaml under vehicle_interface_raw_node,
so it can be changed at the pool with a text editor and no rebuild:

    2552  position + yaw   - matches what variant A sends
    3576  position only    - adds IGNORE_YAW, ArduSub picks its own heading

That second value is worth a run. The observed failure was the sub yawing onto
the correct track bearing and then sitting there, which is consistent with the
s-curve reset described in vehicle_interface_node.cpp but also with a commanded
yaw fighting the position controller. Only trying it separates them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

VARIANT_EXE = "vehicle_interface_raw_node"


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
        description="Mission file. Plain YAML, not ROS params.",
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
    arg_stream = DeclareLaunchArgument(
        "stream_mode",
        default_value="dedupe",
        description="'dedupe' (forward on change) or 'continuous' (every tick).",
    )

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "jit_bringup.launch.py"])
        ),
        launch_arguments={
            "vehicle_interface_exe": VARIANT_EXE,
            "stream_mode": LaunchConfiguration("stream_mode"),
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
        arg_stream,
        LogInfo(
            msg=[
                "[bringup] VARIANT B - setpoint_raw/local (PositionTarget), stream_mode=",
                LaunchConfiguration("stream_mode"),
                ". type_mask comes from jit_params.yaml.",
            ]
        ),
        base,
    ])
