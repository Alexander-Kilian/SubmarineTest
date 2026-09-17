#!/usr/bin/env python3
"""VARIANT A - the full stack with vehicle_interface_node.

Guided targets go out on ``/mavros/setpoint_position/local`` as a PoseStamped.
This is the current design and the control arm of the experiment: it is the code
that produced the one-shot-setpoint analysis, and it has NOT yet been tested in
the water with the dedupe active.

Two runs to get from this variant:

    ros2 launch jit_bringup bringup_a_setpoint_local.launch.py
    ros2 launch jit_bringup bringup_a_setpoint_local.launch.py stream_mode:=continuous

The first is the fix under test. The second reproduces the original failure on
purpose - it streams at ``send_rate_hz``, which is what the widely quoted
"minimum 2 Hz, typical 20 Hz" guidance prescribes. That guidance is PX4 OFFBOARD
advice: PX4 drops out of offboard without a >2 Hz stream. ArduPilot GUIDED has
no such requirement for POSITION targets - ``GUID_TIMEOUT`` covers attitude,
velocity and acceleration only. Running both arms settles that from the vehicle
rather than from documentation.

Everything else - the safety stack, the health bus, the relay chain,
local_guided_node - is identical across all three variants.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

VARIANT_EXE = "vehicle_interface_node"


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
                "[bringup] VARIANT A - setpoint_position/local (PoseStamped), stream_mode=",
                LaunchConfiguration("stream_mode"),
            ]
        ),
        base,
    ])
