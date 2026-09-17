#!/usr/bin/env python3
"""Bring-up for the JIT submarine stack.

WHAT CHANGED FROM THE OLD CHAINED LAUNCH
----------------------------------------
The previous launch file started one node, probed its topic, and only then
started the next. That encoded the old series architecture into the bring-up.
With a health bus it is the wrong shape: ordering is ``system_monitor_node``'s
job at runtime, and it already does it. Nothing can arm until ``jit/health``
says so, and until each node has checked in the monitor simply reports which
one it is still waiting on.

So: everything starts at once, everything respawns, and the runtime gate does
the sequencing.

THE ONE HARD GATE
-----------------
``gpio_estop_node`` is started with ``respawn=False`` and a handler that shuts
the whole launch down if it exits non-zero. That node exits non-zero when it
cannot claim its GPIO lines, when it cannot read the relay feedback, or when
the boot self-test finds the relay welded shut. A vehicle whose e-stop does not
work should not finish booting, and respawning it in a loop would just hide the
fault behind a wall of restarts.

Set ``boot_selftest:=false`` for bench work where the relay or the feedback LDO
is not connected.

Usage:
    ros2 launch jit_bringup jit_bringup.launch.py
    ros2 launch jit_bringup jit_bringup.launch.py enable_control:=false
    ros2 launch jit_bringup jit_bringup.launch.py boot_selftest:=false
    ros2 launch jit_bringup jit_bringup.launch.py params_file:=/path/to/my.yaml
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

RESPAWN_DELAY_S = 2.0


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
        description="Mission file for local_guided_node. Plain YAML, not ROS params.",
    )
    arg_control = DeclareLaunchArgument(
        "enable_control",
        default_value="true",
        description="Start the locomotion and vehicle-I/O nodes. false = safety stack only.",
    )
    arg_selftest = DeclareLaunchArgument(
        "boot_selftest",
        default_value="true",
        description="Run the welded-relay check at startup. Only disable on the bench.",
    )
    arg_vehicle_exe = DeclareLaunchArgument(
        "vehicle_interface_exe",
        default_value="vehicle_interface_node",
        description=(
            "Which vehicle-interface variant to run. These are three transports for "
            "the same guided waypoints, being compared in the water:\n"
            "  vehicle_interface_node          A - setpoint_position/local\n"
            "  vehicle_interface_raw_node      B - setpoint_raw/local, explicit type_mask\n"
            "  vehicle_interface_mission_node  C - mission/push + AUTO\n"
            "Prefer the bringup_a/b/c wrappers, which also set the matching params."
        ),
    )
    arg_stream_mode = DeclareLaunchArgument(
        "stream_mode",
        default_value="dedupe",
        description=(
            "Guided setpoint pacing for variants A and B. 'dedupe' forwards a target "
            "only when it moves; 'continuous' forwards every tick at send_rate_hz, "
            "which is the PX4-derived 'at least 2 Hz, typically 20 Hz' advice. This is "
            "the core experiment, so it is a launch argument rather than only a "
            "params-file entry. A launch argument BEATS jit_params.yaml. Ignored by "
            "variant C, which uploads a mission instead of streaming."
        ),
    )

    params = LaunchConfiguration("params_file")
    waypoints = LaunchConfiguration("waypoint_file")
    vehicle_exe = LaunchConfiguration("vehicle_interface_exe")
    stream_mode = LaunchConfiguration("stream_mode")
    control_on = IfCondition(LaunchConfiguration("enable_control"))

    def node(pkg, exe, extra_params=None, name=None, **kw):
        param_list = [params]
        if extra_params:
            param_list.append(extra_params)
        return Node(
            package=pkg, executable=exe, name=name if name is not None else exe,
            parameters=param_list, output="screen", **kw
        )

    # --- Safety stack -------------------------------------------------------
    crsf_node = node(
        "jit_safety", "crsf_channel_node",
        respawn=True, respawn_delay=RESPAWN_DELAY_S,
    )

    # No respawn: see the module docstring. This node failing is a stop-work.
    gpio_node = node(
        "jit_safety", "gpio_estop_node",
        # ParameterValue with an explicit type: a bare LaunchConfiguration
        # resolves to the STRING "true"/"false", and the node declares this
        # parameter as a bool, so passing it raw is a type error at runtime.
        extra_params={
            "boot_selftest": ParameterValue(
                LaunchConfiguration("boot_selftest"), value_type=bool
            )
        },
        respawn=False,
    )

    monitor_node = node(
        "jit_safety", "system_monitor_node",
        respawn=True, respawn_delay=RESPAWN_DELAY_S,
    )

    # --- Indication ---------------------------------------------------------
    led_node = node("jit_ui", "led_driver_node", respawn=True, respawn_delay=RESPAWN_DELAY_S)

    # --- Control stack ------------------------------------------------------
    # The variant is chosen by argument. The node name follows the executable so
    # that the per-node blocks in jit_params.yaml still key correctly.
    #
    # waypoint_file is passed here as well as to local_guided_node: variant C
    # builds its mission from the same file. Variants A and B never declare that
    # parameter, and an override for a parameter a node does not declare is
    # simply unused - the same mechanism that lets one params file serve every
    # node in this launch.
    vehicle_node = node(
        "jit_control", vehicle_exe, name=vehicle_exe,
        extra_params={"waypoint_file": waypoints, "stream_mode": stream_mode},
        respawn=True, respawn_delay=RESPAWN_DELAY_S, condition=control_on,
    )
    manual_node = node(
        "jit_control", "manual_control_node",
        respawn=True, respawn_delay=RESPAWN_DELAY_S, condition=control_on,
    )
    local_node = node(
        "jit_control", "local_guided_node",
        extra_params={"waypoint_file": waypoints},
        respawn=True, respawn_delay=RESPAWN_DELAY_S, condition=control_on,
    )
    global_node = node(
        "jit_control", "global_guided_node",
        respawn=True, respawn_delay=RESPAWN_DELAY_S, condition=control_on,
    )

    def on_estop_exit(event, context):
        """Abort the launch only on a REAL failure.

        gpio_estop_node returns 1 when it throws: GPIO lines it cannot claim, a
        feedback pin it cannot read, or a welded relay found by the boot
        self-test. Any of those means the e-stop does not work and nothing else
        should be running.

        A clean exit (0) or death by signal (negative returncode) is just the
        launch shutting down - Ctrl-C sends SIGINT to every process, this one
        included. Treating that as a failure printed an alarming and completely
        false "the e-stop is not functional" on every normal Ctrl-C.
        """
        rc = event.returncode
        if rc is None or rc <= 0:
            return [LogInfo(msg="[bringup] gpio_estop_node exited cleanly (rc={}).".format(rc))]
        return [
            LogInfo(
                msg="[bringup] gpio_estop_node FAILED (rc={}) - the e-stop is not "
                    "functional. Shutting the launch down. Check the GPIO lines, the "
                    "relay feedback wiring, and whether the boot self-test found a "
                    "welded relay.".format(rc)
            ),
            EmitEvent(event=Shutdown(reason="gpio_estop_node failed")),
        ]

    abort_on_estop_failure = RegisterEventHandler(
        OnProcessExit(target_action=gpio_node, on_exit=on_estop_exit)
    )

    return LaunchDescription([
        arg_params,
        arg_waypoints,
        arg_control,
        arg_selftest,
        arg_vehicle_exe,
        arg_stream_mode,
        LogInfo(msg="[bringup] starting the JIT stack - health gate does the sequencing"),
        abort_on_estop_failure,
        crsf_node,
        gpio_node,
        monitor_node,
        led_node,
        vehicle_node,
        manual_node,
        local_node,
        global_node,
    ])
