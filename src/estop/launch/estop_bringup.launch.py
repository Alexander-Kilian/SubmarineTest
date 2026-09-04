#!/usr/bin/env python3
"""Chained, health-gated bringup for the estop package.

Bring-up order:

    crsf_channel_node  -->  gpio_estop_node  -->  manual_control_node

Each stage starts its node (with ``respawn=True``) and then runs a probe that
waits for that node's primary topic to publish. The next stage starts only when
the probe succeeds.

* ``crsf_channel_node``  - probe ``/crsf/channel_threshold``. That topic is
  published every poll cycle regardless of RC link state, so receiving one
  message just means "the node is alive and its timer loop is running". The
  transmitter does not need to be on. Failure is retried forever (the node has
  ``respawn=True``, so a transient fault self-heals).

* ``gpio_estop_node``   - probe ``/estop/status``. The node does all of its
  GPIO setup (open chip, request line, drive LOW) in its constructor; any
  failure throws and the process exits non-zero, so the topic never appears.
  Failure is retried a bounded number of times, then the whole launch is shut
  down - a non-functional e-stop means nothing else should run.

* ``manual_control_node`` - started last, only after both probes have passed.
  Gated by the ``enable_manual_control`` launch argument.

This launch file is start-up insurance only. Runtime protection is the
watchdog inside gpio_estop_node and the gate inside manual_control_node; those
still do their job whatever order things came up in.

Usage:
    ros2 launch estop estop_bringup.launch.py
    ros2 launch estop estop_bringup.launch.py enable_manual_control:=false
    ros2 launch estop estop_bringup.launch.py params_file:=/path/to/my.yaml
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKG = "estop"

PROBE_TIMEOUT_S = 12      # length of one `ros2 topic echo --once` attempt
RETRY_DELAY_S = 5.0       # wait between a failed probe and the next attempt
NODE_SETTLE_S = 2.0       # let a freshly started node open its hardware first
GPIO_MAX_TRIES = 5        # gpio_estop_node: shut the launch down after this many


def _echo_cmd(topic, qos_args):
    """A bash one-liner that exits 0 iff a message is seen on `topic` within
    PROBE_TIMEOUT_S. `timeout` returns 124 when it kills a silent echo."""
    return [
        "bash", "-c",
        "timeout {t} ros2 topic echo --once {qos} {topic} >/dev/null 2>&1".format(
            t=PROBE_TIMEOUT_S, qos=qos_args, topic=topic
        ),
    ]


def make_probe_stage(topic, qos_args, on_success, on_exhausted, max_tries):
    """Return a zero-arg factory that yields [handler, probe] for one stage.

    On probe success -> `on_success` actions.
    On probe failure -> a fresh probe/handler pair after RETRY_DELAY_S, up to
    `max_tries` (0 = unlimited); once exhausted -> `on_exhausted` actions.
    """
    state = {"tries": 0}

    def build():
        state["tries"] += 1
        attempt = state["tries"]

        probe = ExecuteProcess(cmd=_echo_cmd(topic, qos_args), output="log")

        def on_exit(event, context):
            if event.returncode == 0:
                return [
                    LogInfo(msg="[bringup] {} healthy (attempt {})".format(topic, attempt)),
                    *on_success,
                ]
            if max_tries and attempt >= max_tries:
                return [
                    LogInfo(msg="[bringup] {} still unhealthy after {} attempts -> aborting".format(
                        topic, attempt)),
                    *on_exhausted,
                ]
            return [
                LogInfo(msg="[bringup] {} probe failed (rc={}, attempt {}); retry in {:.0f}s".format(
                    topic, event.returncode, attempt, RETRY_DELAY_S)),
                TimerAction(period=RETRY_DELAY_S, actions=build()),
            ]

        return [
            RegisterEventHandler(OnProcessExit(target_action=probe, on_exit=on_exit)),
            probe,
        ]

    return build


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare(PKG), "config", "estop_params.yaml"]
    )

    arg_params = DeclareLaunchArgument("params_file", default_value=default_params)
    arg_manual = DeclareLaunchArgument("enable_manual_control", default_value="true")
    params = LaunchConfiguration("params_file")

    def node(exe, **kw):
        return Node(
            package=PKG, executable=exe, name=exe,
            parameters=[params], output="screen",
            respawn=True, respawn_delay=NODE_SETTLE_S, **kw
        )

    crsf_node = node("crsf_channel_node")
    gpio_node = node("gpio_estop_node")
    manual_node = node(
        "manual_control_node",
        condition=IfCondition(LaunchConfiguration("enable_manual_control")),
    )

    # --- Stage 3: manual control (leaf) ---
    # manual_node itself is gated by enable_manual_control; the log line stays
    # honest about that.
    start_manual = [
        LogInfo(msg="[bringup] gpio_estop_node OK -> manual_control_node "
                    "(started unless enable_manual_control:=false)"),
        manual_node,
    ]

    # --- Stage 2: gpio_estop_node, then probe /estop/status ---
    gpio_probe = make_probe_stage(
        topic="/estop/status",
        # /estop/status is transient_local + reliable; match it so the latched
        # sample is delivered even between 2 Hz heartbeats.
        qos_args="--qos-durability transient_local --qos-reliability reliable",
        on_success=start_manual,
        on_exhausted=[
            EmitEvent(event=Shutdown(reason="gpio_estop_node failed its health check")),
        ],
        max_tries=GPIO_MAX_TRIES,
    )
    start_gpio = [
        LogInfo(msg="[bringup] crsf_channel_node OK -> starting gpio_estop_node"),
        gpio_node,
        TimerAction(period=NODE_SETTLE_S, actions=gpio_probe()),
    ]

    # --- Stage 1: crsf_channel_node, then probe /crsf/channel_threshold ---
    crsf_probe = make_probe_stage(
        topic="/crsf/channel_threshold",
        qos_args="",  # default volatile QoS - matches the publisher
        on_success=start_gpio,
        on_exhausted=[],
        max_tries=0,  # retry forever; crsf_channel_node respawns on its own
    )

    return LaunchDescription([
        arg_params,
        arg_manual,
        LogInfo(msg="[bringup] starting crsf_channel_node"),
        crsf_node,
        TimerAction(period=NODE_SETTLE_S, actions=crsf_probe()),
    ])
