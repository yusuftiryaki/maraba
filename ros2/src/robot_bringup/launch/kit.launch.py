"""The kit socket (docs/kit-soketi.md): kit_manager, kit_gnss, gps_selector,
ntrip_client, kit_camera, trunk_localizer.

Included by robot.launch.py with use_kit:=true. Nothing here is on the motor
command path: speed limits, the watchdog and the deadman do not change with
the kit (user decision, 2026-10-25).

    kit_fake_dir:=/tmp/fake_kit       fake_kit's files instead of the Pi's GPIO + 1-Wire
    kit_gnss_port:=/tmp/fake_kit_gnss a fake F9P (fake_kit gnss, or sim_node's)
    ntrip_file:=~/.robot/ntrip.yaml   the caster; missing = no RTK, everything else works

With use_kit, sensors.launch.py moves the NEO-6M to gps/neo6m/* and
gps_selector becomes the one publisher of gps/fix and gps/vel.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    fake_dir = LaunchConfiguration("kit_fake_dir")
    gnss_port = LaunchConfiguration("kit_gnss_port")
    ntrip_file = LaunchConfiguration("ntrip_file")
    camera_shim = LaunchConfiguration("kit_camera_shim")
    site_file = LaunchConfiguration("site_file")
    camera_time_log = LaunchConfiguration("kit_camera_time_log")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("kit_fake_dir", default_value=""),
        # udev symlink for the F9P's USB (docs/deployment.md).
        DeclareLaunchArgument("kit_gnss_port", default_value="/dev/kit_gnss"),
        DeclareLaunchArgument("ntrip_file",
                              default_value=os.path.expanduser("~/.robot/ntrip.yaml")),
        # The Camera SDK shim (robot_kit/camera.py). Empty: ~/.robot/sdk/kit_camera_shim
        # if it exists; the fake: "ros2 run robot_kit fake_camera_shim".
        DeclareLaunchArgument("kit_camera_shim", default_value=""),
        # (robot clock, camera media time) pairs while recording (camera.py)
        DeclareLaunchArgument("kit_camera_time_log",
                              default_value=os.path.expanduser("~/.robot/kit_camera_time.csv")),
        # The site's trees (poi "ağaç") for trunk_localizer; empty = no trees.
        DeclareLaunchArgument("site_file", default_value=""),

        Node(
            package="robot_kit", executable="kit_manager", name="kit_manager",
            output="screen",
            parameters=[{"fake_dir": fake_dir, "use_sim_time": use_sim_time}],
            respawn=True, respawn_delay=2.0,
        ),
        Node(
            package="robot_kit", executable="kit_gnss", name="kit_gnss", output="screen",
            parameters=[{"port": gnss_port, "use_sim_time": use_sim_time}],
            respawn=True, respawn_delay=2.0,
        ),
        Node(
            package="robot_kit", executable="gps_selector", name="gps_selector",
            output="screen", parameters=[{"use_sim_time": use_sim_time}],
            respawn=True, respawn_delay=1.0,
        ),
        Node(
            package="robot_kit", executable="kit_camera", name="kit_camera", output="screen",
            parameters=[{"shim_command": camera_shim, "time_log": camera_time_log,
                         "use_sim_time": use_sim_time}],
            respawn=True, respawn_delay=2.0,
        ),
        # The pose from the trunks the 360° camera sees (docs/kit-soketi.md) ->
        # ekf_global's pose0 (ekf.yaml). Idle without trees or detections.
        Node(
            package="robot_kit", executable="trunk_localizer", name="trunk_localizer",
            output="screen",
            parameters=[{"site_file": site_file, "use_sim_time": use_sim_time}],
            respawn=True, respawn_delay=2.0,
        ),
        Node(
            package="robot_kit", executable="ntrip_client", name="ntrip_client",
            output="screen",
            parameters=[{"credentials_file": ntrip_file, "use_sim_time": use_sim_time}],
            respawn=True, respawn_delay=5.0,
        ),
    ])
