"""
ugvc_bt.launch.py
─────────────────
Launches the BT executor.
pass the GPS waypoints as arguments:

  ros2 launch ugvc_bt ugvc_bt.launch.py \
    wp1_lat:=30.0444 wp1_lon:=31.2357 \
    wp2_lat:=30.0450 wp2_lon:=31.2360 \
    wp3_lat:=30.0448 wp3_lon:=31.2350
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory("ugvc_bt")
    default_tree = os.path.join(pkg_dir, "trees", "ugvc_main_tree.xml")

    return LaunchDescription([

        # GPS waypoints — set these on competition day
        DeclareLaunchArgument("wp1_lat", default_value="30.0444"),
        DeclareLaunchArgument("wp1_lon", default_value="31.2357"),
        DeclareLaunchArgument("wp2_lat", default_value="30.0450"),
        DeclareLaunchArgument("wp2_lon", default_value="31.2360"),
        DeclareLaunchArgument("wp3_lat", default_value="30.0448"),
        DeclareLaunchArgument("wp3_lon", default_value="31.2350"),

        DeclareLaunchArgument("tree_xml_path",
            default_value=default_tree),

        Node(
            package="ugvc_bt",
            executable="bt_executor",
            name="bt_executor",
            output="screen",
            parameters=[{
                "tree_xml_path":  LaunchConfiguration("tree_xml_path"),
                "tick_rate_ms":   100,
                "wp1_lat": LaunchConfiguration("wp1_lat"),
                "wp1_lon": LaunchConfiguration("wp1_lon"),
                "wp2_lat": LaunchConfiguration("wp2_lat"),
                "wp2_lon": LaunchConfiguration("wp2_lon"),
                "wp3_lat": LaunchConfiguration("wp3_lat"),
                "wp3_lon": LaunchConfiguration("wp3_lon"),
            }]
        )
    ])