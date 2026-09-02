#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    model_path = LaunchConfiguration("model_path")
    device = LaunchConfiguration("device")
    event_history_path = LaunchConfiguration("event_history_path")
    start_debug = LaunchConfiguration("start_debug")

    pose_config = PathJoinSubstitution(
        [FindPackageShare("person_pose"), "config", "pose.yaml"]
    )
    zone_config = PathJoinSubstitution(
        [FindPackageShare("person_zone_cpp"), "config", "person_zone.yaml"]
    )

    pose = Node(
        package="person_pose",
        executable="pose_publisher",
        output="screen",
        parameters=[
            pose_config,
            {
                "color_topic": color_topic,
                "model_path": model_path,
                "device": device,
            },
        ],
    )
    zone = Node(
        package="person_zone_cpp",
        executable="person_zone_node",
        output="screen",
        parameters=[
            zone_config,
            {
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "event_history_path": event_history_path,
            },
        ],
    )
    debug = Node(
        package="person_zone_debug",
        executable="debug_node",
        output="screen",
        condition=IfCondition(start_debug),
        parameters=[{"color_topic": color_topic}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "color_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/depth/camera_info",
            ),
            DeclareLaunchArgument(
                "model_path", default_value="yolov8n-pose.pt"
            ),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument("event_history_path", default_value=""),
            DeclareLaunchArgument("start_debug", default_value="true"),
            pose,
            zone,
            debug,
        ]
    )
