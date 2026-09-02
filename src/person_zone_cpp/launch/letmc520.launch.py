#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    device = LaunchConfiguration("device")
    start_debug = LaunchConfiguration("start_debug")

    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("astra_camera"),
                    "launch",
                    "letmc520.launch.xml",
                ]
            )
        )
    )
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("person_zone_cpp"),
                    "launch",
                    "pipeline.launch.py",
                ]
            )
        ),
        launch_arguments={
            "model_path": model_path,
            "device": device,
            "start_debug": start_debug,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path", default_value="yolov8n-pose.pt"
            ),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument("start_debug", default_value="true"),
            camera,
            pipeline,
        ]
    )
