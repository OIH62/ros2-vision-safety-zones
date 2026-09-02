#!/usr/bin/env python3

"""Backward-compatible alias for the LeTMC-520 integrated launch file."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("person_zone_cpp"),
                            "launch",
                            "letmc520.launch.py",
                        ]
                    )
                )
            )
        ]
    )
