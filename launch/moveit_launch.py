#!/usr/bin/env python

"""Launch Webots Gen3 Robot simulation with MoveIt2."""

import os
import pathlib
import yaml
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)


PACKAGE_NAME = "webots_gen3"


def generate_launch_description():
    launch_description_nodes = []
    package_dir = get_package_share_directory(PACKAGE_NAME)

    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, "resource", filename)).read_text()

    def load_yaml(filename):
        return yaml.safe_load(load_file(filename))

    # Check if moveit is installed
    if "moveit" in get_packages_with_prefixes():
        # Configuration
        description = {"robot_description": load_file("gen3.urdf")}
        description_semantic = {
            "robot_description_semantic": load_file("moveit_gen3.srdf")
        }
        description_kinematics = {
            "robot_description_kinematics": load_yaml("moveit_kinematics.yaml")
        }
        description_joint_limits = {
            "robot_description_planning": load_yaml("moveit_joint_limits.yaml")
        }
        sim_time = {"use_sim_time": True}

        # Rviz node
        rviz_config_file = os.path.join(
            package_dir, "resource", "moveit_visualization.rviz"
        )

        use_rviz = LaunchConfiguration("rviz", default=True)
        launch_description_nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    sim_time,
                ],
                condition=IfCondition(use_rviz),
            )
        )

        # MoveIt2 node
        movegroup = {"move_group": load_yaml("moveit_movegroup.yaml")}
        moveit_controllers = {
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_simple_controller_manager": load_yaml("moveit_controllers.yaml"),
        }

        launch_description_nodes.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    moveit_controllers,
                    movegroup,
                    description_joint_limits,
                    sim_time,
                ],
            )
        )

    else:
        launch_description_nodes.append(
            LogInfo(
                msg='"moveit" package is not installed, \
                                                please install it in order to run this demo.'
            )
        )

    return LaunchDescription(launch_description_nodes)
