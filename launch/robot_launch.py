#!/usr/bin/env python

import os
import launch
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


package_dir = get_package_share_directory("webots_gen3")


# Define all the ROS 2 nodes that need to be restart on simulation reset here
def get_ros2_nodes(*args):
    # Driver node
    ros2_control_params = os.path.join(package_dir, "config", "ros2_controllers.yaml")
    driver = WebotsController(
        robot_name="Gen3",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "config", "ros2_control.urdf"
                )
            },
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            ros2_control_params,
        ],
    )

    # ROS2 control spawners for Gen3
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )

    robotiq_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=[
            "robotiq_gripper_controller",
            "-c",
            "/controller_manager",
        ]
        + controller_manager_timeout,
    )

    right_finger_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=[
            "right_finger_controller",
            "-c",
            "/controller_manager",
        ]
        + controller_manager_timeout,
    )

    ros2_control_spawners = [
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
        robotiq_gripper_controller,
        right_finger_controller_spawner,
    ]

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=driver, nodes_to_start=ros2_control_spawners
    )

    initial_manipulator_positioning = Node(
        package="webots_gen3",
        executable="retract_manipulator",
        output="screen",
    )

    mimic_gripper_finger_node = Node(
        package="webots_gen3",
        executable="mimic_gripper_finger",
        output="screen",
    )
    return [driver, waiting_nodes, mimic_gripper_finger_node]  # , initial_manipulator_positioning]


def generate_launch_description():
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", "gen3.wbt"])
    )
    ros2_supervisor = Ros2SupervisorLauncher()

    with open(os.path.join(package_dir, "resource", "gen3.urdf")) as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": True,
            }
        ],
    )

    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription(
        [
            webots,
            ros2_supervisor,
            robot_state_publisher,
            webots_event_handler,
            reset_handler,
        ]
        + get_ros2_nodes()
    )
