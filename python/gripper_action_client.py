#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import GripperCommand


class GripperClient(Node):

    def __init__(self):
        super().__init__('gripper_action_client')
        # Action server is usually "<controller_name>/gripper_cmd"
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

    def send_goal(self, position: float, max_effort: float):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position      # Desired position (e.g., 0.8 to close, 0.0 to open)
        goal_msg.command.max_effort = max_effort  # Limit force/torque

        self.get_logger().info(f'Sending goal: position={position}, max_effort={max_effort}')

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    gripper_client = GripperClient()

    # Example: close gripper
    future = gripper_client.send_goal(position=0.8, max_effort=50.0)

    rclpy.spin_until_future_complete(gripper_client, future)

    result = future.result()
    if result and result.accepted:
        gripper_client.get_logger().info('Goal accepted, waiting for result...')
        result_future = result.get_result_async()
        rclpy.spin_until_future_complete(gripper_client, result_future)
        gripper_client.get_logger().info(f'Result: {result_future.result().result}')
    else:
        gripper_client.get_logger().warn('Goal rejected.')

    gripper_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
