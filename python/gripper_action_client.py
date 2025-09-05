#!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import GripperCommand


class GripperClient(Node):

    def __init__(self):
        super().__init__('gripper_action_client')
        # Action server is usually "<controller_name>/gripper_cmd"
        self._action_name = '/robotiq_gripper_controller/gripper_cmd'
        self._action_client = ActionClient(self, GripperCommand, self._action_name)

    def send_goal(self, position: float, max_effort: float):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position      # Desired position (e.g., 0.8 to close, 0.0 to open)
        goal_msg.command.max_effort = max_effort  # Limit force/torque

        self.get_logger().info(f'Sending goal: position={position}, max_effort={max_effort}')

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

    def close(self, position: float = 0.69, max_effort: float = 50.0):
        """Close the gripper to `position` (default fully/mostly closed).

        This is a synchronous helper that sends the goal and waits for the result.
        Returns the goal result or None if rejected.
        """
        future = self.send_goal(position=position, max_effort=max_effort)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle or not getattr(goal_handle, 'accepted', False):
            self.get_logger().warn('Close goal rejected.')
            return None

        self.get_logger().info('Close goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

    def open(self, max_effort: float = 50.0):
        """Open the gripper (position 0.0) synchronously and return the goal result."""
        return self.close(position=0.0, max_effort=max_effort)


def main(args=None):
    rclpy.init(args=args)
    gripper_client = GripperClient()

    # Example: close gripper
    future = gripper_client.send_goal(position=0.7, max_effort=50.0)

    rclpy.spin_until_future_complete(gripper_client, future)

    # future = gripper_client.send_goal(position=0.0, max_effort=50.0)

    # rclpy.spin_until_future_complete(gripper_client, future)

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
