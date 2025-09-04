from rclpy.action import ActionClient
from moveit_msgs.msg import MotionPlanRequest, JointConstraint, Constraints, PlanningOptions
from moveit_msgs.action import MoveGroup
from copy import deepcopy
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
import time
import numpy as np


joints_to_consider = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

# MoveGroupActionClient implementation for direct use
class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__("moveit_plan_execute_python")

        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = (
            self.get_clock().now().to_msg()
        )
        self.motion_plan_request.workspace_parameters.header.frame_id = "base_link"
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        self.motion_plan_request.start_state.is_diff = True

        jc = JointConstraint()
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0

        joint_angles = [0, -106, -148, 0, -59, 90]  # NUC REST
        joint_angles = [0, 15, -130, 0, 55, 90]  # HOME
        joint_angles = [ja * 3.141592 / 180 for ja in joint_angles]

        joints = {}
        joints["joint_1"] = joint_angles[0]
        joints["joint_2"] = joint_angles[1]
        joints["joint_3"] = joint_angles[2]
        joints["joint_4"] = joint_angles[3]
        joints["joint_5"] = joint_angles[4]
        joints["joint_6"] = joint_angles[5]

        constraints = Constraints()
        for joint, angle in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))

        self.motion_plan_request.goal_constraints.append(constraints)

        self.motion_plan_request.pipeline_id = "move_group"
        self.motion_plan_request.group_name = "manipulator"
        self.motion_plan_request.num_planning_attempts = 10
        self.motion_plan_request.allowed_planning_time = 5.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.5
        self.motion_plan_request.max_acceleration_scaling_factor = 0.5
        self.motion_plan_request.max_cartesian_speed = 0.0

        self.planning_options = PlanningOptions()
        self.planning_options.plan_only = False
        self.planning_options.look_around = False
        self.planning_options.look_around_attempts = 0
        self.planning_options.max_safe_execution_cost = 0.0
        self.planning_options.replan = True
        self.planning_options.replan_attempts = 10
        self.planning_options.replan_delay = 0.1

        self._action_client = ActionClient(self, MoveGroup, "/move_action")

    def set_joint_angles(self, joint_angles):
        # Overwrite constraints with new joint angles
        constraints = Constraints()
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        for i, angle in enumerate(joint_angles):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = angle
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        self.motion_plan_request.goal_constraints = [constraints]

    def send_goal(self):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.motion_plan_request
        goal_msg.planning_options = self.planning_options

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(str(future.result()))
        # Do not shutdown rclpy here, as this is used as a helper in other nodes

    def feedback_callback(self, feedback_msg):
        # self.get_logger().info(str(feedback_msg))
        pass


class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__("IKAC")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 1)
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("IK service not available, waiting again...")

    def joint_states_cb(self, msg):
        joint_state = JointState()
        joint_state.header = msg.header
        for j in joints_to_consider:
            if j in msg.name:
                joint_state.name.append(j)
                joint_state.position.append(msg.position[msg.name.index(j)])
                joint_state.velocity.append(msg.velocity[msg.name.index(j)])
                joint_state.effort.append(msg.effort[msg.name.index(j)])
        self.joint_state = joint_state

    def get_ik(self, pose_stamped, group_name="manipulator"):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.robot_state.joint_state = deepcopy(self.joint_state)
        req.ik_request.avoid_collisions = True
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().error_code.val == 1:
            joint_angles = []
            joint_state = future.result().solution.joint_state
            for j in joints_to_consider:
                if j in joint_state.name:
                    idx = joint_state.name.index(j)
                    joint_angles.append(joint_state.position[idx])
            return joint_angles
        else:
            self.get_logger().error("IK failed or returned no solution.")
            return None


class CompleteEpisode(Node):
    def __init__(self):
        super().__init__('complete_episode')
        self.cli = self.create_client(Trigger, '/randomize_cube')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        self.req = Trigger.Request()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.done = False
        self.run_episode()

    def run_episode(self):
        if self.done:
            return

        moveit_client = MoveGroupActionClient()
        moveit_client.set_joint_angles([0., 0.34, -1.34, 0., -1.44, 1.56])
        moveit_client.send_goal()
        # Wait for MoveGroup action to finish
        while not hasattr(moveit_client, '_get_result_future'):
            rclpy.spin_once(moveit_client, timeout_sec=0.1)
        while not moveit_client._get_result_future.done():
            rclpy.spin_once(moveit_client, timeout_sec=0.1)
        
        self.get_logger().info('Calling /randomize_cube service...')
        future = self.cli.call_async(self.req)
        future.add_done_callback(self._randomize_done)

    def _randomize_done(self, future):
        self.get_logger().info('Cube randomized. Waiting 1 second...')
        time.sleep(1)
        self.tf_buffer.clear()
        trans = None
        while trans is None:
            try:
                rclpy.spin_once(self)  # allow processing of incoming joint states
                trans = self.tf_buffer.lookup_transform('base_link', 'cube', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
                self.get_logger().info(f'Cube pose: {trans.transform}')
                position = trans.transform.translation
                orientation = trans.transform.rotation
            except Exception as e:
                self.get_logger().warn(f'Waiting for transform: {e}')
                trans = None
                time.sleep(0.1)

        # Prepare IK client
        ik_client = MinimalClientAsync()
        # Wait for joint state
        while ik_client.joint_state is None:
            rclpy.spin_once(ik_client)

        # Step 1: Move above the cube
        above_pose = PoseStamped()
        above_pose.header.frame_id = 'base_link'
        above_pose.pose.position.x = position.x
        above_pose.pose.position.y = position.y
        above_pose.pose.position.z = position.z + 0.1
        above_pose.pose.orientation = orientation
        # Step 2: Move to the cube
        at_pose = above_pose
        at_pose.pose.position.z = position.z

        at_joint_angles = ik_client.get_ik(at_pose)
        if at_joint_angles:
            moveit_client = MoveGroupActionClient()
            moveit_client.set_joint_angles(at_joint_angles)
            moveit_client.send_goal()
            self.get_logger().info('Moved to the cube. Sequence complete.')
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = CompleteEpisode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
