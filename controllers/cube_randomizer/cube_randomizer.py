"""
Supervisor controller for Webots to randomly reposition a cube (DEF GREEN) within specified boundaries.

"""
X_MIN, X_MAX = 0.43, 0.66
Y_MIN, Y_MAX = -0.21, 0.21
Z_FIXED = 0.43

import random
from controller import Supervisor
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger
import math
import numpy as np
from tf_transformations import quaternion_matrix, translation_matrix, quaternion_from_matrix
import tf_transformations

BASE_DEF = 'GEN3'  # base_link
CUBE_DEF = 'GREEN'

TIME_STEP = 32  # ms, typical Webots timestep
CHANGE_INTERVAL = 5.0  # seconds


class CubeRandomizer(Supervisor):
    def __init__(self):
        super().__init__()
        self.cube = self.getFromDef(CUBE_DEF)
        self.base = self.getFromDef(BASE_DEF)
        if self.cube is None:
            print(f"[ERROR] Could not find DEF {CUBE_DEF} in the world.")
            exit(1)
        if self.base is None:
            print(f"[ERROR] Could not find DEF {BASE_DEF} in the world.")
            exit(1)
        # ROS2 node and tf2 broadcaster
        rclpy.init(args=None)
        self.node = rclpy.create_node('cube_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.srv = self.node.create_service(Trigger, 'randomize_cube', self.handle_randomize_cube)

    def randomize_position(self):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        z = Z_FIXED
        self.cube.getField('translation').setSFVec3f([x, y, z])
        # Random orientation only around Z axis (yaw)
        axis = [0, 0, 1]
        angle = random.uniform(0, 2 * math.pi)
        self.cube.getField('rotation').setSFRotation(axis + [angle])
        print(f"[INFO] Cube moved to: x={x:.3f}, y={y:.3f}, z={z:.3f}, axis={axis}, angle={angle:.2f}")

    def handle_randomize_cube(self, request, response):
        self.randomize_position()
        response.success = True
        response.message = "Cube randomized."
        return response

    def publish_cube_tf(self):
        # Get cube pose in world
        cube_translation = self.cube.getField('translation').getSFVec3f()
        cube_rotation = self.cube.getField('rotation').getSFRotation()  # axis-angle: [x, y, z, theta]
        # Get base_link pose in world
        base_translation = self.base.getField('translation').getSFVec3f()
        base_rotation = self.base.getField('rotation').getSFRotation()

        # Compute cube pose w.r.t. base_link (GEN3)
        def tf_matrix(translation, rotation):
            x, y, z = translation
            rx, ry, rz, theta = rotation
            # Axis-angle to quaternion
            qx = rx * math.sin(theta/2)
            qy = ry * math.sin(theta/2)
            qz = rz * math.sin(theta/2)
            qw = math.cos(theta/2)
            T = np.dot(translation_matrix([x, y, z]), quaternion_matrix([qx, qy, qz, qw]))
            return T

        T_base = tf_matrix(base_translation, base_rotation)
        T_cube = tf_matrix(cube_translation, cube_rotation)
        T_base_inv = np.linalg.inv(T_base)
        T_cube_in_base = np.dot(T_base_inv, T_cube)
        # Extract translation and quaternion
        trans = T_cube_in_base[:3, 3]
        quat = quaternion_from_matrix(T_cube_in_base)

        # Extract yaw (theta) from cube's axis-angle rotation (around Z)
        axis = cube_rotation[:3]
        theta = cube_rotation[3]

        # Convert axis-angle to quaternion
        qx = axis[0] * np.sin(theta/2)
        qy = axis[1] * np.sin(theta/2)
        qz = axis[2] * np.sin(theta/2)
        qw = np.cos(theta/2)
        # Convert quaternion to euler
        euler = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
        yaw = euler[2]
        # Normalize yaw to [-90°, 90°] for minimal rotation (cube symmetry)
        yaw_deg = np.degrees(yaw)
        # First, wrap to [-180, 180]
        while yaw_deg > 180:
            yaw_deg -= 360
        while yaw_deg < -180:
            yaw_deg += 360
        # Then, wrap to [-90, 90]
        while yaw_deg > 90:
            yaw_deg -= 180
        while yaw_deg < -90:
            yaw_deg += 180
        yaw_norm = np.radians(yaw_deg)
        # Compose quaternion for wrapped yaw
        wrapped_quat = tf_transformations.quaternion_about_axis(yaw_norm, [0,0,1])
        # Apply 180-degree rotation about X to flip Z axis down
        flip_quat = tf_transformations.quaternion_about_axis(np.pi, [1,0,0])
        quat = tf_transformations.quaternion_multiply(flip_quat, wrapped_quat)
        print(f"[INFO] Cube yaw (theta): {yaw_norm:.3f} rad ({yaw_deg:.1f} deg) [wrapped to ±90°, Z flipped down, TF published]")

        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'cube'
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

    def run(self):
        try:
            while self.step(TIME_STEP) != -1:
                self.publish_cube_tf()
                rclpy.spin_once(self.node, timeout_sec=0.01)
        finally:
            rclpy.shutdown()

if __name__ == "__main__":
    randomizer = CubeRandomizer()
    randomizer.run()
