import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_srvs.srv import Trigger
import h5py
import numpy as np
from cv_bridge import CvBridge


class RobomimicLogger(Node):
    def __init__(self):
        super().__init__('robomimic_logger')
        self.bridge = CvBridge()
        self.reset_episode()
        self.episode_idx = 0
        self.h5 = h5py.File('robomimic_dataset.hdf5', 'a')

        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Image, '/Gen3/kinova_color/image_color', self.image_cb, 10)
        self.finish_srv = self.create_service(Trigger, '/finish_episode', self.finish_episode_cb)
        self.start_srv = self.create_service(Trigger, '/start_episode', self.start_episode_cb)

    def start_episode_cb(self, request, response):
        self.reset_episode()
        response.success = True
        response.message = "Episode buffers reset. Ready to record."
        return response

    def reset_episode(self):
        self.joints = []
        self.left_finger = []
        self.images = []
        # self.actions will be constructed at episode end using next-state-as-action
        self.actions = None

    def joint_cb(self, msg):
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'left_finger_joint']
        joint_pos = [msg.position[msg.name.index(j)] for j in joint_names]
        self.joints.append(joint_pos)
        # also record left finger separately for convenience
        try:
            lf = msg.position[msg.name.index('left_finger_joint')]
            self.left_finger.append(lf)
        except Exception:
            pass

    def image_cb(self, msg):
        # keep images channel-last (H, W, C) so HDF5 matches robomimic expected layout
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        img = np.asarray(cv_img, dtype=np.uint8)  # (H, W, C)
        self.images.append(img)

    def finish_episode_cb(self, request, response):
        try:
            T = min(len(self.joints), len(self.images))
            if T < 2:
                response.success = False
                response.message = "Not enough data to save for this episode."
                return response
            # Next-state-as-action: action[t] = joints[t+1] for t in 0..T-2
            actions = np.array(self.joints[1:T], dtype=np.float32)
            # Truncate obs and images to T-1 to align
            joints_arr = np.array(self.joints[:T-1], dtype=np.float32)
            left_finger_arr = np.array(self.left_finger[:T-1], dtype=np.float32)
            images_arr = np.array(self.images[:T-1], dtype=np.uint8)
            grp = self.h5.create_group(f'data/{self.episode_idx:04d}')
            grp.create_dataset('actions', data=actions)
            # store observations under data/<ep>/obs/<key> as robomimic expects
            obs_grp = grp.create_group('obs')
            obs_grp.create_dataset('joints', data=joints_arr)
            # left finger may be empty if not populated
            if left_finger_arr.size > 0:
                obs_grp.create_dataset('left_finger_joint', data=left_finger_arr)
            obs_grp.create_dataset('image', data=images_arr)
            # record number of samples for this episode (used by utilities)
            grp.attrs['num_samples'] = joints_arr.shape[0]
            self.h5.flush()
            self.episode_idx += 1
            self.reset_episode()
            response.success = True
            response.message = f"Episode {self.episode_idx-1:04d} saved."
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobomimicLogger()
    rclpy.spin(node)
    node.h5.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
