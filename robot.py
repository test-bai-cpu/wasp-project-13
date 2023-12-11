import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
from soft_hand_grasping.robot import Franka, QBSoftHand

from utils import load_config


"""
grasp_pose: The pose we manually define for grasping object
eef_pose: The pose controller use to move the robot.
          It only a z-axis offset in Franka two finger gripper but it's more complex in QB hand.
"""


class FrankaQBSoftHand(Franka):
    def __init__(self, config):
        super().__init__(config)

        self.hand = MyQBSoftHand(config.hand)
        self.eef_T_grasp = None

    def get_eef_pose_from_grasp_pose(self, grasp_pose):

        r_T_grasp = np.identity(4)
        r_T_grasp[:3, 3] = grasp_pose[:3]
        r_T_grasp[:3, :3] = R.from_euler('XYZ', grasp_pose[3:]).as_matrix()

        r_T_eef = np.dot(r_T_grasp, np.linalg.inv(self.eef_T_grasp))
        position = r_T_eef[:3, 3]
        euler = R.from_matrix(r_T_eef[:3, :3]).as_euler('XYZ')
        return position, euler


class MyQBSoftHand(QBSoftHand):
    def __init__(self, config):
        super().__init__(config)

    def reset(self):
        rospy.loginfo("Reset QB Soft Hand")
        self.open_fingers()
        self.close_fingers(0.65)


if __name__ == '__main__':
    rospy.init_node('robot', anonymous=True)

    config = load_config("./config/franka_qbhand.yaml")
    franka = FrankaQBSoftHand(config)
    hand = MyQBSoftHand(config.hand)

    # TODO: Get the from detection module and camera
    obj_center = np.array([0.52664231, 0.14654924, -0.00715971])

    # Manually define
    container_position_dict = {
        'r': {'square':     np.array([ 0.6587, -0.2903,  0.1815]),
              'Semicircle': np.array([ 0.6527, -0.1903,  0.1815]),
              'triangle':   np.array([ 0.6467, -0.3703,  0.1815])},
        'g': {'square':     np.array([ 0.4587, -0.2903,  0.1815]),  # v
              'Semicircle': np.array([ 0.4527, -0.1903,  0.1815]),
              'triangle':   np.array([ 0.4467, -0.3703,  0.1815])},
        'b': {'square':     np.array([ 0.2587, -0.2903,  0.1815]),
              'Semicircle': np.array([ 0.2527, -0.1903,  0.1815]),
              'triangle':   np.array([ 0.2467, -0.3703,  0.1815])}
        }

    while True:
        cmd = int(input("\n[0: Reset / 1: Pick / 2: close / 3: open / 4: Check pose / 5: Place / 9: EXIT] CMD: "))
        if cmd == 1:    # Pick
            grasp_point_offset = list(map(float, input("grasp point offset (-0.14 0 0.035): ").split()))
            if len(grasp_point_offset) == 0:
                franka.eef_T_grasp = np.identity(4)
                franka.eef_T_grasp[:3, 3] = np.array([-0.14, 0, 0.035])

            else:
                grasp_point_offset = np.array(grasp_point_offset)
                # desired_position = obj_center + grasp_point_offset
                # desired_euler = np.array([-179.9, -1., 0.1]) * np.pi / 180.
                # print(f"desired_position: {desired_position}")

            # r_T_grasp = np.identity(4)
            # r_T_grasp[:3, 3] = obj_center
            # r_T_grasp[:3, :3] = R.from_euler('XYZ', [-3.1415, -0.017453, 0.00175]).as_matrix()

            grasp_pose = np.concatenate([
                obj_center, np.array([-3.1415, -0.017453, 0.00175])])
            desired_position, desired_euler = \
                franka.get_eef_pose_from_grasp_pose(grasp_pose)

            # Quickly to the middle
            achieved_pose = franka.get_eef_achieved_pose()
            temp = (desired_position + achieved_pose[:3]) / 2
            temp[2] += 0.1
            franka.go_to(temp, desired_euler, step_len=1e-5)
            # Slowly to reach the end
            franka.go_to(desired_position, desired_euler, step_len=5e-6)

        elif cmd == 2:  # close
            franka.close_fingers(width=0.25)

        elif cmd == 3:  # open
            franka.open_fingers(width=1)
            franka.close_fingers(width=0.65)

        elif cmd == 4:  # Check pose
            # -0.907571211 = -52 degree match camera's 38 degree
            # 12 degree
            desired_position = np.array([0.347, 0.143, 0.333])
            desired_euler = np.array([-0.907571211, -0, 0.20943951])
            franka.go_to(desired_position, desired_euler, step_len=5e-6)

            sub_cmd = input("Go back to normal pose? [Y/n]: ")
            if sub_cmd in ['', 'Y', 'y']:
                target_position = np.array([0.5, 0., 0.4])
                target_euler = np.array([-np.pi, 0., 0.])
                franka.go_to(target_position, target_euler, step_len=5e-6)

        elif cmd == 5:  # Place
            color, shape = input("Container: ([color shape] r square): ").split()

            container_position = container_position_dict[color][shape].copy()

            grasp_pose = np.concatenate([
                container_position, np.array([-3.1415, -0.017453, 0.00175])])
            target_position, target_euler = \
                franka.get_eef_pose_from_grasp_pose(grasp_pose)

            target_position[2] += 0.1
            franka.go_to(target_position, target_euler, step_len=1e-5)
            target_position[2] -= 0.1
            franka.go_to(target_position, target_euler, step_len=5e-6)

            # target_position = container_position + grasp_point_offset
            # target_position[2] += 0.1
            # target_euler_angles = np.array([-179.9, -1., 0.1]) * np.pi / 180.
            # franka.go_to(target_position, target_euler_angles, step_len=1e-5)
            # target_position[2] -= 0.1
            # franka.go_to(target_position, target_euler_angles, step_len=1e-5)

            # HERE: 12/11
            # TODO: Use this to try container's pose
            # TEST:
            delta_z = float(input("Delta theta_z (5): "))
            delta_rot = R.from_euler('XYZ', np.array([0, 0, delta_z]), degrees=True).as_matrix()

            # Curent
            achieved_pose = franka.get_eef_achieved_pose()
            achieved_rot = R.from_euler('XYZ', achieved_pose[3:]).as_matrix()

            # Target (EEF pose: Extrinsic rotation --- Fixed order)
            desired_rot = np.dot(delta_rot, achieved_rot)
            desired_position = achieved_pose[:3]
            desired_euler = R.from_matrix(desired_rot).as_euler('XYZ')

            # Convert grasp pose to End-effector pose
            grasp_pose = np.concatenate([desired_position, desired_euler])
            target_position, target_euler = \
                franka.get_eef_pose_from_grasp_pose(grasp_pose)
            franka.go_to(target_position, target_euler, step_len=1e-5)

            rospy.sleep(1)
            achieved_pose = franka.get_eef_achieved_pose()
            t, r = achieved_pose[:3], achieved_pose[3:] * 180 / np.pi
            rospy.loginfo(f"\n\nAchieved pose: \n    {t}, \n    {r}")

            # delta_position = list(map(float, input("Delta Position (0.05 0 0.05): ").split()))
            # delta_position = np.array(delta_position)
            # achieved_pose = franka.get_eef_achieved_pose()

            # desired_position = achieved_pose[:3] + delta_position
            # print(f"Desired position = {desired_position}")
            # franka.go_to(desired_position, achieved_pose[3:], step_len=1e-4)
            # print(f"Achieve position = {franka.get_eef_achieved_pose()[:3]}")

        elif cmd == 0:
            sub_cmd = input("Hand only [Y/n]?")
            if sub_cmd in ['', 'Y', 'y']:
                rospy.sleep(1)
                franka.hand.reset()
            else:
                franka.reset()

        elif cmd == 9:
            rospy.signal_shutdown("Ctrl+c")
            exit()
