import numpy as np
import rospy
from soft_hand_grasping.robot import Franka, QBSoftHand

from utils import load_config


class FrankaQBSoftHand(Franka):
    def __init__(self, config):
        super().__init__(config)

        self.hand = MyQBSoftHand(config.hand)


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
    rospy.sleep(2)
    franka.hand.reset()

    grasp_point_offset = np.array([-0.14, 0, 0.035])

    # Get the from detection module and camera
    obj_center = np.array([0.52664231, 0.14654924, -0.00715971])

    container_position_dict = {
        'r': {'square':     np.array([0.50064219, -0.28610462, 0.18627774])-grasp_point_offset,
              'Semicircle': np.array([0.49459091, -0.1813342, 0.18790609])-grasp_point_offset,
              'triangle':   np.array([0.50670854, -0.36087837, 0.18536153])-grasp_point_offset},
        'g': {'square':     np.array([0.29874306, -0.25329639, 0.21647217])-grasp_point_offset,
              'Semicircle': np.array([0.30010505, -0.35297543, 0.20628493])-grasp_point_offset,
              'triangle':   np.array([0.30048699, -0.17938881, 0.23803249])-grasp_point_offset},
        'b': {'square':     np.array([0.09447825, -0.35188347, 0.1927359])-grasp_point_offset,
              'Semicircle': np.array([0.09826421, -0.17446814, 0.29901948])-grasp_point_offset,
              'triangle':   np.array([0.08608308, -0.43524722, 0.18199256])-grasp_point_offset}
        }

    # grasp_point_offset = np.arrag([-0.18, -0.04, 0.1])
    while True:
        cmd = int(input("\n[0: Reset / 1: Pick / 2: close / 3: open / 4: Place / 9: EXIT] CMD: "))
        if cmd == 1:
            grasp_point_offset = list(map(float, input("grasp point offset (-0.14 0 0.035): ").split()))
            if len(grasp_point_offset) == 0:
                grasp_point_offset = [-0.14, 0, 0.035]

            grasp_point_offset = np.array(grasp_point_offset)
            desired_position = obj_center + grasp_point_offset
            desired_euler_angles = np.array([-179.9, -1., 0.1]) * np.pi / 180.
            print(f"desired_position: {desired_position}")

            # Quickly to the middle
            achieved_pose = franka.get_eef_achieved_pose()
            temp = (desired_position + achieved_pose[:3]) / 2
            franka.go_to(temp, desired_euler_angles, step_len=1e-5)
            # Slowly to reach the end
            franka.go_to(desired_position, desired_euler_angles, step_len=5e-6)

        elif cmd == 2:
            franka.close_fingers(width=0.25)

        elif cmd == 3:
            franka.open_fingers(width=1)
            franka.close_fingers(width=0.65)

        elif cmd == 4:
            shape, color = input("Container: ([color shape] r square): ").split()

            container_position = container_position_dict[color][shape].copy()

            target_position = container_position + grasp_point_offset
            target_position[2] += 0.1
            target_euler_angles = np.array([-179.9, -1., 0.1]) * np.pi / 180.
            franka.go_to(target_position, target_euler_angles, step_len=1e-5)
            target_position[2] -= 0.1
            franka.go_to(target_position, target_euler_angles, step_len=1e-5)

            # delta_position = list(map(float, input("Delta Position (0.05 0 0.05): ").split()))
            # delta_position = np.array(delta_position)
            # achieved_pose = franka.get_eef_achieved_pose()

            # desired_position = achieved_pose[:3] + delta_position
            # print(f"Desired position = {desired_position}")
            # franka.go_to(desired_position, achieved_pose[3:], step_len=1e-4)
            # print(f"Achieve position = {franka.get_eef_achieved_pose()[:3]}")

        elif cmd == 0:
            franka.reset()

        elif cmd == 9:
            rospy.signal_shutdown("Ctrl+c")
            exit()
