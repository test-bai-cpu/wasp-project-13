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

    # Get the from detection module and camera
    obj_center = np.array([0.52664231, 0.14654924, -0.00715971])

    # grasp_point_offset = np.arrag([-0.18, -0.04, 0.1])
    while True:
        cmd = int(input("\n[0: Reset / 1: Go to / 2: close / 3: open / 9: EXIT] CMD: "))
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

        elif cmd == 0:
            franka.reset()

        elif cmd == 9:
            rospy.signal_shutdown("Ctrl+c")
            exit()
