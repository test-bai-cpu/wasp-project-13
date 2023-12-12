#!/usr/bin/env python3
# from __future__ import division

import numpy as np
from scipy.spatial.transform import Rotation as R
from soft_hand_grasping.robot import Franka, QBSoftHand

# ROS
import rospy
import std_msgs

from utils import load_config
from robot import FrankaQBSoftHand


class ControlModule:
    def __init__(self):

        self.active_topic = "/active/control_module"
        self.plan_topic = "/llm_plan"

        self.ready = False
        config = load_config("./config/franka_qbhand.yaml")
        self.franka = FrankaQBSoftHand(config)

        self.sub_active = rospy.Subscriber(
            self.active_topic, std_msgs.msg.Bool, self.active, queue_size=1,
            buff_size=1)

        self.container_position_dict = {
        'r': {'square':     np.array([ 0.6587, -0.2903,  0.1815, 0, 0, 0]),
              'Semicircle': np.array([ 0.6527, -0.1903,  0.1815, 0, 0, 0]),
              'triangle':   np.array([ 0.6467, -0.3703,  0.1815, 0, 0, 0])},
        'g': {'square':     np.array([ 0.4587, -0.2903,  0.1815, 0, 0, 0]),  # v
              'Semicircle': np.array([ 0.4527, -0.1903,  0.1815, 0, 0, 0]),
              'triangle':   np.array([ 0.4467, -0.3703,  0.1815, 0, 0, 0])},
        'b': {'square':     np.array([ 0.2587, -0.2903,  0.1815, 0, 0, 0]),
              'Semicircle': np.array([ 0.2527, -0.1903,  0.1815, 0, 0, 0]),
              'triangle':   np.array([ 0.2467, -0.3703,  0.1815, 0, 0, 0])}
        }

        # -0.907571211 = -52 degree match camera's 38 degree
        # 12 degree
        self.pose_in_front_camera = np.array([
            0.347, 0.143, 0.333, -0.907571211, -0, 0.20943951])

    def active(self, trigger):
        if not trigger.data:
            return

        trigger.data = False
        rospy.loginfo("Trigger Control Module")

        plan_list = self.get_plan()
        for plan in plan_list:
            # pick red square
            action, color, shape = plan.split()

            if action == 'pick':
                position = self.get_object_position(color, shape)
                self.franka.pick_object(position)

            elif action == 'place':
                object_pose = self.check_object_pose()
                container_pose = self.get_container_pose(color, shape)

                obj_T_cont = self.calculate_different_transform(object_pose, container_pose)

                # HERE: 12/11
                # NOTE: This obj_T_container is the delta pose for eef
                position = obj_T_cont[:3, 3]
                euler = R.from_matrix(obj_T_cont[:3, :3]).as_euler('XYZ')


                self.franka.place_in_container(pose)

    def get_plan(self):
        llm_plan = "pick blue square\nplace green square"
        plane_list = llm_plan.split('\n')
        return plane_list

    def get_object_position(self, color, shape):
        # TODO: Get from detection module

        # TEMP
        position = np.array([0.52664231, 0.14654924, -0.00715971])
        return position

    def calculate_different_transform(self, object_pose, container_pose):

        r_T_obj = np.identity(4)
        r_T_obj[:3, 3] = object_pose[:3]
        r_T_obj[:3, :3] = R.from_euler('XYZ', object_pose[3:]).as_matrix()

        r_T_cont = np.identity(4)
        r_T_cont[:3, 3] = container_pose[:3]
        r_T_cont[:3, :3] = R.from_euler('XYZ', container_pose[3:]).as_matrix()

        obj_T_cont = np.dot(np.linalg.inv(r_T_obj), r_T_cont)

        return obj_T_cont


    def check_object_pose(self):
        self.franka.go_to(self.pose_in_front_camera[:3],
                          self.pose_in_front_camera[3:],
                          step_len=5e-6)
        # TODO: Call object pose estimateion

        # TEMP: Fake one
        object_pose = self.franka.get_grasp_pose_from_eef_pose(
            self.pose_in_front_camera)
        object_pose[3:] = np.array([0, 0, 0])
        return object_pose

    def get_container_pose(self, color, shape):
        c = color[0]
        pose = self.container_position_dict[c][shape]
        return pose


if __name__ == '__main__':
    rospy.init_node('robot', anonymous=True)

    rate = rospy.Rate(1)
    rospy.loginfo("Build Control Module...")
    control_module = ControlModule()
    rospy.loginfo("Control Module is online")

    rospy.spin()
