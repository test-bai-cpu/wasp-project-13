import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
from soft_hand_grasping.camera import KinectAzure

from utils import load_config, visual_image, get_pointcloud, visual_pointcloud


if __name__ == '__main__':
    rospy.init_node('camera', anonymous=True)

    config = load_config("./config/camera.yaml")
    camera = KinectAzure(config)

    bbox_top_left = (895, 945)
    bbox_width = (90, 90)
    obj_pixel_position = (940, 970)     # x, y

    # robot_T_cam (calibration) -- Nov 8
    robot_T_cam = np.identity(4)
    robot_T_cam[:3, 3] = np.array([
        0.450898, 0.5901853, 0.544763])
    robot_T_cam[:3, :3] = R.from_quat([
        0.1011432, -0.8927931, 0.4387508, 0.0137195]).as_matrix()


    while True:
        cmd = int(input("[1: Take one / 2: Take continuous / 3: EXIT] CMD: "))
        if cmd == 1:
            color_image, depth_image = camera.get_rgbd_images()
            # Crop image
            color_image[:bbox_top_left[1], :] = 0
            color_image[bbox_top_left[1]+bbox_width[1]:, :] = 0
            color_image[:, :bbox_top_left[0]] = 0
            color_image[:, bbox_top_left[0]+bbox_width[0]:] = 0
            depth_image[:bbox_top_left[1], :] = 0
            depth_image[bbox_top_left[1]+bbox_width[1]:, :] = 0
            depth_image[:, :bbox_top_left[0]] = 0
            depth_image[:, bbox_top_left[0]+bbox_width[0]:] = 0
            # visual_image(color_image, depth_image)
            # exit()

            points, _ = get_pointcloud(color_image, depth_image, camera.intrinsics)
            valid_idx = np.logical_and(
                points[:, 2] > 0.05, points[:, 2] < 1.5)
            points = points[valid_idx]
            # visual_pointcloud(points)

            obj_center = np.mean(points, axis=0)

            # Transform point cloud from camera frame to robot frame
            obj_center = \
                np.dot(robot_T_cam[0:3, 0:3], np.transpose(obj_center)) \
                + robot_T_cam[0:3, 3]

            print("obj_center (robot frame) =", obj_center)

        elif cmd == 2:
            while True:
                color_image, depth_image = camera.get_rgbd_images()
                visual_image(color_image, depth_image)
        elif cmd == 3:
            rospy.signal_shutdown("Ctrl+c")
            exit()