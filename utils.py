import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


def load_config(config_path):
    """ Loading config file. """

    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    class Struct:
        def __init__(self, **kwargs):
            for key, value in kwargs.items():
                if isinstance(value, dict):
                    setattr(self, key, Struct(**value))
                else:
                    setattr(self, key, value)

        def __getitem__(self, key):
            return self.__dict__[key]

        def __repr__(self):
            return repr(vars(self))

    return Struct(**config)


def visual_image(color_image, depth_image, show=True, save=False):
    if show:
        import matplotlib.patches as patches
        _, axes = plt.subplots(1, 2, figsize=(15, 10))
        axes = axes.flat
        for ax in axes:
            ax.axis("off")
        axes[0].set_title("Color")
        axes[0].imshow(color_image)
        axes[1].set_title("Depth")
        axes[1].imshow(depth_image)

        rect = patches.Rectangle((895, 945), 90, 90, linewidth=1, edgecolor='r', facecolor='none')
        axes[0].add_patch(rect)

        plt.show()



    if save:
        img_id = int(input("image ID: "))
        image = Image.fromarray(color_image)
        image.save(f"color_image_{img_id}.png")
        nan_mask = np.isnan(depth_image)
        depth_image[nan_mask] = 0.
        norm_depth = ((depth_image / 2.0) * 255).astype(np.uint8)

        depth = Image.fromarray(norm_depth)
        depth.save(f"depth_image_{img_id}.png")


def visual_pointcloud(points):
    import open3d as o3d
    # Create an Open3D point cloud from the NumPy array
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])


def get_pointcloud(color_img, depth_img, camera_intrinsics):
    # Get depth image size
    im_h = depth_img.shape[0]
    im_w = depth_img.shape[1]

    # Project depth into 3D point cloud in camera coordinates
    pix_x, pix_y = np.meshgrid(np.linspace(0, im_w-1, im_w),
                               np.linspace(0, im_h-1, im_h))
    cam_pts_x = np.multiply(pix_x - camera_intrinsics[0][2],
                            depth_img / camera_intrinsics[0][0])
    cam_pts_y = np.multiply(pix_y - camera_intrinsics[1][2],
                            depth_img / camera_intrinsics[1][1])
    cam_pts_z = depth_img.copy()
    cam_pts_x.shape = (im_h*im_w, 1)
    cam_pts_y.shape = (im_h*im_w, 1)
    cam_pts_z.shape = (im_h*im_w, 1)

    # Reshape image into colors for 3D point cloud
    rgb_pts_r = color_img[:, :, 0]
    rgb_pts_g = color_img[:, :, 1]
    rgb_pts_b = color_img[:, :, 2]
    rgb_pts_r.shape = (im_h*im_w, 1)
    rgb_pts_g.shape = (im_h*im_w, 1)
    rgb_pts_b.shape = (im_h*im_w, 1)

    cam_pts = np.concatenate((cam_pts_x, cam_pts_y, cam_pts_z), axis=1)
    rgb_pts = np.concatenate((rgb_pts_r, rgb_pts_g, rgb_pts_b), axis=1)

    return cam_pts, rgb_pts
