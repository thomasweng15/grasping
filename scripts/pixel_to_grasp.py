
import os
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np 
import skimage
from pyquaternion import Quaternion as Quat
import cv2
np.set_printoptions(suppress=True)



if __name__ == '__main__':
    # Get inputs
    dirname="testrun/"
    px_coords, theta, im_overhead, _ = np.load(dirname + "params.npy")
    px_coords = np.rint(px_coords).astype(int)
    px_x, px_y = px_coords

    # Plot grasp point in overhead rgb image
    # plt.imshow(im_overhead)
    # plt.scatter(px_coords[0], px_coords[1], c='red')
    # plt.show()

    # Get grasp position in overhead camera frame
    K = np.array([
        [618.976990, 0.0, 324.686005],
        [0.0, 618.997620, 234.163773],
        [0.0, 0.0, 1.0]
    ])
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    im_depth = get_im(dirname, "homeDepthImage")
    z = im_depth[px_y, px_x] / 1000.0
    x = z * np.abs(px_x - cx) / fx
    y = z * np.abs(px_y - cy) / fy
    pose_eegrasp_c = np.array([x, y, z, 0, 0, 0, 1])
    
    # Account for tool0 offset from ee
    ee_offset = -0.226
    pose_tool0grasp_c = np.copy(pose_eegrasp_c)
    pose_tool0grasp_c[2] += ee_offset

    # Convert grasp position to world pose
    pose_tool0home_w = get_pose(dirname, "homeSensorSnapshot.yaml")

    tf_tool0_camera = np.array([
        [0.36685286, 0.93018527, -0.01320435, -0.03807055],
        [-0.9302708, 0.3668722, -0.00101334, -0.0164097],
        [0.00390172, 0.01265536, 0.99991231, 0.14671274],
        [0.,         0.,         0.,         1.]])

    tf_world_tool0home = get_tf_from_pose(pose_tool0home_w)
    tf_world_camerahome = tf_world_tool0home.dot(tf_tool0_camera)

    tf_camerahome_tool0grasp = get_tf_from_pose(pose_tool0grasp_c)
    tf_world_tool0grasp = tf_world_camerahome.dot(tf_camerahome_tool0grasp)

    tf_tool0home_tool0grasp = np.linalg.inv(tf_world_tool0home).dot(tf_world_tool0grasp)

    if True:
        poses = [
            pose_tool0home_w, 
            tf_world_camerahome[0:3, 3], 
            tf_world_tool0grasp[0:3,3]
        ]
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        x = [p[0] for p in poses]
        y = [p[1] for p in poses]
        z = [p[2] for p in poses]
        ax.scatter3D(x, y, z, s=50)

        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(0, 1)
        ax.set_zlim(0, 1)
        plt.show('3d.png')
