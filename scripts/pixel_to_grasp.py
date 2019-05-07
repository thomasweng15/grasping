
import os
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np 
import skimage
from pyquaternion import Quaternion as Quat
import cv2
from shared import *
np.set_printoptions(suppress=True)

def get_planned_grasp(im_depth, px_coords, theta, pose_tool0home_w):
    """
    Get tool0 grasp tf based on depth image and pixel coordinates of grasp.
    """
    K = np.array([
        [618.976990, 0.0, 324.686005],
        [0.0, 618.997620, 234.163773],
        [0.0, 0.0, 1.0]
    ])
    pose_eegrasp_c = get_pose_eegrasp_c(im_depth, K, px_coords, theta)
    
    # Account for tool0 offset from ee
    pose_tool0grasp_c = set_eetool0_offset(pose_eegrasp_c)

    # Convert grasp position to world pose
    tf_tool0_camera = np.array([
        [0.36685286, 0.93018527, -0.01320435, -0.03807055],
        [-0.9302708, 0.3668722, -0.00101334, -0.0164097],
        [0.00390172, 0.01265536, 0.99991231, 0.14671274],
        [0.,         0.,         0.,         1.]])

    tf_tool0home_tool0grasp = get_tool0_tf(pose_tool0home_w, pose_tool0grasp_c, tf_tool0_camera)
    quat = Quat(matrix=tf_tool0home_tool0grasp[0:3,0:3])
    print(quat.elements)

    return tf_tool0home_tool0grasp


def get_pose_eegrasp_c(im_depth, K, px_coords, theta):
    """
    Get end effector grasp pose in overhead camera frame.
    """
    px_y, px_x = px_coords
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    z = im_depth[px_y, px_x] / 1000.0
    x = z * np.abs(px_x - cx) / fx
    y = z * np.abs(px_y - cy) / fy
    quat = Quat(axis=[0.0, 0.0, 1.0], radians=theta)
    # print(quat.elements)
    qelts = quat.elements
    pose_eegrasp_c = np.array([x, y, z, qelts[1], qelts[2], qelts[3], qelts[0]])
    return pose_eegrasp_c

def set_eetool0_offset(pose_eegrasp_c, ee_offset=-0.226):
    """
    Take the end effector grasp pose and return the tool0 grasp pose.
    """
    pose_tool0grasp_c = np.copy(pose_eegrasp_c)
    pose_tool0grasp_c[2] += ee_offset
    return pose_tool0grasp_c

def get_tool0_tf(pose_tool0home_w, pose_tool0grasp_c, tf_tool0_camera):
    """
    Return the tool0 transformation to send to the motion planner.
    """
    tf_world_tool0home = get_tf_from_pose(pose_tool0home_w)
    tf_world_camerahome = tf_world_tool0home.dot(tf_tool0_camera)

    tf_camerahome_tool0grasp = get_tf_from_pose(pose_tool0grasp_c)
    tf_world_tool0grasp = tf_world_camerahome.dot(tf_camerahome_tool0grasp)

    tf_tool0home_tool0grasp = np.linalg.inv(tf_world_tool0home).dot(tf_world_tool0grasp)

    if True:
        tfs = [
            tf_world_tool0home,
            tf_world_camerahome,
            tf_world_tool0grasp
        ]
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        x = [tf[0,3] for tf in tfs]
        y = [tf[1,3] for tf in tfs]
        z = [tf[2,3] for tf in tfs]
        ax.scatter3D(x, y, z, s=50)

        vectors = get_dir_vecs(tfs)
        for tf, v, color in vectors:
            xline = [tf[0,3], tf[0,3]+0.1*v[0]]
            yline = [tf[1,3], tf[1,3]+0.1*v[1]]
            zline = [tf[2,3], tf[2,3]+0.1*v[2]]

            ax.plot3D(xline, yline, zline, c=color)

        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(0, 1)
        ax.set_zlim(0, 1)
        plt.show('3d.png')

    return tf_tool0home_tool0grasp

def get_dir_vecs(tfs):
    unit_x = [
        [1, 0, 0, 1],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]
    unit_y = [
        [1, 0, 0, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]
    unit_z = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1]
    ]
    vecs = []
    for tf in tfs:
        vec = tf*unit_x
        vecs.append((tf, vec[0:3, 3], 'red'))
        vec = tf*unit_y
        vecs.append((tf, vec[0:3, 3], 'green'))
        vec = tf*unit_z
        vecs.append((tf, vec[0:3, 3], 'blue'))

    return vecs

if __name__ == '__main__':
    # Get inputs
    dirname="testrun/"
    pixels, theta, im_overhead, _ = np.load(dirname + "params.npy")
    pixels= np.rint(pixels).astype(int)
    print(pixels, theta)

    pose_tool0home_w = get_pose(dirname, "homeSensorSnapshot.yaml")
    im_depth = get_im(dirname, "homeDepthImage")
    planned_grasp_tf = get_planned_grasp(im_depth, pixels, theta, pose_tool0home_w)

    # Plot grasp point in overhead rgb image
    if False:
        plt.imshow(im_overhead)
        plt.scatter(pixels[0], pixels[1], c='red')
        plt.plot([pixels[0]+10*np.cos(theta), pixels[0]+30*np.cos(theta)], [pixels[1]+10*np.sin(theta), pixels[1]+30*np.sin(theta)], color='red')
        plt.plot([pixels[0]-10*np.cos(theta), pixels[0]-30*np.cos(theta)], [pixels[1]-10*np.sin(theta), pixels[1]-30*np.sin(theta)], color='red') 
        plt.show()

