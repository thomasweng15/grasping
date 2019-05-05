
import os
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np 
import skimage
from pyquaternion import Quaternion as Quat
import cv2
np.set_printoptions(suppress=True)

def get_pose(dirname, filestr):
    files = os.listdir(dirname)
    snapshot = [f for f in files if filestr in f][0] 
    with open("%s/%s" % (dirname, snapshot), "r") as f:
        lines = [line.rstrip('\n') for line in f]
        for i, line in enumerate(lines):
            if "Tool Pose:" in line:
                pose = [float(l.split("  - ")[-1]) for l in lines[i+1:i+8]]
                return pose
    return None

def get_im(dirname, filestr):
    files = os.listdir(dirname)
    im_name = [f for f in files if filestr in f][0]
    im = skimage.io.imread("%s/%s" % (dirname, im_name))
    return im
    
def get_quat(arr):
    return Quat(arr[3], arr[0], arr[1], arr[2]) # ROS to pyquaternion convention

def get_tf_from_pose(pose):
    quat = get_quat(pose[3:])
    transform = quat.transformation_matrix
    transform[0:3, 3] += pose[0:3]
    return transform

if __name__ == '__main__':
    # Get inputs
    dirname="testrun/"
    pose_tool0home_w = get_pose(dirname, "homeSensorSnapshot.yaml")
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
    grasp_pos = np.array([x, y, z])

    # Convert grasp position to world pose
    tf_tool0_camera = np.array([
        [0.36685286, 0.93018527, -0.01320435, -0.03807055],
        [-0.9302708, 0.3668722, -0.00101334, -0.0164097],
        [0.00390172, 0.01265536, 0.99991231, 0.14671274],
        [0.,         0.,         0.,         1.]])

    pose_eegrasp_c = [grasp_pos[0], grasp_pos[1], grasp_pos[2], 0, 0, 0, 1]
    tf_camera_eegrasp = get_tf_from_pose(pose_eegrasp_c)
    tf_eegrasp_tool0 = np.linalg.inv(tf_tool0_camera).dot(tf_camera_eegrasp)

    tf_world_tool0 = get_tf_from_pose(pose_tool0home_w)
    tf_eegrasp_world = tf_eegrasp_tool0.dot(np.linalg.inv(tf_world_tool0))
    print(tf_eegrasp_world)
    # pose_eegrasp_w = []
    print(tf_eegrasp_world[0:3,3])

    # Convert grasp position in overhead camera frame to world frame

    if True:
        poses = [pose_tool0home_w, tf_eegrasp_world[0:3, 3]]
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        x = [p[0] for p in poses]
        y = [p[1] for p in poses]
        z = [p[2] for p in poses]
        ax.scatter3D(x, y, z, s=50)

        ax.set_xlim(-0.5, 0.5)
        # ax.set_ylim(0, 1)
        # ax.set_zlim(0, 1)
        plt.show('3d.png')
