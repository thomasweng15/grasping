
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

if __name__ == '__main__':
    # Get inputs
    dirname="testrun/"
    overhead_pose = get_pose(dirname, "homeSensorSnapshot.yaml")
    px_coords, theta, im_overhead, _ = np.load(dirname + "params.npy")
    px_coords = np.rint(px_coords).astype(int)
    px_x, px_y = px_coords

    # Get overhead tool0 pose
    

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
    grasp_position = np.array([x, y, z])


    # Convert grasp position in overhead camera frame to world frame
