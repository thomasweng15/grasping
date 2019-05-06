import os
import numpy as np 
from pyquaternion import Quaternion as Quat
import cv2
import skimage
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