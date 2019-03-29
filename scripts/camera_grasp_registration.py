
import os
import matplotlib.pyplot as plt
import numpy as np 
import skimage
from pyquaternion import Quaternion as Quat
np.set_printoptions(suppress=True)

def get_transform_from_pose(pose):
    quat = Quat(pose[6], pose[3], pose[4], pose[5]) # Convert from ROS to pyquaternion convention
    transform = quat.transformation_matrix
    transform[0:3, 3] += pose[0:3]
    return np.linalg.inv(transform)

def get_px_coords(pose):
    K = np.array([
        [618.976990, 0.0, 324.686005],
        [0.0, 618.997620, 234.163773],
        [0.0, 0.0, 1.0]
    ])

    scale = 2.54

    # u = K[0,0]/2.54*pose[0] / pose[2] + K[0,2]
    # v = K[1,1]/2.54*pose[1] / pose[2] + K[1,2]
    # return (u, v)

    K[0,0] = K[0,0]/scale
    K[1,1] = K[1,1]/scale
    res = np.matmul(K, pose[0:3])
    res = res / res[2]
    return res

def get_grasp_pose_pixels(tool0_grasp_pose_world_frame):
    # Adjust tool0_grasp_pose_world_frame for tool0 offset
    tool0_grasp_pose_world_frame[2] -= .0226

    tool0_overhead_pose_world_frame = [0.07, 0.818, 0.6, -0.831469612676492, 0.5555702324599514, 0.0, 0.0]
    world_tool0_transform = get_transform_from_pose(tool0_overhead_pose_world_frame)
    
    camera_tool0_transform = np.array([ 
        [0.36685286, 0.93018527, -0.01320435, -0.03807055],
        [-0.9302708, 0.3668722, -0.00101334, -0.0164097],
        [0.00390172, 0.01265536, 0.99991231, 0.14671274],
        [0.,         0.,         0.,         1.]])
    
    world_camera_transform = np.matmul(np.linalg.inv(camera_tool0_transform), world_tool0_transform)
    # world_camera_transform = np.matmul(camera_tool0_transform, world_tool0_transform)
    # print("tool in world", tool0_grasp_pose_world_frame)
    # print("camera in world tf", world_camera_transform.dot(np.array([0, 0, 0, 1])))
    tool0_grasp_position_camera_frame = world_camera_transform.dot(np.hstack((tool0_grasp_pose_world_frame[0:3], 1)))
    # print("tool pos in camera", tool0_grasp_position_camera_frame)
    # tool0_grasp_position_camera_frame = np.array([0., 0.12, 0.2])
    
    # tool0_grasp_position_tool0_frame = np.matmul(world_tool0_transform, np.hstack((tool0_grasp_pose_world_frame[0:3], 1)))
    # tool0_grasp_position_camera_frame = np.linalg.inv(camera_tool0_transform).dot(tooldiff)
    # tool0_grasp_position_camera_frame = camera_tool0_transform.dot(tool0_grasp_position_tool0_frame)
    # print("tool grasp position tool frame tf", tool0_grasp_position_tool0_frame)

    pixels = get_px_coords(tool0_grasp_position_camera_frame)
    return pixels

def get_pose_and_rgb_im(dirname):
    files = os.listdir(dirname)

    IKPlans = sorted([f for f in files if "IKPlan.yaml" in f])
    IKPlan = IKPlans[-1]

    with open("%s/%s" % (dirname, IKPlan), "r") as f:
        lines = [line.rstrip('\n') for line in f]
        for i, line in enumerate(lines):
            if "end_pose_world_frame" in line:
                pose_str = line + lines[i+1]
                pose = [float(x) for x in pose_str.split('[')[1].split(']')[0].split(',')]
                break
    
    # Get RGB image
    RGBImages = sorted([f for f in files if "RGBImage" in f], reverse=True)
    RGBImage = RGBImages[-2]
    rgb_im = skimage.io.imread("%s/%s" % (dirname, RGBImage))

    return pose, rgb_im

# Visualize grasp point
def visualize_grasp(name, im, px, py):
    plt.imshow(im)
    plt.scatter(px, py, color='red', marker='o')
    plt.savefig("%s.png" % name)
    plt.close()

dirs = [
    "attempt-2019-03-22-16-16-08",
    "attempt-2019-03-22-16-16-24",
    "attempt-2019-03-22-16-16-40",
    "attempt-2019-03-22-16-16-58",
    "attempt-2019-03-22-16-17-15"
]

for dirname in dirs:
    grasp_pose, rgb_im = get_pose_and_rgb_im("../data/" + dirname)
    pixels = get_grasp_pose_pixels(grasp_pose)
    print(pixels)
    visualize_grasp(dirname, rgb_im, pixels[0], pixels[1])


