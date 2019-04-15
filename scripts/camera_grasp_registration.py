
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
    return transform

def get_pixel_coords(pose):
    K = np.array([
        [618.976990, 0.0, 324.686005],
        [0.0, 618.997620, 234.163773],
        [0.0, 0.0, 1.0]
    ])

    # scale_x = 0.965 / 640 * K[0,0]
    # scale_y = 0.965 / 480 * K[1,1]
    scale_x = 1.27
    scale_y = 1.27

    K[0,0] = K[0,0]/scale_x
    K[1,1] = K[1,1]/scale_y
    res = np.matmul(K, pose[0:3])
    res = res / res[2]
    return res

def get_grasp_pose_camera_frame(tool0_overhead_pose_world_frame, tool0_grasp_pose_world_frame):
    # Adjust tool0_grasp_pose_world_frame for tool0 offset
    tool0_grasp_pose_world_frame[2] -= 0.0226

    world_to_tool0_overhead_tf = get_transform_from_pose(tool0_overhead_pose_world_frame)
    
    camera_to_tool0_tf = np.array([ 
        [0.36685286, 0.93018527, -0.01320435, -0.03807055],
        [-0.9302708, 0.3668722, -0.00101334, -0.0164097],
        [0.00390172, 0.01265536, 0.99991231, 0.14671274],
        [0.,         0.,         0.,         1.]])
    
    world_to_overhead_camera_tf = np.linalg.inv(camera_to_tool0_tf).dot(np.linalg.inv(world_to_tool0_overhead_tf))

    world_to_tool0_grasp_tf = get_transform_from_pose(tool0_grasp_pose_world_frame)
    tool0_grasp_pose_camera_frame = world_to_overhead_camera_tf.dot(world_to_tool0_grasp_tf)

    return tool0_grasp_pose_camera_frame

def get_grasp_pose_theta(overhead_pose, grasp_pose):
    # Rotation in the z axis
    q_overhead = Quat(overhead_pose[3], overhead_pose[0], overhead_pose[1], overhead_pose[2])
    q_grasp = Quat(grasp_pose[3], grasp_pose[0], grasp_pose[1], grasp_pose[2])

    q_diff = q_grasp * q_overhead.inverse

    th_z = q_diff.radians

    print(th_z)
    print(np.rad2deg(th_z))
    return -th_z

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

# Visualize grasp point
def visualize_grasp(name, pixels, theta, grasp_pos):
    im_overhead = get_im(dirname, "homeRGBImage")
    im_approach = get_im(dirname, "approachRGBImage")
    im_grasp = get_im(dirname, "grasp1RGBImage")

    plt.subplot(221)
    plt.imshow(im_overhead)
    plt.scatter(pixels[0], pixels[1], s=5, color='red', marker='.')
    plt.plot([pixels[0]+10*np.cos(theta), pixels[0]+30*np.cos(theta)], [pixels[1]+10*np.sin(theta), pixels[1]+30*np.sin(theta)], color='red')
    plt.plot([pixels[0]-10*np.cos(theta), pixels[0]-30*np.cos(theta)], [pixels[1]-10*np.sin(theta), pixels[1]-30*np.sin(theta)], color='red')
    plt.subplot(222)
    plt.imshow(im_approach)
    plt.subplot(223)
    plt.text(0.1, 0.5, "%.2f, %.2f, %.2f" % (grasp_pos[0], grasp_pos[1], grasp_pos[2]))
    plt.subplot(224)
    plt.imshow(im_grasp)

    plt.savefig("../out/%s.png" % name)
    plt.close()

if __name__ == '__main__':
    for folder in sorted(os.listdir("../data")):
        if "attempt" not in folder:
            continue

        dirname = "../data/" + folder
        overhead_pose = get_pose(dirname, "homeSensorSnapshot.yaml")
        grasp_pose = get_pose(dirname, "grasp1SensorSnapshot.yaml")

        grasp_pose_camera_frame = get_grasp_pose_camera_frame(overhead_pose, grasp_pose)
        pixels = get_pixel_coords(grasp_pose_camera_frame[0:4, 3])
        theta = get_grasp_pose_theta(overhead_pose, grasp_pose)

        visualize_grasp(folder, pixels, theta, grasp_pose[0:3])


