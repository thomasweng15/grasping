
import os
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np 
import skimage
from pyquaternion import Quaternion as Quat
import cv2
np.set_printoptions(suppress=True)

def get_quat(arr):
    return Quat(arr[3], arr[0], arr[1], arr[2]) # ROS to pyquaternion convention

def get_tf_from_pose(pose):
    quat = get_quat(pose[3:])
    transform = quat.transformation_matrix
    transform[0:3, 3] += pose[0:3]
    return transform

def get_pixel_coords(pose):
    K = np.array([
        [618.976990, 0.0, 324.686005],
        [0.0, 618.997620, 234.163773],
        [0.0, 0.0, 1.0]
    ])

    objectPoints = np.array([pose[0:3]])
    imagePoints, _ = cv2.projectPoints(objectPoints, np.array([0., 0., 0.]), np.array([0., 0., 0.]), K, None)
    return imagePoints[0, 0]

def plot_3d(poses, vectors):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    x = [p[0] for p in poses]
    y = [p[1] for p in poses]
    z = [p[2] for p in poses]
    ax.scatter3D(x, y, z, s=50)
    
    for p, v in zip(poses, vectors):
        xline = [p[0], p[0]+0.1*v[0]]
        yline = [p[1], p[1]+0.1*v[1]]
        zline = [p[2], p[2]+0.1*v[2]]
        ax.plot3D(xline, yline, zline)
    
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(0, 1)
    ax.set_zlim(0, 1)
    plt.show('3d.png')

def get_dir_vecs(poses):
    unit_q = Quat(0, 0, 0, 1)
    vecs = []
    for pose in poses:
        quat = get_quat(pose[3:])
        vec = ((quat*unit_q) * quat.conjugate).vector
        vecs.append(vec)

    return vecs

def get_grasp_pose_camera_frame(pose_tool0home_w, pose_tool0grasp_w):
    """
    pose_tool0home_w: 6DOF pose of tool0 overhead in world frame
    pose_tool0grasp_w: 6DOF pose of tool0 grasp in world frame
    """
    # Account for tool0 offset from ee
    ee_offset = -0.226
    pose_eegrasp_w = np.copy(pose_tool0grasp_w)
    pose_eegrasp_w[2] += ee_offset

    pose_eehome_w = np.copy(pose_tool0home_w)
    pose_eehome_w[2] += ee_offset

    # Get world coordinates of camera pose
    tf_tool0_camera = np.array([
        [0.36685286, 0.93018527, -0.01320435, -0.03807055],
        [-0.9302708, 0.3668722, -0.00101334, -0.0164097],
        [0.00390172, 0.01265536, 0.99991231, 0.14671274],
        [0.,         0.,         0.,         1.]])

    tf_world_tool0home = get_tf_from_pose(pose_tool0home_w)
    tf_world_camerahome = tf_world_tool0home.dot(tf_tool0_camera)

    # Get grasp position in camera frame
    tf_world_tool0grasp = get_tf_from_pose(pose_eegrasp_w)
    tf_camerahome_grasp_c = np.linalg.inv(tf_world_camerahome).dot(tf_world_tool0grasp) # in camera frame

    # Visualize
    q_camerahome = Quat(matrix=tf_world_camerahome[:3,:3])
    pose_camerahome_w = list(tf_world_camerahome[0:3, 3]) + list(q_camerahome.vector) + [q_camerahome.real]

    poses = [pose_tool0home_w, pose_tool0grasp_w, pose_eegrasp_w, pose_eehome_w, pose_camerahome_w]
    vecs = get_dir_vecs(poses)
    plot_3d(poses, vecs)

    return tf_camerahome_grasp_c

def get_grasp_pose_theta(overhead_pose, grasp_pose):
    q_overhead = Quat(overhead_pose[6], overhead_pose[3], overhead_pose[4], overhead_pose[5])
    q_grasp = Quat(grasp_pose[6], grasp_pose[3], grasp_pose[4], grasp_pose[5])

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
    plt.text(0.1, 0.5, "%.2f, %.2f, %.2f" % (pixels[0], pixels[1], theta))
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
