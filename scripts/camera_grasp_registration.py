
import matplotlib.pyplot as plt
import numpy as np 
np.set_printoptions(suppress=True)

# Camera intrinsic matrix
K = np.array([
    [618.976990, 0.0, 324.686005, 0],
    [0.0, 618.997620, 234.163773, 0],
    [0.0, 0.0, 1.0, 0]])

### Test transform from 3D point in camera frame to 2D intrinsic matrix.
def get_px_from_camera_pt(K, p_hm)
    # Multiply with K to get homogeneous pixel
    px_hm = np.matmul(K, p_hm.T)

    # Perspective division to set w back to 1
    px_hm = px_hm / px_hm[2]

    # Round and extract pixel coordinates
    px, py, _ = np.rint(px_hm)
    return px, py

# Homogeneous 3D point in camera frame
p_hm = np.array([0.1, 0.1, 0.4, 1])

px, py = get_px_from_camera_pt(K, p_hm)


### Get real 3D point from snapshots-thomas/2019-02-13-18-53-17SensorSnapshot.yaml
# Joint angles from snapshots-thomas/2019-02-13-18-53-17SensorSnapshot.yaml
joint_angles = np.array([-1.450477425252096, -1.871162239705221, -0.8121445814715784, -2.029160801564352, 1.570809483528137, 1.298530101776123])

# Compute transform from world to tool frame with forward kinematics
def fk(joint_angles):
    # dummy function
    return np.array([
        [1 0 0 0.0], 
        [0 0 0 0.0], 
        [0 0 0 0.0],
        [0 0 0 1]
    ])
g_wt_o = fk(joint_angles_overhead)
g_wt_g = fk(joint_angles_grasp)

# Get transform from world to camera frame
g_wc_overhead = g_wt_overhead*g_tc

# # get real-world end effector position
# p_ee = g_wt_g[:, 1:3]

def get_tool_camera_tf():
    xyz = np.array([-0.038245, -0.016097, 0.147432])

    # Generated rotation matrix from rpy
    R = np.array([
        [0.3679918,  0.9298182, -0.0045070],
        [-0.9298133,  0.3679529, -0.0076098],
        [-0.0054174,  0.0069910,  0.9999609]
    ])
    return np.vstack((np.column_stack((R, xyz)), np.array([0, 0, 0, 1])))

# Transform of tool frame to camera frame
g_tc = get_tool_camera_tf()

# Transform of camera frame to tool frame
g_ct = np.linalg.inv(g_tc)

print(g_tc)
print(g_ct)
