
import matplotlib.pyplot as plt
import numpy as np 
np.set_printoptions(suppress=True)

###
## Go from grasp point to camera point
###

# Initial transform from inertial frame to ee
world_ee_init_tf = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 1], # 1m overhead
    [0, 0, 0, 1]
])

# Transform at grasp time from inertial frame to ee
world_ee_grasp_tf = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# Transform from pose to camera origin
ee_camera_tf = np.array([
    [0.3679918,  0.9298182, -0.0045070, -0.038245],
    [-0.9298133,  0.3679529, -0.0076098, -0.038245],
    [-0.0054174,  0.0069910,  0.9999609, 0.147432],
    [0, 0, 0, 1]
])

# Camera intrinsic matrix
K = np.array([
    [618.976990, 0.0, 324.686005, 0],
    [0.0, 618.997620, 234.163773, 0],
    [0.0, 0.0, 1.0, 0]])

# Transform into camera frame
ee_camera_init_tf = np.matmul(world_ee_init_tf, ee_camera_tf) # TODO double check the directionality

# Get the camera matrix
# http://ksimek.github.io/2013/08/13/intrinsic/
camera_mat = np.matmul(K, ee_camera_init_tf)

# Get 3D world point we want to find in 2D pixel coords
# TODO adjust for offset of ee from grasp point
grasp_pt = world_ee_grasp_tf[0:3, 3]
hm_grasp_pt = np.concatenate((grasp_pt, np.array([1])))

# Get 2D pixel coords
hm_grasp_px = np.matmul(camera_mat, hm_grasp_pt)
grasp_px = np.rint((hm_grasp_px / hm_grasp_px[2])[0:2])

print(grasp_px)
