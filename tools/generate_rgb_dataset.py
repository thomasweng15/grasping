
import skimage
import numpy as np
import matplotlib.pyplot as plt

def get_crops(rgb_im, depth_im, crop_width=96, stride=1):
    rgb_crops = []
    depth_crops = []
    height, width, _ = rgb_im.shape
    for i in xrange(0, height, stride):
        for j in range(0, width, stride):
            if i + crop_width > height or j + crop_width > width:
                continue

            rgb_crop = rgb_im[i:i+crop_width, j:j+crop_width]
            depth_crop = depth_im[i:i+crop_width, j:j+crop_width]

            rgb_crops.append(rgb_crop)
            depth_crops.append(depth_crop)

    return rgb_crops, depth_crops

def save_crops(rgb_crops, depth_crops, save_path="/home/tweng/fcgqcnn_env/snapshots-thomas-tiles"):
    for i in range(len(rgb_crops)):
        np.savez("%s/tf_rgb_ims_%s.npz" % (save_path, str(i).zfill(5)), rgb_crops[i])
        np.savez("%s/tf_depth_ims_%s.npz" % (save_path, str(i).zfill(5)), depth_crops[i])

def visualize(rgb_im, depth_im):
    plt.subplot(2, 1, 1)
    plt.imshow(rgb_im)
    plt.subplot(2, 1, 2)
    plt.imshow(depth_im, cmap=plt.cm.gray_r)
    plt.show()

# Get input images 
# TODO iterate over all images in directory
input_dirname = "/home/tweng/fcgqcnn_env/snapshots-thomas"
rgb_fname = "2019-02-13-18-53-1712345RGBImage.png"
rgb_im = skimage.data.imread("%s/%s"%(input_dirname, rgb_fname))
depth_fname = "2019-02-13-18-53-1712345DepthImage.png"
depth_im = skimage.data.imread("%s/%s"%(input_dirname, depth_fname), as_gray=True)

# Crop rgb and depth to 96 x 96 tiles
rgb_crops, depth_crops = get_crops(rgb_im, depth_im)

# Save 96 x 96 tiles
# save_crops(rgb_crops, depth_crops)


