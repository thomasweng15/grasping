
import os
import skimage
import numpy as np
import matplotlib.pyplot as plt

def get_crops(rgb_im, depth_im, crop_width=96, stride=5):
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

def get_depth_fname(rgb_fname, files):
    prefix = rgb_fname.split('RGB')[0]
    for f in files:
        if prefix in f and 'Depth' in f:
            return f
    return ''

# Get input images
# TODO also get the label
if __name__ == '__main__':
    input_path = "/home/tweng/fcgqcnn_env/snapshots-thomas-noheight"
    dirs = [f for f in os.listdir(input_path)]
    for d in dirs:
        # Read images 
        rgb_im = skimage.data.imread("%s/%s/color_0.png" % (input_path, d))
        depth_im = np.load("%s/%s/depth_0.npy" % (input_path, d))

        # Crop rgb and depth to 96 x 96 tiles
        rgb_crops, depth_crops = get_crops(rgb_im, depth_im)
            
        # Save 96 x 96 tiles
        # save_crops(rgb_crops, depth_crops)

        break
