
import sys
import os
import numpy as np
import skimage
import matplotlib.pyplot as plt

# Flags
ALIGN_TBL_HEIGHT = True

# Parameters
input_dirname = 'snapshots-thomas' if len(sys.argv) < 2 else sys.argv[1]
output_base_dir = 'snapshots-thomas-converted' if len(sys.argv) < 2 else sys.argv[2] 
sensor = 'primesense'
table_height = 0.703

# Function to create and populate directory
def init_outdir(full_dir):
    if not os.path.isdir(full_dir):
        os.makedirs(full_dir)

        # Copy the sensor-related files into the dir
        os.popen("cp %s/* %s" % (sensor, full_dir))

# Get files
fnames = os.listdir(input_dirname)
for fname in fnames:
    if 'Depth' in fname:
        # Load depth image and transform it
        im = skimage.data.imread("%s/%s"%(input_dirname, fname), as_gray=True)
        im = (im / 1000.0)
        im[im > 0] -= (np.max(im) - table_height) if ALIGN_TBL_HEIGHT else 0

        # Save depth image
        out_dir = fname.split("Depth", 1)[0]
        full_dir = "%s/%s/%s" % (output_base_dir, out_dir, sensor)
        init_outdir(full_dir)
        np.save("%s/depth_0.npy" % full_dir, im)

    if 'RGB' in fname:
        # Copy color image
        out_dir = fname.split("RGB", 1)[0]
        full_dir = "%s/%s/%s" % (output_base_dir, out_dir, sensor)
        init_outdir(full_dir)
        os.popen("cp %s/%s %s/color_0.png" % (input_dirname, fname, full_dir))


