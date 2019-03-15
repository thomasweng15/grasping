`python show.py [PATH TO IMAGE]` Displays a depth image the way gqcnn displays it

`python convert_depth_format.py [INPUT PATH] [OUTPUT PATH]` takes all depth images at the input path and converts them into .npy. It copies all rgb and converted depth images to the new output directory

`bash run_gqcnn.sh` Runs gqcnn on all images, you can specify which path to look at, probably you want OUTPUT PATH 
