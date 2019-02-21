
cwd=$(pwd)
cd ~/catkin_ws/src/gqcnn/
for file in /media/tweng/drive2/$1/*; do
	fname=$(basename "$file")
	python examples/policy.py --config_filename ~/catkin_ws/src/grasping/config/policy.yaml --image_id $fname --out_dir /media/tweng/drive2/$2
done
cd $cwd
