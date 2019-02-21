# grasping

## Installation
1. Install ROS
2. Install dependencies
    * Moveit: `sudo apt install ros-kinetic-moveit`
    * gqcnn
3. Create workspace and add thing2.sh to workspace root
4. Add sawyer_moveit, gqcnn, and grasping source
5. Copy `models/` to `~/.gazebo/models`

## Simulation

0. Setup
    * Source the simulated robot: `bash ~/catkin_ws/thing2.sh sim`
1. Launch the sawyer world: `roslaunch grasping sawyer_world.launch`
2. Launch moveit and rviz: `roslaunch grasping moveit_rviz.launch`
3. Save perception images: `~/catkin_ws/src/perception/tools/capture_images.py --config_filename ~/catkin_ws/src/grasping/config/capture_images.yaml NAME`
4. Run gqcnn on saved images: `python examples/policy.py --config_filename /usr0/home/tweng/catkin_ws/src/grasping/config/policy.yaml`



