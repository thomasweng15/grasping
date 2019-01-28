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
2. Launch moveit: `roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true rviz_config:=$(rospack find grasping)/config/rviz.rviz`
3. Launch camera: ``


