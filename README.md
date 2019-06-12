# MARA ROS


This repository provides ROS 2 support for [MARA](https://acutronicrobotics.com/products/mara/).

## Packages

 - `mara_bringup`: roslaunch scripts for starting the MARA.
 - `mara_description`: 3D models of the MARA for description and visualization.
 - `robotiq_140_gripper_description`: 3D models of the Robotiq 140 gripper for visualization.
 - `robotiq_85_gripper_description`: 3D models of the Robotiq 85 gripper for visualization.
 - `robotiq_hande_gripper_description`: 3D models of the Robotiq Hand-E gripper for visualization.
 - `mara_moveit_config`: Configuration for setting up MoveIT!

## Compile

```bash
mkdir -p ~/ros_mara_ws/src
cd ~/ros_mara_ws/src
git clone https://github.com/AcutronicRobotics/MARA_ROS1 -b crystal
cd ~/ros_mara_ws/
catkin_make_isolated --install
```
