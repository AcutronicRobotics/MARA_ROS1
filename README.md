# MARA ROS


This repository provides ROS 2 support for [MARA](https://acutronicrobotics.com/products/mara/).

![](https://acutronicrobotics.com/docs/user/pages/02.Products/01.MARA/MARA2.jpg)

## Packages

 - `mara_bringup`: roslaunch scripts for starting the MARA.
 - `mara_description`: 3D models of the MARA for description and visualization.
 - `robotiq_arg2f_model_visualization`: 3D models of the Robotiq 140 gripper for visualization.
 - `mara_moveit_config`: Configuration for setting up MoveIT!

## Compile

```bash
mkdir -p ~/ros_mara_ws/src
cd ~/ros_mara_ws/src
git clone https://github.com/AcutronicRobotics/MARA_ROS1
cd ~/ros_mara_ws/
catkin_make_isolated --install
```
