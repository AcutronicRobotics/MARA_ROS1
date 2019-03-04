cmake_minimum_required(VERSION 2.8.3)

project(hrim_actuator_gripper_msgs)

find_package(catkin REQUIRED message_generation std_msgs)

add_message_files(DIRECTORY msg FILES
  "CycleTime.msg"
  "StateGripper.msg"
  "StateFingerGripper.msg"
  "GoalTactilePressure.msg"
  "SpecsFingerGripper.msg"
  "SpecsMagneticGripper.msg"
  "SpecsVacuumGripper.msg"
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
