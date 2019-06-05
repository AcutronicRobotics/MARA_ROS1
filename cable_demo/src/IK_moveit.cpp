#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

#define M_PI 3.14159265358979323846

boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
boost::shared_ptr<tf::TransformListener> tf_listener;

void move_home()
{
  // Set home
  std::vector<double> target_joint{-M_PI/2, -M_PI/6, M_PI/6, 0., M_PI/6, 0.};
  move_group->setJointValueTarget(target_joint);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (not success){
      ROS_ERROR_NAMED("IK_moveit", "Plan to home failed");
      return;
  }
  ROS_INFO_NAMED("IK_moveit", "Plan succeed! Moving home");

  // Execute
  success = false;
  int it = 0;
  while (success != true && it < 5){
    success = (move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (not success) {ROS_ERROR_NAMED("IK_moveit", "Execution failed");}
    it++;
  }
  ROS_INFO_NAMED("IK_moveit", "Executed successfully");
}


void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_goal_C)
{
  ROS_INFO_NAMED("IK_moveit", "Received pose!");

  // tf::StampedTransform transform;
  // try{
  //   ros::Time now = ros::Time::now();
  //   tf_listener.waitForTransform("base_link", "ee_link", now, ros::Duration(3.0));
  //   tf_listener.lookupTransform("base_link", "ee_link", now, transform);
  // }
  // catch (tf::TransformException ex){
  //   ROS_ERROR("%s", ex.what());
  //   ros::Duration(1.0).sleep();
  // }

  geometry_msgs::PoseStamped init_pose;
  init_pose.header.stamp = pose_goal_C->header.stamp;
  init_pose.header.frame_id = "camera_color"; // Hard-coded. Original was rs_camera_frame, but it's unreachable
  init_pose.pose.position.x = pose_goal_C->pose.position.x;
  init_pose.pose.position.y = pose_goal_C->pose.position.y;
  init_pose.pose.position.z = pose_goal_C->pose.position.z;
  init_pose.pose.orientation.x = pose_goal_C->pose.orientation.x;
  init_pose.pose.orientation.y = pose_goal_C->pose.orientation.y;
  init_pose.pose.orientation.z = pose_goal_C->pose.orientation.z;
  init_pose.pose.orientation.w = pose_goal_C->pose.orientation.w;

  geometry_msgs::PoseStamped ori_pose;
  try{
    ros::Time now = ros::Time::now();
    tf_listener->waitForTransform("base_link", "ee_link", now, ros::Duration(3.0));
    tf_listener->transformPose("base_link", move_group->getCurrentPose(), ori_pose);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::PoseStamped target_pose;
  try{
    ROS_INFO_NAMED("IK_moveit", "Searching transformation...");
    ros::Time now = ros::Time::now();
    tf_listener->waitForTransform("base_link", init_pose.header.frame_id, now, ros::Duration(3.0));
    tf_listener->transformPose("base_link", init_pose, target_pose);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  ROS_INFO_NAMED("IK_moveit", "Transforation fetched!");


  target_pose.pose.position.z = ori_pose.pose.position.z;
  target_pose.pose.orientation.x = ori_pose.pose.orientation.x;
  target_pose.pose.orientation.y = ori_pose.pose.orientation.y;
  target_pose.pose.orientation.z = ori_pose.pose.orientation.z;
  target_pose.pose.orientation.w = ori_pose.pose.orientation.w;

  move_group->setPoseTarget(target_pose.pose);

  // geometry_msgs::Pose target_pose;
  // target_pose.position.x = transform.getOrigin().x() + 0.005;
  // target_pose.position.y = transform.getOrigin().y();
  // target_pose.position.z = transform.getOrigin().z();
  // target_pose.orientation.x = transform.getRotation().x();
  // target_pose.orientation.y = transform.getRotation().y();
  // target_pose.orientation.z = transform.getRotation().z();
  // target_pose.orientation.w = transform.getRotation().w();
  //
  // move_group->setPoseTarget(target_pose);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (not success){
      ROS_ERROR_NAMED("IK_moveit", "Plan failed");
      return;
  }
  ROS_INFO_NAMED("IK_moveit", "Plan succeed! Moving...");

  // Execute
  // Execute
  success = false;
  int it = 0;
  while (success != true && it < 5){
    success = (move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (not success) {ROS_ERROR_NAMED("IK_moveit", "Execution failed");}
    it++;
  }
  ROS_INFO_NAMED("IK_moveit", "Executed successfully");

  // // Wait 5 seconds
  // ros::Duration(5.0).sleep();
  //
  // // Go back home
  // move_home();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "IK_moveit");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/mara/pred_target", 1000, pose_callback);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initalize global variables
  move_group.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));
  tf_listener.reset(new tf::TransformListener());

  // Start going to home position
  move_home();

  // Infinite loop
  ros::waitForShutdown();

  return 0;
}
