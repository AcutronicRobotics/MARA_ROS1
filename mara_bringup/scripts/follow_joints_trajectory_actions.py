#! /usr/bin/env python

import rospy
import time
import sys

import actionlib
from actionlib_msgs.msg import GoalStatus, GoalID

import math
import copy
import yaml

import control_msgs.msg
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rospy.rostime import Duration

class maraFollowJoint(object):
    # create messages that are used to publish feedback/result
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name, yml_file_str):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)


        rospy.Subscriber("/follow_joint_trajectory/cancel", GoalID, self.cancel_cb)

        self._as.start()

        with open(yml_file_str, 'r') as ymlfile:
            cfg = yaml.load(ymlfile)

        self.names_actions = cfg['motors']

        self.actions_list = []

        for i in range(len(self.names_actions)):
            self.actions_list.append(actionlib.SimpleActionClient(self.names_actions[i], FollowJointTrajectoryAction))

    def feedback_cb(self, feedback):
        self.as_.publishFeedback(feedback);

    def cancel_cb(self, cancel):
        print("Cancel recieved!");
        print(cancel)
        self._as.set_aborted()
        for i in range(len(self.names_actions)):
            pub = rospy.Publisher(self.names_actions[i] + "/cancel", GoalID, queue_size=1)
            pub.publish(cancel)

    def execute_cb(self, goal):

        # Waits until the action server has started up and started
        # listening for goals.
        number_of_joints = len(self.names_actions)
        number_of_points_to_follow = len(goal.trajectory.points)
        for i in range(len(self.names_actions)):
            print(self.names_actions[i])
            while not self.actions_list[i].wait_for_server(Duration(1.0)):
                print("Waiting for" + self.names_actions[i] + "service")


        msg_actions = []
        for joint_number in range(number_of_joints):
            msg_actions.append(FollowJointTrajectoryGoal())


        for t in range(number_of_points_to_follow):
            for joint_number in range(number_of_joints):
               p = JointTrajectoryPoint()
               p.positions.append(goal.trajectory.points[t].positions[joint_number])
               p.velocities.append(goal.trajectory.points[t].velocities[joint_number])
               p.accelerations.append(goal.trajectory.points[t].accelerations[joint_number])
               p.time_from_start.secs = goal.trajectory.points[t].time_from_start.secs;
               p.time_from_start.nsecs = goal.trajectory.points[t].time_from_start.nsecs;
               msg_actions[joint_number].trajectory.points.append(p)

        for joint_number in range(number_of_joints):
            # client.send_goal(goal_client, feedback_cb=self.feedback_cb)
            self.actions_list[joint_number].send_goal(msg_actions[joint_number])


        time_to_wait = (goal.trajectory.points[number_of_points_to_follow-1].time_from_start.secs
                      + goal.trajectory.points[number_of_points_to_follow-1].time_from_start.nsecs/1e+9) + 2;

        # Waits for the server  to finish performing the action
        for joint_number in range(number_of_joints):
            # client.send_goal(goal_client, feedback_cb=self.feedback_cb)
            self.actions_list[joint_number].wait_for_result(Duration(time_to_wait))

        success = True

        for joint_number in range(number_of_joints):
            if self.actions_list[joint_number].get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('%s: Succeeded' % self._action_name)
                if self._as.is_active():
                    print("Motor " + self.names_actions[joint_number] + " finished well")
                else:
                    success = False;
            else:
                print("Motor " + self.names_actions[joint_number] + " failed")
                success = False;

        if(success):
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted(text="JointTrajectory action did not succeed");

if __name__ == '__main__':

    if(len(sys.argv)<2):
        print("usage: follow_joints_trajectory_actions.py yml_file_str\n")
        exit(0)

    rospy.init_node('mara_follow_joint_trajectory')
    server = maraFollowJoint('/follow_joint_trajectory', sys.argv[1])
    rospy.spin()
