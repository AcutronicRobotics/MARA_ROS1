#! /usr/bin/env python

import rospy
import time

import actionlib

import math
import copy

import control_msgs.msg
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class maraFollowJoint(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def jointStateCallback(self, msg):
        self.joints_state_msg = msg

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.state_subscriber = rospy.Subscriber("/mara_controller/state", JointTrajectoryControllerState, self.jointStateCallback)

        names_joints = ["/hros_actuation_servomotor_0000000000012/trajectory",
                        "/hros_actuation_servomotor_000000000001/trajectory",
                        "/hros_actuation_servomotor_000000000002/trajectory",
                        "/hros_actuation_servomotor_0000000000022/trajectory",
                        "/hros_actuation_servomotor_000000000003/trajectory",
                        "/hros_actuation_servomotor_0000000000032/trajectory"]

        self.publishers_joint = []

        for i in range(len(names_joints)):
            self.publishers_joint.append(rospy.Publisher(names_joints[i], JointTrajectory, queue_size=1))

    def execute_cb(self, goal):

        success = True

        print goal
        number_of_joints = 6
        number_of_points_to_follow = len(goal.trajectory.points)

        msg = []
        for joint_number in range(number_of_joints):
            msg.append(JointTrajectory())

        for t in range(number_of_points_to_follow):
            for joint_number in range(number_of_joints):
               p = JointTrajectoryPoint()
               p.positions.append(goal.trajectory.points[t].positions[joint_number])
               p.velocities.append(goal.trajectory.points[t].velocities[joint_number])
               p.accelerations.append(goal.trajectory.points[t].accelerations[joint_number])
               p.time_from_start.secs = goal.trajectory.points[t].time_from_start.secs;
               p.time_from_start.nsecs = goal.trajectory.points[t].time_from_start.nsecs;
               msg[joint_number].points.append(p)

        for joint_number in range(number_of_joints):
            print("**************************************** joint_number ", joint_number)
            print(msg[joint_number]);
            self.publishers_joint[joint_number].publish(msg[joint_number])

        time.sleep(goal.trajectory.points[number_of_points_to_follow-1].time_from_start.secs
                    + goal.trajectory.points[number_of_points_to_follow-1].time_from_start.nsecs/1e+9);

        if success:
            # self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('mara_follow_joint_trajectory')
    server = maraFollowJoint('/follow_joint_trajectory')
    rospy.spin()
