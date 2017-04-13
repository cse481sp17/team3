#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg


ACTION_NAME = 'torso_controller/follow_joint_trajectory'
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
	self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
	self.client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
	if height <= self.MAX_HEIGHT and height >= self.MIN_HEIGHT:
		jtp = trajectory_msgs.msg.JointTrajectoryPoint()
		jtp.positions.append(height)
		jtp.time_from_start = rospy.Duration(TIME_FROM_START)

		goal = control_msgs.msg.FollowJointTrajectoryGoal()
		goal.trajectory.joint_names.append(JOINT_NAME)
		goal.trajectory.points.append(jtp)

		self.client.send_goal(goal)	
		self.client.wait_for_result()












