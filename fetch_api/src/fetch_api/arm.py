# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg

import actionlib

import control_msgs.msg
import trajectory_msgs.msg

import rospy

from .arm_joints import ArmJoints

ACTION_NAME = 'arm_controller/follow_joint_trajectory'

# Time to move the arm
ARM_MOVEMENT_TIME = 5

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
	self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
	self.client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
	jtp = trajectory_msgs.msg.JointTrajectoryPoint()

	jtp.positions.extend(arm_joints.values())
	jtp.time_from_start = rospy.Duration(ARM_MOVEMENT_TIME)

	goal = control_msgs.msg.FollowJointTrajectoryGoal()

	goal.trajectory.joint_names.extend(arm_joints.names())
	goal.trajectory.points.append(jtp)

	self.client.send_goal(goal)
	self.client.wait_for_result()
