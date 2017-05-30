#! /usr/bin/env python

import rospy
import actionlib
import control_msgs.msg

ACTION_NAME = 'gripper_controller/gripper_action'
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.GripperCommandAction)
        self.client.wait_for_server()

    def open(self):
        """Opens the gripper.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        goal.command.max_effort = self.MAX_EFFORT
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return True # todo figure out how to know if it didn't work

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return True # todo figure out how to know if it didn't work
