#!/usr/bin/env python
import actionlib
import move_base_msgs.msg
class Navigator(object):
    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        self.move_base_client.wait_for_server()

    # Moves to given pose. Blocks until we get there.
    def move_to_posestamped(self, pose_stamped):
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose = pose_stamped
        print("Sending navigation goal:", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        self.move_base_client.send_goal_and_wait(goal)
        print("Reached navigation goal")

    def stop(self):
        self.move_base_client.cancel_goal()
