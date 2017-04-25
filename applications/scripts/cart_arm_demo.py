#! /usr/bin/env python

import fetch_api
import rospy

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass



def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    argv = rospy.myargv()
    pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))
    gripper_poses = [pose1, pose2]

    arm = fetch_api.Arm()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    while not rospy.is_shutdown():
        for pose in gripper_poses:
            rospy.sleep(1)
            # Finish building the Pose_stamped message
            # If the message stamp is not current it could be ignored
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            # Set the message pose
            gripper_pose_stamped.pose = pose

            error = arm.move_to_pose(gripper_pose_stamped)
            if error is not None:
                rospy.logerr(error)


if __name__ == '__main__':
    main()
