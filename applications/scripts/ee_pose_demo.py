#! /usr/bin/env python

import fetch_api
import rospy
import tf

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('pose_demo')
    wait_for_time()

    listener = tf.TransformListener()
    rospy.sleep(0.1)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('gripper_link', 'base_link', rospy.Time(0))
            print(trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            #print(e)
            continue

        rate.sleep()

if __name__ == '__main__':
    main()
