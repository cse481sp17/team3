#! /usr/bin/env python

import rospy, copy
from geometry_msgs.msg import Twist, Vector3
import nav_msgs.msg
import math
import tf.transformations as tft
class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist)
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)
        self.latest_odom = None
        while self.latest_odom == None:
            rospy.sleep(1.)
        print("got odom it is", self.latest_odom, "with x position", self.latest_odom.position.x)

    def _odom_callback(self, msg):
        # TODO: do something
        self.latest_odom = msg.pose.pose


    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while self.latest_odom == None:
            rospy.sleep(1.)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.latest_odom)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance

        # TODO: Be sure to handle the case where the distance is negative!
        while abs(self.latest_odom.position.x - start.position.x) < distance:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()

    def quat_to_yaw(self, q):
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = m[0, 0]
        y = m[1, 0]
        theta_rads = math.atan2(y, x)
        return theta_rads

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while self.latest_odom == None:
            rospy.sleep(1.)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.latest_odom)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?


        if abs(angular_distance) > (2 * math.pi):
            angular_distance = angular_distance % (2 * math.pi)

        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!

# angular distance -> pi
# current orientation -> z

# 0.5
#
        start = self.quat_to_yaw(start.orientation) % (2 * math.pi)
        finish = (start + angular_distance) % (2 * math.pi)
        print("finish", finish * 180 / math.pi)
        amount_left = 2 * math.pi
        while True: #
            latest = self.quat_to_yaw(self.latest_odom.orientation) % (2 * math.pi)
            amount = None
            if angular_distance > 0:
                amount = (finish - latest) % (2 * math.pi)
            else:
                amount = (latest - finish) % (2 * math.pi)
            if (amount > amount_left):
                break
            amount_left = amount

            print("started orientation", start * 180 / math.pi)
            print("latest", latest * 180 / math.pi)
            print(amount_left * 180 / math.pi)

            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        twist_msg = Twist(Vector3(linear_speed, 0, 0), Vector3(0, 0, angular_speed))
        self.pub.publish(twist_msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        self.move(0,0)
