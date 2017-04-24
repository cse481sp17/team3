#! /usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import fetch_api
import copy, math

class DestinationMarker(object):
    def __init__(self, server, x, y, name, driver):
        # ... Initialization, marker creation, etc. ...
        self._server = server
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "base_link"
        self.int_marker.name = name
        self.int_marker.pose.position.x = x
        self.int_marker.pose.position.y = y

        self.box_marker = create_box_marker()
        self.button_control = InteractiveMarkerControl()
        self.button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        self.button_control.always_visible = True
        self.button_control.markers.append(self.box_marker)
        self.int_marker.controls.append(self.button_control)
        self._server.insert(self.int_marker, self._callback)
        self._server.applyChanges()
        self._driver = driver


    def _callback(self, msg):
         interactive_marker = self._server.get(msg.marker_name)
         position = interactive_marker.pose.position
         rospy.loginfo('User clicked {} at {}, {}, {}'.format(msg.marker_name, position.x, position.y, position.z))
         self._driver.goal = position # Updates the Driver's goal.
         print("in callback, updated goal to ", self._driver.goal)


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def create_int_marker():
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "my_marker"
    int_marker.description = "Simple Click Control"
    int_marker.pose.position.x = 1
    int_marker.pose.orientation.w = 1
    return int_marker

def create_box_marker():
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    return box_marker

def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')

class Driver(object):
    def __init__(self, base):
         self.goal = None
         self._base = base
         rospy.sleep(1.)

    def start(self):
        state = 'turn'
        goal = None
        absolute_goal = None
        dx = 0
        dy = 0
        while True:
            if self.goal == None:
                rospy.sleep(0.1)
                continue
            # Check if someone changed self.goal from outside
            if goal != self.goal:
                goal = copy.deepcopy(self.goal)
                absolute_goal = copy.deepcopy(self.goal) # gets overwritten

                absolute_goal.x = self._base.latest_odom.position.x
                absolute_goal.y = self._base.latest_odom.position.y
                theta = self._base.quat_to_yaw(self._base.latest_odom.orientation)
                print("theta is", theta * 180 / math.pi)
                print("original absolute goal is", absolute_goal.x + self._base.latest_odom.position.x, "and", absolute_goal.y + self._base.latest_odom.position.y)
                x_prime = goal.x * math.cos(theta) - goal.y * math.sin(theta)
                y_prime = goal.y * math.cos(theta) + goal.x * math.sin(theta)
                absolute_goal.x += x_prime
                absolute_goal.y += y_prime

                # TODO: restart the turn/move sequence
                state = 'turn'
            dx = absolute_goal.x - self._base.latest_odom.position.x
            dy = absolute_goal.y - self._base.latest_odom.position.y
            angle = math.atan2(dy, dx)
            # radians to turn = goal angle - our angle
            radians_to_turn = angle - self._base.quat_to_yaw(self._base.latest_odom.orientation)
            #print("x", self._base.latest_odom.position.x, "y", self._base.latest_odom.position.y, "facing", self._base.quat_to_yaw(self._base.latest_odom.orientation), "goal", absolute_goal)

            if state == 'turn':
                #print("turning")
                # TODO: Compute how much we need to turn to face the goal
                radians_to_turn = angle - self._base.quat_to_yaw(self._base.latest_odom.orientation)
                #print("to turn", radians_to_turn)
                if abs(radians_to_turn) >= 0.1:
                    self._base.move(0, .5 if radians_to_turn > 0 else -.5)
                else:
                    state = 'move'

            if state == 'move':
                #print("moving")
                if abs(radians_to_turn) >= 0.1:
                    state = 'turn'
                # TODO: Compute how far we have moved and compare that to desired_distance
                desired_distance = math.sqrt(dx**2 + dy**2)
                #print("distance", desired_distance)
                # Make sure that the robot has the ability to drive backwards if it overshoots
                if desired_distance > .2:
                    # TODO: possibly adjust speed to slow down when close to the goal
                    self._base.move(.1, 0)

            rospy.sleep(0.1)

    def check(angle, threshold, start):
        return abs(start - angle) <= threshold


def main():
    rospy.init_node('simple_marker')
    wait_for_time()
    # server = InteractiveMarkerServer("simple_marker")
    # int_marker = create_int_marker()
    # box_marker = create_box_marker()
    # button_control = InteractiveMarkerControl()
    # button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    # button_control.always_visible = True
    # button_control.markers.append(box_marker)
    # int_marker.controls.append(button_control)
    # server.insert(int_marker, handle_viz_input)
    # server.applyChanges()
    # rospy.spin()

    driver = Driver(fetch_api.Base())
    server = InteractiveMarkerServer('simple_marker')
    marker1 = DestinationMarker(server, 2, 2, 'dest1', driver)
    marker2 = DestinationMarker(server, 1, 0, 'dest2', driver)
    marker3 = DestinationMarker(server, 3, -1, 'dest3', driver)
    driver.start()
    rospy.spin()

if __name__ == '__main__':
    main()
