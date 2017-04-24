#!/usr/bin/env python

import fetch_api
import rospy


import sys, select, termios, tty

import nav_msgs.msg

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

msg = """
Control Your Fetch!
---------------------------
Moving around:
        w
   a    s    d

Space: force stop
i/k: increase/decrease only linear speed by 5 cm/s
u/j: increase/decrease only angular speed by 0.25 rads/s
anything else: stop smoothly

CTRL-C to quit
"""

moveBindings = {'w': (1, 0), 'a': (0, 1), 'd': (0, -1), 's': (-1, 0)}

speedBindings = {
    'i': (0.05, 0),
    'k': (-0.05, 0),
    'u': (0, 0.25),
    'j': (0, -0.25),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .2
turn = 1


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('fetch_teleop_key')
    base = fetch_api.Base()

    x = 0
    th = 0
    status = 0
    count = 0
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    marker_publisher = rospy.Publisher('visualization_marker', Marker)
    position = None
    id_counter = 0
    def update_path(data):
        global position, id_counter
        cur_pos = data.pose.pose.position
        if (position == None or abs(cur_pos.x - position.x) > .1 or abs(cur_pos.y - position.y) > .1
            or abs(cur_pos.z - position.z > .1)):
            marker = Marker(
                        type=Marker.SPHERE_LIST,
                        id=id_counter,
                        lifetime=rospy.Duration(),
                        pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                        points=[Point(0, 0, 0)],
                        scale=Vector3(0.06, 0.06, 0.06),
                        header=Header(frame_id='base_link'),
                        color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
            marker_publisher.publish(marker)
            id_counter += 1
            position = cur_pos

    def show_text_in_rviz(text):
        marker = Marker(
                    type=Marker.TEXT_VIEW_FACING,
                    id=0,
                    lifetime=rospy.Duration(1.5),
                    pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.06, 0.06, 0.06),
                    header=Header(frame_id='base_link'),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                    text=text)
        marker_publisher.publish(marker)

    rospy.sleep(1.)
    rospy.Subscriber('/odom', nav_msgs.msg.Odometry, update_path)
    show_text_in_rviz("Why did the chicken cross the road?")
    try:
        print msg
        print vels(speed, turn)
        while (1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed += speedBindings[key][0]
                turn += speedBindings[key][1]
                count = 0

                print vels(speed, turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == ' ':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.02)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.02)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn
            base.move(control_speed, control_turn)
    except e:
        rospy.logerr('{}'.format(e))
    finally:
        base.stop()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
