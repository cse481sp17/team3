#!/usr/bin/env python
import fetch_api
import rospy
import pickle
from util import Order, Station, Item
import OrderHandler
import StationInformation
import Navigator
from demonstration_program import *
import threading
import acciobot_main.msg
import std_msgs.msg
#import srv.HandleOrder

# TO RUN:
#   start gazebo
#   roslaunch applications nav_rviz.launch (for now) TODO make our own launch file that starts nav
#   rosrun acciobot_main backend.py

PROGRAM_FILE = '/home/team3/catkin_ws/src/cse481c/acciobot_main/satdemonstration.pickle'

class RobotState(object):
    def __init__(self, order_handler, status_pub):
        self.current_order = None
        self.lock = threading.Lock()
        self.order_handler = order_handler
        self.status_pub = status_pub

    def dispatch_order(self, order_id):
        with self.lock:
            if self.current_order is None:
                self.current_order = order_id

    def finished_order(self):
        with self.lock:
            self.current_order = None
            message = "DONE"
            self.status_pub.publish(std_msgs.msg.String(message))

    def get_current_order(self):
        with self.lock:
            return self.current_order

    def order_callback(self, order_msg):
        rospy.logerr("CallbacK!")
        self.order_handler.add_order(order_msg)
        self.dispatch_order(order_msg.order_id)

    def hello(self):
        rospy.logerr("hello from robot state")

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def load_locations(location_file):
    with open(location_file) as f:
        return pickle.load(f)

def load_stations(station_file):
    with open(station_file) as f:
        return pickle.load(f)

def printHI(response):
    rospy.logerr("ARGH")

def main():
    rospy.logerr("before before lineee")
    rospy.init_node('accio_backend')
    wait_for_time()
    # TODO incorporate navigation
    navigator = Navigator.Navigator()
    station_handler = StationInformation.StationInformation(navigator)
    print("Initializing the gripper, arm, torso, head...")
    print("To quit before these are initialized, do ctrl-C and then ctrl-D.")
    gripper = Gripper()
    print("Gripper ready...")
    arm = Arm()
    print("Arm ready...")
    torso = Torso()
    print("Torso ready...")
    head = Head()
    print("Head ready...")
    base = Base()
    print("Base ready...")
    program_handler = ProgramHandler(PROGRAM_FILE, gripper, arm, torso, head, base)
    #load_program(PROGRAM_FILE, gripper, arm, torso, head) # karan commented this out, refactoring
    status_pub = rospy.Publisher('print_update', std_msgs.msg.String, queue_size=10)
    order_handler = OrderHandler.OrderHandler(station_handler, program_handler, status_pub)
    robot_state = RobotState(order_handler, status_pub)
    rospy.sleep(1)

    #rospy.Subscriber('available_items', acciobot_main.msg.ItemStock, printHI)
    rospy.Subscriber('dispatch_order', acciobot_main.msg.Order, robot_state.order_callback)
    # dispatch_sub = rospy.Subscriber('dispatch_order', acciobot_main.msg.Order, printHI)

    def stop_things():
        navigator.stop()
        arm.cancel_all_goals()
        base.stop()
    rospy.on_shutdown(stop_things)

    rospy.logerr("Waiting for orders")

    while True:
        if robot_state.get_current_order() is not None:
            rospy.logerr("Robot state has a current order")
            order = order_handler.remove_order(robot_state.get_current_order())
            success = order.fulfill_order()
            # TODO: Uncomment when navigation is ready
            print("press enter to continue [removed]")
            # raw_input()
            navigator.move_to_posestamped(station_handler.get_cashier().location)
            robot_state.finished_order()
        rospy.sleep(1)


if __name__ == "__main__":
    main()
