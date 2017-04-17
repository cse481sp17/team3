#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import (SetTorso, SetTorsoResponse, SetHead, SetHeadResponse, SetArm,
    SetArmResponse, SetGripper, SetGripperResponse)


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._head = fetch_api.Head()
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()

    def handle_set_torso(self, request):
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_head(self, request):
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadResponse()

    def handle_set_arm(self, request):
        self._arm.move_to_joints(fetch_api.ArmJoints.from_list(request.positions))
        return SetArmResponse()

    def handle_set_gripper(self, request):
        if request.todo:
            self._gripper.open()
        else:
            self._gripper.close()
        return SetGripperResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    head_service = rospy.Service('web_teleop/set_head', SetHead,
                                  server.handle_set_head)
    arm_service = rospy.Service('web_teleop/set_arm', SetArm,
                                  server.handle_set_arm)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper,
                                  server.handle_set_gripper)

    rospy.spin()


if __name__ == '__main__':
    main()
