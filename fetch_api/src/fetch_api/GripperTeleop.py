#!/usr/bin/env python

import copy
import pickle

import rospy
from fetch_api import Arm, Gripper

import std_msgs.msg
import geometry_msgs.msg

import numpy as np
import tf.transformations as tft

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from geometry_msgs.msg import PoseStamped, Pose
GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # gripper_im = InteractiveMarker() ...
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = rospy.Time(0)
        self.markers = createMarkers(ps)
        self.gripper_im_control = InteractiveMarkerControl()
        self.gripper_im_control.always_visible = True
        self.gripper_im_control.interaction_mode = InteractiveMarkerControl.MENU
        self.gripper_im_control.markers = self.markers
        self.gripper_im = InteractiveMarker()
        self.gripper_im.name = "gripper"

        # self.gripper_im.pose.position.x = 0
        # self.gripper_im.pose.position.y = 0

        self.gripper_im.header.frame_id = 'base_link'

        self.gripper_im.controls.append(self.gripper_im_control)
        controls = DOF_Marker_Control()
        self.gripper_im.controls.extend(controls)
        self.gripper_im.scale = 0.420

        #self.markers[2].controls.extend(DOF_Marker_Control())
        menu_items = []
        titles = ["move", "open", "close"]
        for i in range(3):
            menu_item = MenuEntry()
            menu_item.command_type = MenuEntry.FEEDBACK
            menu_item.id = i
            menu_item.title = titles[i]
            menu_items.append(menu_item)
        self.gripper_im.menu_entries = menu_items

        self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        ps = PoseStamped()
        ps.header.frame_id = feedback.header.frame_id
        ps.pose = copy.deepcopy(feedback.pose)

        ps.header.stamp = rospy.Time(0)
        print()
        print(ps)

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 0:
                if self._arm.compute_ik(ps):
                    kwargs = {
                        'allowed_planning_time': 5,
                        'execution_timeout': 5,
                        'num_planning_attempts': 2,
                        'replan': True
                    }
                    # rospy.sleep(1)

                    self._arm.move_to_pose(ps, **kwargs)
            elif feedback.menu_entry_id == 1:
                self._gripper.open()
            elif feedback.menu_entry_id == 2:
                self._gripper.close()

            # self.markers = createMarkers(ps)

            # self.gripper_im_control.markers = self.markers
            # self.gripper_im.pose = feedback.pose

            # self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
            # self._im_server.applyChanges()
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # self.markers = createMarkers(self.gripper_im.pose)
            # self.gripper_im_control.markers = self.markers
            # self._im_server.clear()
            # self.markers = createMarkers(ps.pose)
            # self.gripper_im_control.markers = self.markers

            if self._arm.compute_ik(ps):
                for marker in self.gripper_im_control.markers:
                    marker.color.g = 255
                    marker.color.r = 0
                    marker.color.b = 0
            else:
                for marker in self.gripper_im_control.markers:
                    marker.color.g = 0
                    marker.color.r = 255
                    marker.color.b = 0

            # self.gripper_im.pose = ps.pose
            self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
            self._im_server.applyChanges()
            self._im_server.setPose('gripper', ps.pose)
            self._im_server.applyChanges()

            # self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
            # self._im_server.applyChanges()

class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = rospy.Time(0)

        self.markersgo = createMarkers(ps)

        ps1 = copy.deepcopy(ps)
        ps1.pose.position.x -= 0.1
        self.markerspre = createMarkers(ps1)

        print('hello?')
        ps2 = copy.deepcopy(ps)
        ps2.pose.position.z += 0.25
        self.markersup = createMarkers(ps2)

        self.gripper_im_control = InteractiveMarkerControl()
        self.gripper_im_control.always_visible = True
        self.gripper_im_control.interaction_mode = InteractiveMarkerControl.MENU

        self.gripper_im_control.markers = self.markersgo
        self.gripper_im_control.markers.extend(self.markerspre)
        self.gripper_im_control.markers.extend(self.markersup)

        self.gripper_im = InteractiveMarker()
        self.gripper_im.name = "gripper"

        self.gripper_im.header.frame_id = 'base_link'

        self.gripper_im.controls.append(self.gripper_im_control)

        controls = DOF_Marker_Control()
        self.gripper_im.controls.extend(controls)

        self.gripper_im.scale = 0.420

        menu_items = []
        titles = ["move", "open", "close"]
        for i in range(3):
            menu_item = MenuEntry()
            menu_item.command_type = MenuEntry.FEEDBACK
            menu_item.id = i
            menu_item.title = titles[i]
            menu_items.append(menu_item)
        self.gripper_im.menu_entries = menu_items

        self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        ps = PoseStamped()
        ps.header.frame_id = feedback.header.frame_id
        ps.pose = copy.deepcopy(feedback.pose)

        ps.header.stamp = rospy.Time(0)

        ps1 = copy.deepcopy(ps)
        ps1.pose.position.x -= 0.1

        ps2 = copy.deepcopy(ps)
        ps2.pose.position.z += 0.25

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 0:
                if self._arm.compute_ik(ps) and self._arm.compute_ik(ps1) and self._arm.compute_ik(ps2):
                    self._gripper.open()

                    kwargs = {
                        'allowed_planning_time': 5,
                        'execution_timeout': 5,
                        'num_planning_attempts': 2,
                        'replan': True
                    }


                    ps1 = transform_from_marker(ps, -.1, 0, 0)
                    ps2 = transform_from_marker(ps, 0, 0, 0.25)

                    # ps1.pose = self.markerspre[2].pose

                    self._arm.move_to_pose(ps1, **kwargs)
                    self._arm.move_to_pose(ps, **kwargs)
                    self._gripper.close()
                    rospy.sleep(.5)
                    self._arm.move_to_pose(ps2, **kwargs)
            elif feedback.menu_entry_id == 1:
                self._gripper.open()
            elif feedback.menu_entry_id == 2:
                self._gripper.close()

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            if self._arm.compute_ik(ps):
                for marker in self.markersgo:
                    marker.color.g = 255
                    marker.color.r = 0
                    marker.color.b = 0
            else:
                for marker in self.markersgo:
                    marker.color.g = 0
                    marker.color.r = 255
                    marker.color.b = 0

            if self._arm.compute_ik(ps1):
                for marker in self.markerspre:
                    marker.color.g = 255
                    marker.color.r = 0
                    marker.color.b = 0
            else:
                for marker in self.markerspre:
                    marker.color.g = 0
                    marker.color.r = 255
                    marker.color.b = 0

            if self._arm.compute_ik(ps2):
                for marker in self.markersup:
                    marker.color.g = 255
                    marker.color.r = 0
                    marker.color.b = 0
            else:
                for marker in self.markersup:
                    marker.color.g = 0
                    marker.color.r = 255
                    marker.color.b = 0

            self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
            self._im_server.applyChanges()
            self._im_server.setPose('gripper', ps.pose)
            self._im_server.applyChanges()

def transform_from_marker(ps, x, y, z):
    p = copy.deepcopy(ps)
    ps1 = copy.deepcopy(ps)
    print(p)
    orientation_arr = np.array([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])
    quat_matrix = tft.quaternion_matrix(orientation_arr)
    print(quat_matrix)
    quat_matrix[0][3] = p.pose.position.x
    quat_matrix[1][3] = p.pose.position.y
    quat_matrix[2][3] = p.pose.position.z

    print(quat_matrix)
    objTpg = np.matrix('1 0 0 ' + str(x) + '; 0 1 0 ' + str(y) + '; 0 0 1 ' + str(z) + '; 0 0 0 1')
    blTpg = np.dot(quat_matrix, objTpg)
    print(blTpg)

    print(blTpg.shape)
    ps1.pose.position.x = blTpg[0, 3]
    ps1.pose.position.y = blTpg[1, 3]
    ps1.pose.position.z = blTpg[2, 3]

    return ps1

def create_box_marker():
    box_marker = Marker()
    box_marker.type = Marker.ARROW
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.1
    box_marker.scale.z = 0.1
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    return box_marker

def createMarkers(ps):
    name1 = "left"
    marker1 = Marker()
    # marker1.header.frame_id = "base_link"
    marker1.type = Marker.MESH_RESOURCE
    marker1.mesh_resource = L_FINGER_MESH
    marker1.pose = copy.deepcopy(ps.pose)
    marker1.pose.position.x += 0.166
    marker1.pose.position.y -= 0.05
    # marker1.pose.orientation.w = 1
    marker1.scale.x = 1
    marker1.scale.y = 1
    marker1.scale.z = 1
    marker1.color.r = 0.0
    marker1.color.g = 0.5
    marker1.color.b = 0.5
    marker1.color.a = 1.0
    #marker1.markers.append(create_box_marker())

    name2 = "right"
    marker2 = Marker()
    # marker2.header.frame_id = "base_link"
    marker2.type = Marker.MESH_RESOURCE
    marker2.mesh_resource = R_FINGER_MESH
    marker2.pose = copy.deepcopy(ps.pose)
    marker2.pose.position.x += 0.166
    marker2.pose.position.y += 0.05
    # marker2.pose.orientation.w = 1
    marker2.scale.x = 1
    marker2.scale.y = 1
    marker2.scale.z = 1
    marker2.color.r = 0.0
    marker2.color.g = 0.5
    marker2.color.b = 0.5
    marker2.color.a = 1.0

    name3 = "gripperer"
    marker3 = Marker()
    # marker3.header.frame_id = "base_link"
    marker3.type = Marker.MESH_RESOURCE
    marker3.mesh_resource = GRIPPER_MESH
    marker3.pose = copy.deepcopy(ps.pose)
    marker3.pose.position.x += 0.166
    marker3.pose.orientation.w = 1
    marker3.scale.x = 1
    marker3.scale.y = 1
    marker3.scale.z = 1
    marker3.color.r = 0.0
    marker3.color.g = 0.5
    marker3.color.b = 0.5
    marker3.color.a = 1.0
    return [marker1, marker2, marker3]

def DOF_Marker_Control():
    marker_controls = []
    new_controller_lol = InteractiveMarkerControl()
    new_controller_lol.orientation.w = 1
    new_controller_lol.orientation.x = 1
    new_controller_lol.orientation.y = 0
    new_controller_lol.orientation.z = 0
    new_controller_lol.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    new_controller_lol.name = "move_x"
    new_controller_lol.always_visible = True
    marker_controls.append(new_controller_lol)
    new_controller_lol = copy.deepcopy(new_controller_lol)
    new_controller_lol.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    new_controller_lol.name = "rotate_x"
    marker_controls.append(new_controller_lol)
    new_controller_lol = copy.deepcopy(new_controller_lol)

    new_controller_lol.orientation.w = 1
    new_controller_lol.orientation.x = 0
    new_controller_lol.orientation.y = 1
    new_controller_lol.orientation.z = 0
    new_controller_lol.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    new_controller_lol.name = "move_z"
    marker_controls.append(new_controller_lol)
    new_controller_lol = copy.deepcopy(new_controller_lol)
    new_controller_lol.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    new_controller_lol.name = "rotate_z"
    marker_controls.append(new_controller_lol)
    new_controller_lol = copy.deepcopy(new_controller_lol)

    new_controller_lol.orientation.w = 1
    new_controller_lol.orientation.x = 0
    new_controller_lol.orientation.y = 0
    new_controller_lol.orientation.z = 1
    new_controller_lol.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    new_controller_lol.name = "move_y"
    marker_controls.append(new_controller_lol)
    new_controller_lol = copy.deepcopy(new_controller_lol)
    new_controller_lol.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    new_controller_lol.name = "rotate_y"
    marker_controls.append(new_controller_lol)
    return marker_controls

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('gripper_teleop')
    wait_for_time()
    arm = Arm()
    gripper = Gripper()

    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    print("after server")
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    # auto_pick.start()
    print("before spin")
    rospy.spin()

if __name__ == '__main__':
    main()
