#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
import robot_controllers_msgs.msg
# from robot_controllers_msgs import ControllerState
import pickle
import ar_track_alvar_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray

import tf

from fetch_api import Arm, Gripper, ArmJoints, Torso, Head, Base
import tf.transformations as tft
import rospy

import copy
import actionlib

import numpy as np

marker_pub_WTFLOL = rospy.Publisher('/marker_move_to', Marker, queue_size=5)

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

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

def createMarkerLOL(ps, num):
    marker1 = Marker()

    marker1.header.frame_id = "base_link"
    marker1.ns = "demo_program"
    marker1.id = num
    marker1.type = Marker.ARROW

    marker1.pose = copy.deepcopy(ps.pose)
    # marker1.pose.position.x = 1
    # marker1.pose.position.y = 1
    # marker1.pose.position.z = 1

    marker1.scale.x = 0.05
    marker1.scale.y = 0.05
    marker1.scale.z = 0.05
    marker1.color.r = 0.0
    marker1.color.g = 0.5
    marker1.color.b = 0.5
    marker1.color.a = 1.0

    return marker1

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
    # marker3.pose.position.x += 0.166
    marker3.scale.x = 1
    marker3.scale.y = 1
    marker3.scale.z = 1
    marker3.color.r = 0.0
    marker3.color.g = 0.5
    marker3.color.b = 0.5
    marker3.color.a = 1.0
    return [marker1, marker2, marker3]

class Action(object):
    def __init__(self):
        pass
    def __str__(self):
        return type(self).__name__

class HeadAction(Action):
    # Maybe we should pass in explicit args instead
    #   of "args" but it becomes a lot...
    #  format of args is one of following:
    #   look_at [FRAME_ID] [X] [Y] [Z]
    #   pan_tilt [PAN_ANG] [TILT_ANG]
    def __init__(self, args, head):
        self.head = head
        self.args = args

    def execute(self):
        command = self.args[0]
        if command == 'look_at':
            frame_id, x, y, z = self.args[1], float(self.args[2]), float(self.args[3]), float(self.args[4])
            self.head.look_at(frame_id, x, y, z)
        elif command == 'pan_tilt':
            pan, tilt = float(self.args[1]), float(self.args[2])
            self.head.pan_tilt(pan, tilt)
        return True

    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['head']
        return odict

    def __setstate__(self, dict):
        self.__dict__.update(dict)
        self.torso = None

class TorsoAction(Action):
    def __init__(self, height, torso):
        self.height = height
        self.torso = torso

    def execute(self):
        self.torso.set_height(self.height)
        return True

    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['torso']
        return odict

    def __setstate__(self, dict):
        self.__dict__.update(dict)
        self.torso = None

class BaseAction(Action):
    def __init__(self, base, distance):
        self.distance = distance
        self.base = base

    def execute(self):
        self.base.go_forward(self.distance)
        return True

    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['base']
        return odict

    def __setstate__(self, dict):
        self.__dict__.update(dict)
        self.base = None

class GripperAction(Action):
    def __init__(self, openArg, gripper, effort=None):
        self.open = openArg
        self.gripper = gripper
        self.effort = effort

    def execute(self):
        if self.open:
            return self.gripper.open()
        else:
            if self.effort is not None:
                return self.gripper.close(self.effort)
            else:
                return self.gripper.close()

    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['gripper']
        return odict

    def __setstate__(self, dict):
        self.__dict__.update(dict)
        self.effort = 75
        self.gripper = None

class TuckAction(Action):
    def __init__(self, arm):
        self.arm = arm
    def execute(self):
        js = ArmJoints()
        js.set_shoulder_pan(1.32)
        js.set_shoulder_lift(1.4)
        js.set_upperarm_roll(-0.2)
        js.set_elbow_flex(1.72)
        js.set_forearm_roll(0)
        js.set_wrist_flex(1.66)
        js.set_wrist_roll(0)
        values = js.values()
        names = js.names()
        l = []
        for name, val in zip(names, values):
            l.append((name, val))
        result = self.arm.move_to_joint_goal(l)
        if result is None:
            return True
        else:
            print("Tuck failed, result:", result)
            return False
    # really does nothing
    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['arm']
        return odict
    def __setstate__(self, dict):
        self.__dict__.update(dict)
        self.arm = None

class MoveAction(Action):
    def __init__(self, absArg, pose, arm, tagId=None, former_tag_pose=None):
        # True if absolute position ('base_link')
        self.absolute = absArg
        # Pose of the robot's arm
        self.pose = pose

        # Tag ID to look for and the fomer tag pose
        self.tagId = tagId
        self.former_tag_pose = former_tag_pose

        self.arm = arm

    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['arm']
        return odict

    def __setstate__(self, dict):
        self.__dict__.update(dict)
        self.arm = None

    def execute(self, tagID=None, desired_pose=None):
        if tagID is not None:
            self.tagId = tagID
        move_to_pose = PoseStamped()
        move_to_pose.header.frame_id = 'base_link'
        moved = True
        print('wtf guys???', self.absolute)
        if self.absolute:
            #print(self.pose.pose.position)
            move_to_pose.pose.position = Point()
            move_to_pose.pose.position.x = self.pose.pose.position[0]
            move_to_pose.pose.position.y = self.pose.pose.position[1]
            move_to_pose.pose.position.z = self.pose.pose.position[2]

            move_to_pose.pose.orientation = Quaternion()
            move_to_pose.pose.orientation.x = self.pose.pose.orientation[0]
            move_to_pose.pose.orientation.y = self.pose.pose.orientation[1]
            move_to_pose.pose.orientation.z = self.pose.pose.orientation[2]
            move_to_pose.pose.orientation.w = self.pose.pose.orientation[3]

            # move_to_pose.pose.position.x -= .166

            # print(move_to_pose)
            #print(move_to_pose)
            result = self.arm.move_to_pose(move_to_pose)
            if result is not None:
                moved = False
        else:
            # needed_marker = get_live_marker(self.tagId)
            #
            # position_offset = Point()
            # position_offset.x = self.pose.pose.position[0] - self.former_tag_pose.pose.pose.position.x
            # position_offset.y = self.pose.pose.position[1] - self.former_tag_pose.pose.pose.position.y
            # position_offset.z = self.pose.pose.position[2] - self.former_tag_pose.pose.pose.position.z
            #
            # orientation_offset = Quaternion()
            # orientation_offset.x = self.pose.pose.orientation[0] - self.former_tag_pose.pose.pose.orientation.x
            # orientation_offset.y = self.pose.pose.orientation[1] - self.former_tag_pose.pose.pose.orientation.y
            # orientation_offset.z = self.pose.pose.orientation[2] - self.former_tag_pose.pose.pose.orientation.z
            # orientation_offset.w = self.pose.pose.orientation[3] - self.former_tag_pose.pose.pose.orientation.w
            #
            # actual_position = Point()
            # actual_position.x = position_offset.x + needed_marker.pose.pose.position.x
            # actual_position.y = position_offset.y + needed_marker.pose.pose.position.y
            # actual_position.z = position_offset.z + needed_marker.pose.pose.position.z
            #
            # actual_orientation = Quaternion()
            # actual_orientation.x = orientation_offset.x + needed_marker.pose.pose.orientation.x
            # actual_orientation.y = orientation_offset.y + needed_marker.pose.pose.orientation.y
            # actual_orientation.z = orientation_offset.z + needed_marker.pose.pose.orientation.z
            # actual_orientation.w = orientation_offset.w + needed_marker.pose.pose.orientation.w
            #
            # new_pose = PoseStamped()
            # new_pose.header.frame_id = 'base_link'
            # new_pose.pose.position = actual_position
            # new_pose.pose.orientation = actual_orientation
            #
            # new_pose.pose.position.x -= .166
            #
            # self.arm.move_to_pose(new_pose)
            #
            # return
            #print('tag pose')
            #print(self.former_tag_pose)
            #print('former pose')
            #print(self.pose)

            # NOTE: Find the tag marker in base link for the new pose
            needed_marker = None
            if desired_pose is not None:
                print("We got a desired pose instead, moving to it...")
                needed_marker = ar_track_alvar_msgs.msg.AlvarMarker()
                needed_marker.pose.pose = copy.deepcopy(desired_pose)

                # TODO(emersonn): What if it is because the fiducial is saved upside down lol?
                # TODO(emersonn): HARDCODED!!!!!
                # needed_marker.pose.pose.orientation.x = 0.7117
                # needed_marker.pose.pose.orientation.y = -0.007
                # needed_marker.pose.pose.orientation.z = -0.702
                # needed_marker.pose.pose.orientation.w = 0.007
                # needed_marker.pose.pose.orientation.x = self.former_tag_pose.pose.pose.orientation.x
                # needed_marker.pose.pose.orientation.y = self.former_tag_pose.pose.pose.orientation.y
                # needed_marker.pose.pose.orientation.z = self.former_tag_pose.pose.pose.orientation.z
                # needed_marker.pose.pose.orientation.w = self.former_tag_pose.pose.pose.orientation.w

                needed_marker.pose.pose.orientation = self.former_tag_pose.pose.pose.orientation
                quaternion = (
                    needed_marker.pose.pose.orientation.x,
                    needed_marker.pose.pose.orientation.y,
                    needed_marker.pose.pose.orientation.z,
                    needed_marker.pose.pose.orientation.w)
                euler = tft.euler_from_quaternion(quaternion)
                roll = euler[0]
                pitch = euler[1]
                yaw = euler[2]
                print("The yaw in demonst is", yaw, "roll is", roll, "pitch is", pitch)

                # needed_marker.pose.pose.position.z += 0.05

                new_marker = createMarkerLOL(needed_marker.pose, 0)
                # print('New marker:', new_marker)
                marker_pub_WTFLOL.publish(new_marker)

                # print('What if fiducial')
                # WTF_marker = get_live_marker(0)
                # new_marker = createMarkerLOL(WTF_marker.pose, 2)
                # print('Marker for fiducial:', new_marker)
                # marker_pub_WTFLOL.publish(new_marker)
            else:
                print('Working with a fiducial instead')
                needed_marker = get_live_marker(self.tagId)
                assert needed_marker

            #print("Goal pose:", needed_marker.pose.pose)

            # NOTE: Find the translation between the old wrist and new wrist offset
            new_pose = None

            # 1: Find the position of the wrist in the tag t1 frame
            whole_thing = copy.deepcopy(self.former_tag_pose)
            former_pose = copy.deepcopy(self.former_tag_pose.pose.pose)
            whole_thing.pose.pose.position = [former_pose.position.x, former_pose.position.y, former_pose.position.z]
            whole_thing.pose.pose.orientation = [former_pose.orientation.x, former_pose.orientation.y, former_pose.orientation.z, former_pose.orientation.w]
            # print(self.former_tag_pose)
            tag_t1 = pose_to_full_matrix(whole_thing.pose.pose)
            gripper_t1 = pose_to_full_matrix(self.pose.pose)


            former_pose = needed_marker.pose.pose
            needed_marker.pose.pose.position = [former_pose.position.x, former_pose.position.y, former_pose.position.z]
            needed_marker.pose.pose.orientation = [former_pose.orientation.x, former_pose.orientation.y, former_pose.orientation.z, former_pose.orientation.w]
            # print(former_pose)
            # needed_marker.pose.pose.position[0] += 0.1
            tag_t2 = pose_to_full_matrix(needed_marker.pose.pose)

            tag_wrist_t1 = np.dot(np.linalg.pinv(tag_t1), gripper_t1)

            # 2: Find the position of wrist in t1 in tag t2
            # tag_t1_base = np.linalg.pinv(tag_t1)
            # tagt1_tagt2 = np.dot(tag_t1_base, tag_t2)
            # tagt2_tagt1 = np.linalg.pinv(tagt1_tagt2)
            #
            # tagt2_wrist = np.dot(tagt2_tagt1, tag_wrist_t1)

            # 3: Find the position of the wrist in the tag t2 frame
            # base_tagt2 = np.linalg.pinv(tag_t2)
            # base_wristt2 = np.dot(base_tagt2, tagt2_wrist)

            # base_wristt2 = np.dot(tag_t2, tagt2_wrist)
            base_wristt2 = np.dot(tag_t2, tag_wrist_t1)

            # 4: Convert into a pose again
            rot2 = tf.transformations.quaternion_from_matrix(base_wristt2)
            trans2 = tf.transformations.translation_from_matrix(base_wristt2)

            new_pose = PoseStamped()
            new_pose.header.frame_id = 'base_link'

            #print(trans2)
            #print(rot2)

            new_pose.pose.position = Point()
            new_pose.pose.orientation = Quaternion()

            new_pose.pose.position.x = trans2[0]
            new_pose.pose.position.y = trans2[1]
            new_pose.pose.position.z = trans2[2]

            new_pose.pose.orientation.x = rot2[0]
            new_pose.pose.orientation.y = rot2[1]
            new_pose.pose.orientation.z = rot2[2]
            new_pose.pose.orientation.w = rot2[3]

            # print(new_pose)

            # NOTE: Move the arm
            #print(new_pose)
            # new_pose.pose.position.x -= .166
            # new_pose.pose.position.y -= .024

            # for marker in createMarkers(new_pose):
            #     print('Publishing markers')
            #     marker.header.frame_id = "base_link"
            #     marker_pub.publish(marker)
            new_marker = createMarkerLOL(new_pose, 1)
            # print('New marker:', new_marker)
            marker_pub_WTFLOL.publish(new_marker)

            # TODO(emersonn): THIS IS WHERE THINGS GO
            raw_input()

            # print('Trying to move to pose:', new_pose)

            result = self.arm.move_to_pose(new_pose)
            if result is not None:
                moved = False
        return moved

def pose_to_full_matrix(pose):
    trans_mat = tf.transformations.translation_matrix(pose.position)
    rot_mat = tf.transformations.quaternion_matrix(pose.orientation)

    # NOTE: Maybe this is not correct?
    full_mat = np.dot(trans_mat, rot_mat)
    return full_mat

class DemonstrationProgram(object):
    def __init__(self):
        self.actions = []

    def add_action(self, action):
        self.actions.append(action)

    def execute(self, fiducial=None, desired_pose=None):
        for action in self.actions:
            print('Executing:', type(action).__name__)
            worked = False
            if (type(action).__name__ == 'MoveAction'):
                if fiducial is None:
                    worked = action.execute(desired_pose=desired_pose)
                else:
                    worked = action.execute(tagID=fiducial)
            else:
                worked = action.execute()
            if not worked:
                print("The action ", type(action).__name__, "failed")
                return False
            rospy.sleep(1)
        return True

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def get_live_marker(tag_id):
    reader = ArTagReader()
    ar_subscriber = rospy.Subscriber('/ar_pose_marker', ar_track_alvar_msgs.msg.AlvarMarkers, callback=reader.callback)
    while len(reader.markers) == 0 or not is_in_marker(reader.markers, tag_id):
        #print('Trying to find tag:', reader.markers)
        rospy.sleep(0.1)

    needed_marker = None
    for marker in reader.markers:
        if marker.id == tag_id:
            needed_marker = marker
            break

    return needed_marker

def is_in_marker(markers, tag_id):
    for marker in markers:
        if marker.id == tag_id:
            return True
    return False

def get_live_markers():
    reader = ArTagReader()
    ar_subscriber = rospy.Subscriber('/ar_pose_marker', ar_track_alvar_msgs.msg.AlvarMarkers, callback=reader.callback)
    while len(reader.markers) == 0:
        print('trying to find tag')
        rospy.sleep(0.1)

    # print(reader.markers)

    return copy.deepcopy(reader.markers)

def create_new_program(head, torso, gripper, arm, base):
    new_program = DemonstrationProgram()
    print('\t add [abs] [tag] | to add a new pose relative to the abs, tag optional')
    print('\t torso [height] | move torso to height')
    print('\t head look_at [FRAME_ID] [X] [Y] [Z]')
    print('\t head pan_tilt [PAN_ANG] [TILT_ANG]')
    print('\t open | to add an open gripper action')
    print('\t close [effort (35-100)] | to add a close gripper action')
    print('\t move [distance] | move the base forwards or backwards the given distance')
    print('\t tuck | tuck the arm')
    print('\t list | list all actions saved so far')
    print('\t tags | list all tags')
    print('\t done | yay done')
    print('')

    # 5. TODO(emersonn): Make a marker for poses
    # // TODO(emersonn): Make it so if moveit doesn't find anything we extend, and also we need to extend moveit
    # // 1. TODO(emersonn): Reuse point cloud, save AR Marker found right in the beginning of execution
    # // TODO(emersonn): When it finishes, all goals should be meh to Fetch robot
    # // 1. TODO(emersonn): Need to make pipeline run only when given message wait for message
    # 2. TODO(emersonn): Add the cylinders to the planning scene
    # 3. TODO: Add the basket to the planning scene
    # 4. TODO: Add picked up item as a collision object
    # TODO: Move arm back to handy position before segmenting again
    # TODO: Logic in between in util.py for pose based segmentation (kinda, need to handle certain cases, like nothing detected in shelf)
    #   Assume each kind of item is on a certain shelf: code Item to haev that (done, is in station instead)
    #   Pose for the box is in the middle of the box: adjust as necessary. Need to adjust left hand side ones with more logic?? Or all left hand sides??
    # // DONEEEEEEEE Lab 34 for aligned boxes instead of axis aligned. Maybe would fix stuff?
    # TODO: Weird bug with fiducials being upside down need to look into stuff? The hardcoding
    # Max cluster size needs to be changed to like 30,000

    # !
    # _controller_client = actionlib.SimpleActionClient('query_controller_states', robot_controllers_msgs.msg.QueryControllerStatesAction)
    # goal = robot_controllers_msgs.msg.QueryControllerStatesGoal()
    # state = robot_controllers_msgs.msg.ControllerState()
    # state.name = 'arm_controller/follow_joint_trajectory'
    # state.state = robot_controllers_msgs.msg.ControllerState.STOPPED
    # goal.updates.append(state)
    # _controller_client.send_goal(goal)
    # _controller_client.wait_for_result()
    # !

    while True:
        user_input = ""
        try:
            print("please enter a command")
            user_input = raw_input()
        except EOFError:
            print("empty?", user_input)
            continue
        print("you typed", user_input)
        if user_input.split()[0] == 'add':
            # TODO(emersonn): Needs arm pose
            arm_pose = PoseStamped()
            arm_pose.header.frame_id = 'base_link'

            listener = tf.TransformListener()
            rospy.sleep(0.2)

            found_it = False

            rate = rospy.Rate(1.0)
            while not found_it:
                try:
                    # NOTE: gripper_link vs wrist_roll_link
                    (trans, rot) = listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))

                    arm_pose.pose.position = copy.deepcopy(trans)
                    arm_pose.pose.orientation = copy.deepcopy(rot)
                    print(trans)

                    # arm_pose.pose.position = [0.047, 0.545, 1.822]
                    # arm_pose.pose.orientation = [-0.274, -0.701, 0.173, 0.635]

                    found_it = True

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print('wtf error ?????')

                rate.sleep()

            print('FOUND IT!')

            abs_arg = user_input.split()[1] == '1'

            tag_id = None
            former_tag_pose = None

            if not abs_arg:
                assert len(user_input.split()) > 2

                tag_id = int(user_input.split()[2])
                former_tag_pose = copy.deepcopy(get_live_marker(tag_id))
                assert former_tag_pose

            new_action = MoveAction(
                abs_arg,
                arm_pose,
                arm,
                tag_id,
                former_tag_pose,
            )
            new_program.add_action(new_action)
            print('>> Added move arm action')
        elif user_input.split()[0] == 'torso':
            new_program.add_action(TorsoAction(float(user_input.split()[1]), torso))
            print('>> Added torso action')
        elif user_input.split()[0] == 'head':
            rest = user_input.split()[1:]
            command = rest[0]
            print('head command', command, command is not 'look_at', command != 'pan_tilt')
            if command != 'look_at' and command != 'pan_tilt':
                print("wrong command")
                break
            new_program.add_action(HeadAction(rest, head))
            print('>> Added head action')
        elif user_input == 'open':
            new_program.add_action(GripperAction(True, gripper))
            print('>> Added open action')
        elif user_input.split()[0] == 'close':
            effort = int(user_input.split()[1])
            new_program.add_action(GripperAction(False, gripper, effort))
            print('>> Added close action')
        elif user_input.split()[0] == 'move':
            distance = float(user_input.split()[1])
            new_program.add_action(BaseAction(base, distance))
            print('>> Added move base action')
        elif user_input == 'tuck':
            new_program.add_action(TuckAction(arm))
            print('>> Added tuck action')
        elif user_input == 'list':
            for action in new_program.actions:
                print(action)
        elif user_input == 'tags':
            for marker in get_live_markers():
                print(marker.id)
        elif user_input == 'done':
            break

    return new_program

def print_main_actions():
    print('\t program [name] | to make a new program')
    print('\t execute [name] [tagid] | to execute a new program (tagid optional)')

    print('\t list | to list all programs')
    print('\t exit | to exit out of this joint')
    print('')

class ProgramHandler(object):
    def __init__(self, load_file, gripper, arm, torso, head, base):
        # print("Initializing the gripper and arm...")
        # print("To quit before gripper/arm are initialized, do ctrl-C and then ctrl-D.")
        # gripper = Gripper()
        # print("Gripper ready...")
        # arm = Arm()
        # print("Arm ready...")
        # torso = Torso()
        # print("Torso ready...")
        # head = Head()
        # print("Head ready...")
        self.program_info = load_program(load_file, gripper, arm, torso, head, base)

    def get_program(self, program_name):
        return self.program_info[program_name]
    def get_drop(self):
        return self.program_info["drop"] #dropoffnew
    def get_tuck(self):
        return self.program_info["tuck"] #tuckmyarmlol

    def get_shelf(self, shelf):
        if shelf == 2:
            return self.get_program("b4secondshelffar") #b4secondshelf TODO(emersonn): NEED TO MOVE TORSO IN NEW
        if shelf == 3:
            return self.get_program("b4thirdshelf") #b4thirdshelf
        print("you did something wrong to get program")

    def get_raise(self):
    	return self.get_program("raise")

    def get_forward(self):
        return self.get_program("moveforward")

    def get_backward(self):
        return self.get_program("moveback")

def load_program(name, gripper, arm, torso, head, base):
    programs = {}
    try:
        with open(name, "r") as f:
            programs = pickle.load(f)
            #try:
            #    programs = pickle.load(f)
            #except EOFError:
            #    print("error while unpickling")
            for key, program in programs.items():
                for action in program.actions:
                    if type(action).__name__ == 'GripperAction':
                        action.gripper = gripper
                    elif type(action).__name__ == 'MoveAction' or type(action).__name__ == 'TuckAction':
                        action.arm = arm
                    elif type(action).__name__ == 'TorsoAction':
                        action.torso = torso
                    elif type(action).__name__ == 'HeadAction':
                        action.head = head
                    elif type(action).__name__ == 'BaseAction':
                        action.base = base
    except IOError:
        print("error while loading")

    return programs

def main():
    rospy.init_node('demonstration_program')
    wait_for_time()

    print("Initializing the gripper and arm...")
    print("To quit before gripper/arm are initialized, do ctrl-C and then ctrl-D.")
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

    # TODO(emersonn): This is where we start the interface
    print('Welcome to the best demonstration program in North America')
    OK = False
    FILENAME = "/home/team3/catkin_ws/src/cse481c/acciobot_main/satdemonstration.pickle"
    programs = load_program(FILENAME, gripper, arm, torso, head, base)
    OK = True
    with open(FILENAME + "copy", "w") as f:
        pickle.dump(programs, f)

    # For persisting the file to disk
    def pickle_it_up():
        if OK:
            with open(FILENAME, "w") as f:
                pickle.dump(programs, f)

    def stop_things():
        pickle_it_up()
        arm.cancel_all_goals()

    rospy.on_shutdown(stop_things)

    while True:
        print_main_actions()
        user_input = raw_input()
        if len(user_input.split()) < 1:
            continue

        parts = user_input.split()
        command = parts[0]
        if len(parts) < 2 and (command == 'program' or command == 'execute'):
            print("you didn't specify a program name")
            continue

        if (command == 'program'):
            program_name = parts[1]
            if program_name in programs:
                print("that program already exists, are you sure? (y/n)")
                yes = raw_input()
                if not yes.startswith('y') and not yes.startswith('Y'):
                    continue
            new_program = create_new_program(head, torso, gripper, arm, base)
            programs[program_name] = new_program
        elif (command == 'execute'):
            # print('1')
            # _controller_client = actionlib.SimpleActionClient('query_controller_states', robot_controllers_msgs.msg.QueryControllerStatesAction)
            # goal = robot_controllers_msgs.msg.QueryControllerStatesGoal()
            # state = robot_controllers_msgs.msg.ControllerState()
            # state.name = 'arm_controller/follow_joint_trajectory'
            # state.state = robot_controllers_msgs.msg.ControllerState.RUNNING
            # goal.updates.append(state)
            # print('2')
            # _controller_client.send_goal(goal)
            # print('3')
            # _controller_client.wait_for_result()

            # if user_input.split()[1] not in programs:
            #     print('lol')
            #     continue
            program_name = parts[1]
            fid = None
            if (len(parts) > 2):
                fid = int(parts[2])
                # TODO error checking or something
            if program_name in programs:
                result = programs[program_name].execute(fid)
                if not result:
                    print("program", program_name, "failed")
            else:
                print('you typed the program name wrong lol')
        elif (command == 'list'):
            print(', '.join(programs))
            print('')
        elif (command == 'exit'):
            break
        else:
            print('try again dude')

if __name__ == '__main__':
    main()
