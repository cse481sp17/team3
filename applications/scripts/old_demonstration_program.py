#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
import robot_controllers_msgs.msg
# from robot_controllers_msgs import ControllerState

import ar_track_alvar_msgs.msg

import tf

from fetch_api import Arm, Gripper

import rospy

import copy
import actionlib

import numpy as np

class Action(object):
    def __init__(self):
        pass

class GripperAction(Action):
    def __init__(self, openArg, gripper):
        self.open = openArg
        self.gripper = gripper

    def execute(self):
        if self.open:
            self.gripper.open()
        else:
            self.gripper.close()

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

    def execute(self):
        move_to_pose = PoseStamped()
        move_to_pose.header.frame_id = 'base_link'

        if self.absolute:
            print(self.pose.pose.position)
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
            print(move_to_pose)
            self.arm.move_to_pose(move_to_pose)
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
            print('tag pose')
            print(self.former_tag_pose)
            print('former pose')
            print(self.pose)

            # NOTE: Find the tag marker in base link for the new pose
            needed_marker = get_live_marker(self.tagId)
            assert needed_marker

            # NOTE: Find the translation between the old wrist and new wrist offset
            new_pose = None

            # 1: Find the position of the wrist in the tag t1 frame
            whole_thing = copy.deepcopy(self.former_tag_pose)
            former_pose = self.former_tag_pose.pose.pose
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

            print(trans2)
            print(rot2)

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
            print(new_pose)
            # new_pose.pose.position.x -= .166
            # new_pose.pose.position.y -= .024
            self.arm.move_to_pose(new_pose)

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

    def execute(self):
        for action in self.actions:
            print('Executing:', action)
            action.execute()
            rospy.sleep(1)

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
    while len(reader.markers) == 0 and not is_in_marker(reader.markers, tag_id):
        print('trying to find tag')
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
        # print("lol")
        rospy.sleep(0.1)

    # print(reader.markers)

    return copy.deepcopy(reader.markers)

def create_new_program():
    new_program = DemonstrationProgram()
    print('\t add [abs] [tag] | to add a new pose relative to the abs, tag optional')

    print('\t open | to add an open gripper action')
    print('\t close | to add a close gripper action')

    print('\t list | list all actions saved so far')

    print('\t tags | list all tags')

    print('\t done | yay done')
    print('')

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

    gripper = Gripper()
    arm = Arm()

    while True:
        user_input = raw_input()
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
            print('>> Added move action')
        elif user_input == 'open':
            new_program.add_action(GripperAction(True, gripper))
            print('>> Added open action')
        elif user_input == 'close':
            new_program.add_action(GripperAction(False, gripper))
            print('>> Added close action')
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
    print('\t execute [name] | to execute a new program')

    print('\t list | to list all programs')
    print('\t exit | to exit out of this joint')
    print('')

def main():
    rospy.init_node('demonstration_program')
    wait_for_time()

    # TODO(emersonn): This is where we start the interface
    print('Welcome to the best demonstration program in North America')

    programs = {}

    while True:
        try:
            print_main_actions()

            user_input = raw_input()
            if len(user_input.split()) < 1:
                continue

            if (user_input.split()[0] == 'program'):
                new_program = create_new_program()
                programs[user_input.split()[1]] = new_program
            elif (user_input.split()[0] == 'execute'):
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
                print('?????')
                programs[user_input.split()[1]].execute()
            elif (user_input == 'list'):
                print(', '.join(programs))
                print('')
            elif (user_input == 'exit'):
                break
            else:
                print('try again dude')
        except:
            print('DUDE')
            continue

if __name__ == '__main__':
    main()
