#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
import robot_controllers_msgs.msg
# from robot_controllers_msgs import ControllerState
import pickle
import ar_track_alvar_msgs.msg

import tf

from fetch_api import Arm, Gripper, ArmJoints

import rospy

import copy
import actionlib

import numpy as np

class Action(object):
    def __init__(self):
        pass
    def __str__(self):
        return type(self).__name__

class GripperAction(Action):
    def __init__(self, openArg, gripper):
        self.open = openArg
        self.gripper = gripper

    def execute(self):
        if self.open:
            self.gripper.open()
        else:
            self.gripper.close()

    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['gripper']
        return odict

    def __setstate__(self, dict):
        self.__dict__.update(dict)
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
        self.arm.move_to_joints(js)
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

    def execute(self, tagID=None):
        if tagID is not None:
            self.tagID = tagID
        move_to_pose = PoseStamped()
        move_to_pose.header.frame_id = 'base_link'

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
            #print('tag pose')
            #print(self.former_tag_pose)
            #print('former pose')
            #print(self.pose)

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

    def execute(self, fiducial=None):
        for action in self.actions:
            print('Executing:', type(action).__name__)
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

def create_new_program(gripper, arm):
    new_program = DemonstrationProgram()
    print('\t add [abs] [tag] | to add a new pose relative to the abs, tag optional')

    print('\t open | to add an open gripper action')
    print('\t close | to add a close gripper action')
    print('\t tuck | tuck the arm')
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


    print("you are not crazy you are here")
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
            print('>> Added move action')
        elif user_input == 'open':
            new_program.add_action(GripperAction(True, gripper))
            print('>> Added open action')
        elif user_input == 'close':
            new_program.add_action(GripperAction(False, gripper))
            print('>> Added close action')
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
    print('\t execute [name] | to execute a new program')

    print('\t list | to list all programs')
    print('\t exit | to exit out of this joint')
    print('')

class ProgramHandler(object):
    def __init__(self, load_file):
        print("Initializing the gripper and arm...")
        print("To quit before gripper/arm are initialized, do ctrl-C and then ctrl-D.")
        gripper = Gripper()
        print("Gripper ready...")
        arm = Arm()
        print("Arm ready...")
        self.program_info = load_program(load_file, gripper, arm)

    def get_program(self, program_name):
        return self.program_info[program_name]

def load_program(name, gripper, arm):
    programs = {}
    try:
        with open(name, "r") as f:
            programs = pickle.load(f)
            for key, program in programs.items():
                for action in program.actions:
                    if type(action).__name__ == 'GripperAction':
                        action.gripper = gripper
                    elif type(action).__name__ == 'MoveAction' or type(action).__name__ == 'TuckAction':
                        action.arm = arm
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

    # TODO(emersonn): This is where we start the interface
    print('Welcome to the best demonstration program in North America')

    FILENAME = "/home/team3/catkin_ws/src/cse481c/acciobot_main/demonstration.pickle"
    programs = load_program(FILENAME, gripper, arm)

    # For persisting the file to disk
    def pickle_it_up():
        with open(FILENAME, "w") as f:
            pickle.dump(programs, f)
    rospy.on_shutdown(pickle_it_up)


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
            new_program = create_new_program(gripper, arm)
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
            if program_name in programs:
                programs[program_name].execute()
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
