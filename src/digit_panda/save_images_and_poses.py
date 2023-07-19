import cv2
import copy
import numpy
import os
import sys
import rospy
from datetime import datetime
from digit_interface.digit import Digit
from digit_panda.srv import Command, CommandResponse
from enum import Enum, auto
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
import configparser
from threading import Lock
import json
import pyquaternion as pyq

class State(Enum):

    CollectDigitData = auto()
    FreezeObjectPose = auto()
    Idle = auto()
    Save = auto()


class StateMachine:

    def __init__(self):
        self.state = State.Idle


    def set_state(self, state):
        self.state = state


    def get_state(self):
        return self.state


class SaveImagesAndPosesReal():

    def __init__(self, config_file_path):

        """
        Constructor.
        """

        config = configparser.ConfigParser()
        config.read(config_file_path)

        self.digit_transform = config.get('Transformations', 'digit_transform')
        self.digit_transform = json.loads(self.digit_transform)
        self.digit_transform = numpy.array(self.digit_transform)

        self.camera_transform = config.get('Transformations', 'camera_transform')
        self.camera_transform = json.loads(self.camera_transform)
        self.camera_transform = numpy.array(self.camera_transform)

        rospy.init_node('digit_node')

        # Create folder for data output
        self.output_path = '/home/panda-admin/Documents/gcaddeo/' + datetime.now().strftime('%d_%m_%Y_%H_%M_%S') + '/'
        self.output_path_images = self.output_path + '/images'
        os.makedirs(self.output_path_images, exist_ok = True)
        rospy.loginfo('Saving data in ' + self.output_path)

        # Connect the DIGIT.
        self.digit = None
        # self.digit = Digit('D20016')
        # self.digit.connect()

        # State machine
        self.counter = 0
        self.state = StateMachine()

        self.franka_state = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.callback_pose_ee)
        self.object_state = rospy.Subscriber('/aruco_board_detector/board_pose', PoseStamped, self.callback_pose_aruco)
        self.command_service = rospy.Service('~command', Command, self.callback_service)

        # Publishers
        self.pub_object_pose = rospy.Publisher('/digit_node/object_pose', PoseStamped, queue_size =10)
        self.pub_digit_pose = rospy.Publisher('/digit_node/digit_pose', PoseStamped, queue_size =10)

        # Mutex
        self.mutex = Lock()

        # End effector pose
        self.pose_franka_ee = None

        # Aruco pose
        self.pose_aruco = None

        # Static object pose
        self.root_to_object_static = None

        # Storage
        self.poses_digit = []
        self.poses_object = []


    def loop(self):

        """
        Update the module by calling this function every getPeriod().
        """

        while not rospy.is_shutdown():

            if self.digit is not None:
                frame = self.digit.get_frame()
                frame_viz = copy.deepcopy(frame)
                frame_viz = cv2.resize(frame_viz, (int(frame.shape[1] * 4), int(frame.shape[0] * 4)))
                cv2.imshow('image', frame_viz)

            with self.mutex:

                actual_state = self.state.get_state()

                root_to_object = None
                root_to_object_published = None

                if self.root_to_object_static is not None:
                    # If we fixed the object pose, stream that instead of the current object pose
                    root_to_object_published = self.root_to_object_static

                elif (self.pose_aruco is not None) and (self.pose_franka_ee is not None):
                    board_transform = pyq.Quaternion\
                                      (
                                          w = self.pose_aruco.orientation.w,
                                          x = self.pose_aruco.orientation.x,
                                          y = self.pose_aruco.orientation.y,
                                          z = self.pose_aruco.orientation.z
                                      ).transformation_matrix
                    board_transform[0,3] = self.pose_aruco.position.x
                    board_transform[1,3] = self.pose_aruco.position.y
                    board_transform[2,3] = self.pose_aruco.position.z

                    # test
                    object_transform = numpy.eye(4)
                    object_transform[0,3] = 0.075
                    object_transform[1,3] = 0.2
                    object_transform[2,3] = 0.09237

                    root_to_object = self.pose_franka_ee @ self.camera_transform @ board_transform @ object_transform
                    root_to_object_published = root_to_object

                if root_to_object_published is not None:
                    self.pub_object_pose.publish(self.set_pose(root_to_object_published))


                if self.pose_franka_ee is not None:

                    ## Create a matrix
                    pose_digit = self.pose_franka_ee @ self.digit_transform
                    self.pub_digit_pose.publish(self.set_pose(pose_digit))

                if actual_state == State.Idle:

                    pass

                elif actual_state == State.CollectDigitData:

                    pass

                elif actual_state == State.Save:

                    pass

                elif actual_state == State.FreezeObjectPose:
                    rospy.loginfo('Freezing the object pose.')

                    self.root_to_object_static = root_to_object

                    self.state.set_state(State.Idle)

            cv2.waitKey(33)


    def set_pose(self, matrix_pose):

        pose_stamped = PoseStamped()
        pose_quat = pyq.Quaternion(matrix=matrix_pose[:3, :3])

        pose_stamped.pose.orientation.w = pose_quat.w
        pose_stamped.pose.orientation.x = pose_quat.x
        pose_stamped.pose.orientation.y = pose_quat.y
        pose_stamped.pose.orientation.z = pose_quat.z
        pose_stamped.pose.position.x = matrix_pose[0,3]
        pose_stamped.pose.position.y = matrix_pose[1,3]
        pose_stamped.pose.position.z = matrix_pose[2,3]

        return pose_stamped


    def callback_service(self, req):

        command = req.command

        with self.mutex:
            if command == 'freeze_object_pose':
                self.state.set_state(State.FreezeObjectPose)
            elif command == 'save':
                self.state.set_state(State.Save)
        #         # numpy.savetxt(self.output_path + '/poses.txt', numpy.array(self.poses))
        #         # numpy.savetxt(self.output_path + '/poses_aruco.txt', numpy.array(self.poses))

        return CommandResponse()


    def callback_pose_ee(self, data):

        with self.mutex:
            self.pose_franka_ee = numpy.zeros((4,4))
            self.pose_franka_ee[:, 0] = data.O_T_EE[:4]
            self.pose_franka_ee[:, 1] = data.O_T_EE[4:8]
            self.pose_franka_ee[:, 2] = data.O_T_EE[8:12]
            self.pose_franka_ee[:, 3] = data.O_T_EE[12:]


    def callback_pose_aruco(self, data):

        with self.mutex:
            self.pose_aruco = data.pose


def main():
    fn = sys.argv[1]
    if os.path.exists(fn):
        print(os.path.basename(fn))
    else:
        print('pass a valid file')
        exit(1)
    module = SaveImagesAndPosesReal(fn)

    module.loop()
