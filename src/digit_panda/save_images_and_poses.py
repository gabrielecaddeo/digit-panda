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
from geometry_msg.msg import PoseStamped
import configparser
from threading import Lock
import json
import pyquaternion as pyq

class State(Enum):

    Collect = auto()
    Idle = auto()
    SaveAruco = auto()


class StateMachine:

    def __init__(self):
        self.state = State.SaveAruco


    def set_state(self, state):
        self.state = state


    def get_state(self):
        return self.state


class SaveImagesAndPosesReal():

    def __init__(self, config_file_path):

        """
        Constructor.
        """
, 
        config = configparser.ConfigParser()
        config.read(config_file_path)
        
        self.digit_transform = config.get('Transformations', 'digit_transform')
        self.digit_transform = json.loads(self.digit_transform)
        self.digit_transform = np.array(self.digit_transform)

        self.camera_transform = config.get('Transformations', 'digit_transform')
        self.camera_transform = json.loadsself.camera_transform()
        self.camera_transform = np.array(self.camera_transform)

        rospy.init_node('digit_node')

        # Create folder for data output
        self.output_path = '/home/panda-admin/Documents/gcaddeo/' + datetime.now().strftime('%d_%m_%Y_%H_%M_%S') + '/'
        self.output_path_images = self.output_path + '/images'
        os.makedirs(self.output_path_images, exist_ok = True)
        rospy.loginfo('Saving data in ' + self.output_path)

        # Connect the DIGIT.
        self.digit = Digit('D20066')
        self.digit.connect()

        # State machine
        self.counter = 0
        self.state = StateMachine()

        self.franka_state = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.callback_pose)
        self.object_state = rospy.Subscriber('/aruco_board_detector/board_pose', PoseStamped, self.callback_pose_aruco)
        self.command_service = rospy.Service('~command', Command, self.callback_service)

        # Publishers
        self.pub_object_pose = rospy.Publisher('/digit_node/object_pose', PoseStamped, queue_size =10)
        self.pub_digit_pose = rospy.Publisher('/digit_node/digit_pose', PoseStamped, queue_size =10)

        # Mutex
        self.mutex = Lock()
        self.mutex2 = Lock()

        # End effector poses
        self.pose = None
        self.poses = []

        # Aruco poses
        # End effector poses
        self.pose_aruco = None
        self.poses_aruco = []


    def loop(self):

        """
        Update the module by calling this function every getPeriod().
        """
        
        while not rospy.is_shutdown():

            frame = self.digit.get_frame()
            frame_viz = copy.deepcopy(frame)
            frame_viz = cv2.resize(frame_viz, (int(frame.shape[1] * 4), int(frame.shape[0] * 4)))
            cv2.imshow('image', frame_viz)

            with self.mutex and self.mutex2:

                actual_state = self.state.get_state()

                
                if actual_state == State.Idle:
                    pass

                elif actual_state == State.Collect:

                    if self.pose is not None:

                        rospy.loginfo('Saving item no. ' + str(self.counter))

                        rospy.loginfo(self.pose)

                        ## Create a matrix

                        pose_digit = np.zeros((4,4))
                        pose_digit[:, 0] = self.pose[:4]
                        pose_digit[:, 1] = self.pose[4:8]
                        pose_digit[:, 2] = self.pose[8:12]
                        pose_digit[:, 3] = self.pose[12:]

                        pose_digit_final = pose_digit @ self.digit_transform
                        final_quat = pyq.Quaternion(matrix=pose_digit_final[:3, :3])
                        pose_digit_pub = PoseStamped()
                        pose_digit_pub.orientation.w = final_quat.w
                        pose_digit_pub.orientation.x = final_quat.x
                        pose_digit_pub.orientation.y = final_quat.y
                        pose_digit_pub.orientation.z = final_quat.z
                        pose_digit_pub.position.x = pose_digit_final[0,3]
                        pose_digit_pub.position.y = pose_digit_final[1,3]
                        popose_digit_pubse.position.z = pose_digit_final[2,3]

                        self.pub_digit_pose.pub(pose_digit_pub)

                        self.poses.append(numpy.array((self.pose)))
                        self.poses_aruco.append(numpy.array((self.pose)))
                        cv2.imwrite(self.output_path_images + '/' + str(self.counter) + '.png', frame)

                        self.counter += 1
                    else:
                        rospy.loginfo('Error: Cannot read end effector pose')
                    self.state.set_state(State.Idle)
                
                elif actual_state == State.SaveAruco:
                    rospy.loginfo('Saving Object pose')

                    transform_matrix = pyq.Quaternion(self.pose_aruco.orientation.w,self.pose_aruco.orientation.x, self.pose_aruco.orientation.y, self.pose_aruco.orientation.z).transformation_matrix
                    transform_matrix[0,3] = self.pose_aruco.position.x
                    transform_matrix[1,3] = self.pose_aruco.position.y
                    transform_matrix[2,3] = self.pose_aruco.position.z

                    final_transform = self.camera_transform @ transform_matrix
                    final_quat = pyq.Quaternion(matrix=final_transform[:3, :3])
                    pose = PoseStamped()
                    pose.orientation.w = final_quat.w
                    pose.orientation.x = final_quat.x
                    pose.orientation.y = final_quat.y
                    pose.orientation.z = final_quat.z
                    pose.position.x = final_transform[0,3]
                    pose.position.y = final_transform[1,3]
                    pose.position.z = final_transform[2,3]

                    self.pub_object_pose.publish(pose)
                

            cv2.waitKey(33)


    def callback_service(self, req):

        command = req.command

        with self.mutex:
            if command == 'collect':
                self.state.set_state(State.Collect)
            elif command == 'save':
                numpy.savetxt(self.output_path + '/poses.txt', numpy.array(self.poses))
                numpy.savetxt(self.output_path + '/poses_aruco.txt', numpy.array(self.poses))


    def callback_pose(self, data):

        with self.mutex:
            self.pose = data.O_T_EE

    def callback_pose_aruco(self, data):

        with self.mutex2:
            self.pose_aruco = data.pose


def main():
    fn = sys.argv[1]
    if os.path.exists(fn):
        print(os.path.basename(fn)
    else:
        print('pass a valid file')
        exit(1)
    module = SaveImagesAndPosesReal(fn)

    module.loop()
