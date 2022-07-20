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
from threading import Lock


class State(Enum):

    Collect = auto()
    Idle = auto()


class StateMachine:

    def __init__(self):
        self.state = State.Idle


    def set_state(self, state):
        self.state = state


    def get_state(self):
        return self.state


class SaveImagesAndPosesReal():

    def __init__(self):

        """
        Constructor.
        """

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
        self.command_service = rospy.Service('~command', Command, self.callback_service)

        # Mutex
        self.mutex = Lock()

        # End effector poses
        self.pose = None
        self.poses = []


    def loop(self):

        """
        Update the module by calling this function every getPeriod().
        """
        while not rospy.is_shutdown():

            frame = self.digit.get_frame()
            frame_viz = cv2.resize(frame, (int(frame.shape[1] * 4), int(frame.shape[0] * 4)))
            cv2.imshow('image', frame_viz)

            with self.mutex:

                actual_state = self.state.get_state()

                if actual_state == State.Idle:
                    pass

                elif actual_state == State.Collect:

                    if self.pose is not None:

                        rospy.loginfo('Saving item no. ' + str(self.counter))

                        rospy.loginfo(self.pose)
                        self.poses.append(numpy.array((self.pose)))
                        cv2.imwrite(self.output_path_images + '/' + str(self.counter) + '.png', frame)

                        self.counter += 1
                    else:
                        rospy.loginfo('Error: Cannot read end effector pose')
                    self.state.set_state(State.Idle)

            cv2.waitKey(33)


    def callback_service(self, req):

        command = req.command

        with self.mutex:
            if command == 'collect':
                self.state.set_state(State.Collect)
            elif command == 'save':
                numpy.savetxt(self.output_path + '/poses.txt', numpy.array(self.poses))


    def callback_pose(self, data):

        with self.mutex:
            self.pose = data.O_T_EE


def main():

    module = SaveImagesAndPosesReal()

    module.loop()
