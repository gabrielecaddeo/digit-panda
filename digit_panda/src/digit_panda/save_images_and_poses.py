import cv2
from digit_interface.digit import Digit
from enum import Enum, auto
import math
import numpy as np
import os
import sys
from threading import Lock
import time
import xml.etree.ElementTree as et
import yarp
import rospy
from franka_msgs.msg import FrankaState

# Not useful
from digit_panda.srv import Command


dirname = os.path.abspath(os.path.dirname(__file__))

if (os.path.isdir(dirname + "/images")):
    images_directory = (dirname + "/images")

else:
    print("Creating directory..")
    os.mkdir(dirname + "/images")
    images_directory = (dirname + "/images")


class State(Enum):

    SaveImage = auto()
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

        # Connect the DIGIT.
        self.digit = Digit("D20066")
        self.digit.connect()
        self.counter = 0
        self.state = StateMachine()
        self.label = 0
        self.mutex = Lock()
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.callback_pose)
        rospy.Subscriber('/save_pose_image', String, self.callback_save)
        print("Deleting label and poses file")
        f = open(dirname + "/labels.csv", "w")
        f.close()
        f = open(dirname + "/poses.txt", "w")
        f.close()
        self.pose = 0

    def loop(self):

        """
        Update the module by calling this function every getPeriod().
        """
        while not rospy.is_shutdown():

            with self.mutex:
                frame = self.digit.get_frame()
                actual_state = self.state.get_state()
                
                
                if actual_state == State.Idle:
                    pass

                elif actual_state == State.SaveImage:

                    print("SaveImage state")

                    f = open(dirname + "/labels.csv", "a")
                    f.write("Image" + str(self.counter) + ".png" + ", " + str(self.label) + "\n")
                    f.close()
                    print("Saving number " + str(self.counter))
                    cv2.imwrite(images_directory + "/Image" + str(self.counter) + ".png", frame)
                    print(self.pose)
                    self.counter += 1
                    self.state.set_state(State.Idle)

                cv2.imshow("image", frame)
                cv2.waitKey(3)


    def callback_save(self, req):
        command = req.command
        with self.mutex:
            if command == 'save':
                self.state.set_state(State.SaveImage)


    def callback_pose(self, req):
        command = req.O_T_EE
        with self.mutex:
            self.pose = command

if __name__ == '__main__':

    module = SaveImagesAndPosesReal()
    module.loop()
