#!/usr/bin/env python
import os
from enum import Enum
import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Bool

from constants import *
from drive import Drive
from image_processor import ImageProcessor
from cv_bridge import CvBridge


class State(Enum):
    follow_left = 0
    follow_right = 1
    follow_middle = 2
    find_bobs_bricks = 3
    break_bobs_bricks = 4


class StateMachineNode:

    def __init__(self):
        rospy.init_node('state_machine')

        self.markers_subscriber = rospy.Subscriber(AR_TAG_TOPIC, AlvarMarkers,
                                                   self.markers_callback, queue_size=10)
        self.starter_subscriber = rospy.Subscriber(STARTER_TOPIC, Bool,
                                                   self.start_callback, queue_size=10)

        self.lights_off = False

        self.state = State.follow_middle

        self.drive = Drive()
        self.image_processor = ImageProcessor()
        self.cv_bridge = CvBridge()

        self.drive_speed = 0.8
        self.goal_dist = 0.7

    def markers_callback(self, msg):
        self.update_state(msg.markers)

        if self.state == State.follow_middle:
            self.drive.drive_middle(self.drive_speed)
        elif self.state == State.follow_left:
            self.drive.drive_left(self.goal_dist, self.drive_speed)
        elif self.state == State.follow_right:
            self.drive.drive_right(self.goal_dist, self.drive_speed)
        elif self.state == State.find_bobs_bricks:
            self.drive.drive_right(self.goal_dist, self.drive_speed)
            if self.image_processor.process_wall_img():
                self.state = State.break_bobs_bricks
        elif self.state == State.break_bobs_bricks:
            if self.drive.drive_straight(self.drive_speed, 1.0):
                self.state = State.follow_left

    def update_state(self, markers):
        ids = [m.id for m in markers]
        if self.lights_off:
            self.state = State.follow_middle
        if any([x in ids for x in FOLLOW_MIDDLE_TAGS]):
            self.state = State.follow_middle
        elif any([x in ids for x in FOLLOW_LEFT_TAGS]):
            self.state = State.follow_left
        elif any([x in ids for x in FOLLOW_RIGHT_TAGS]):
            self.state = State.follow_right

    def start_callback(self, msg):
        if msg.data:
            self.lights_off = True
            print("The lights are off.")


if __name__ == '__main__':
    os.system('python starter.py')
    s = StateMachineNode()
    rospy.spin()
