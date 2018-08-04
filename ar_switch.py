import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers


class Ar_Find():
    def __init__(self):
        self.found = False
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.arCallback)
        self.ar_placeholder = AlvarMarkers()
        #Id of the marker
        self.id = 0
        #Distance from the ar tag
        self.distance = 1000

    def scan(self):
        self.process(self.ar_placeholder)


    def process(self, data):
        try:
            self.distance = data.markers[0].pose.pose.position.z
            self.id = data.markers[0].id
            self.found = True
        except IndexError:
            self.distance = 1000
            self.id = -1
            self.found = False

    def arCallback(self, data):
        self.ar_placeholder = data
