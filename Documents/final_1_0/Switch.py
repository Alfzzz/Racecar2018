#! /usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import os
import time
import right
import left


class ar_switch():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)
        self.master = 0

    def callback(self,marker):
        if (len(marker.markers) > 0) and:
            if marker.markers.id != None:
                print(marker.markers[0].id)
                if marker.markers[0].id == 1:
                    right.right()


                elif marker.markers[0].id == 4:
                    left.left()
                else:
                    pass

if __name__ == "__main__":
    rospy.init_node("ar_switch", anonymous = True)
    node =ar_switch()
rospy.spin()
