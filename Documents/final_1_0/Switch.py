#! /usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import Right
import Left


class ar_switch():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)

    def callback(self,marker):
        if len(marker.markers) > 0:
            if marker.markers[0].id == 1:
         	    Right.Follow_Wall()

            elif marker.markers[0].id == 4:
                Left.Follow_Wall()
                   

if __name__ == "__main__":
    rospy.init_node("ar_switch", anonymous = True)
    node =ar_switch()
rospy.spin()
