#! /usr/bin/env python


import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import right
import left


class ar_switch():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)
        self.master = 0

    def callback(self,marker):
        if len(marker.markers) > 0 and marker.markers[0].id != 255:	
            self.master = marker.markers[0].id
            print(self.master)


        if self.master == 19 or self.master == 23:
            right.right()
            print("Right")
        elif self.master == 18 or self.master == 22:
            left.left()
            print("Left")
        else:
            pass

if __name__ == "__main__":
    rospy.init_node("ar_switch", anonymous = True)
    node =ar_switch()
rospy.spin()
