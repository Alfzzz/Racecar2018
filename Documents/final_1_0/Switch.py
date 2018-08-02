#! /usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import right
import left

current_wall = 0
class ar_switch():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size = 1)
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        self.cmd_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)

        self.rins = right.Follow_Right_Wall()
        self.lins = left.Follow_Left_Wall()

        self.flag = None
        self.last_ar = None
        self.minimum = None

    def ar_callback(ar_markers):

        if len(ar_markers.markers) > 1:
            for i in range(1, len(ar_markers.markers)):
                minimum = ar_markers.markers[0].id
                if ar_markers.markers[i].pose.pose.position.y > minimum:
                    minimum = ar_markers.markers[i].id
            self.flag = 1
        elif len(ar_markers.markers) == 1:
            minimum = ar_markers.markers[0].id
            self.flag = 1
        else:
            pass

        if self.flag == None:
            self.last_ar = 0
        else:
        if minimum < 50:
                self.last_ar = minimum
        else:
                pass
        print (self.last_ar)


        if self.last_ar == 19 or self.last_ar == 23:
            self.rins.ackermann_cmd_input_callback(AckermannDriveStamped())

        elif self.last_ar == 18 or self.last_ar == 22:
            self.lins.ackermann_cmd_input_callback(AckermannDriveStamped())

if __name__ == "__main__":
    rospy.init_node("ar_switch", anonymous = True)
    node = ar_switch()
    rospy.spin()
