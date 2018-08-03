
#! /usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from ackermann_msgs.msg import AckermannDriveStamped
import time
import right
from right import *
import left

flag=None
current_wall = 0
class ar_switch1():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size = 1)
        #rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, right.Follow_Wall_Right.ackermann_cmd_input_callback)
        #self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 10)

	
        #self.rins = right.Follow_Wall_Right() 
        #self.lins = left.Follow_Wall_Left() 


        self.minimum=0    
    def ar_callback(self, ar_markers):
    	
        global minimum,flag,last_ar
        if len(ar_markers.markers) > 1:
            for i in range(1, len(ar_markers.markers)):
                minimum = ar_markers.markers[0].id
                if ar_markers.markers[i].pose.pose.position.y > minimum:
                    minimum = ar_markers.markers[i].id
            flag = 1
        elif len(ar_markers.markers) == 1:
            minimum = ar_markers.markers[0].id
            flag = 1
        else:
            pass

        if flag == None:
            last_ar = 0
        else:
            if minimum < 50:
                last_ar = minimum
            else:
                pass
        print (last_ar)

if __name__ == "__main__":
    rospy.init_node("ar_switch1", anonymous = True)
    node = ar_switch1()
    rospy.spin()
