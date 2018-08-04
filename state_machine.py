#!/usr/bin/env python
import rospy
from left import Left
from right import Right
#from start import Start
from enum import IntEnum
from ar_track_alvar_msgs.msg import AlvarMarkers

class State_Machine:
    def __init__(self):
        self.ar_marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback, queue_size=1)
	STATES = {-1:'MidWall',0:'LeftWall',1:'RightWall',2:"Start",3:"}
	rospy.init_node("switch_final")
        current_state = STATES['LeftWall']
	rate = rospy.Rate(10)
	left=Left()
	right=Right()
	#start=Start()
	rospy.loginfo("Buscando")
		  
    def ar_callback(self,ar_markers):

        if len(ar_markers.markers) > 1:
            markers_distances = [marker.pose.pose.position.z for marker in ar_markers.markers]
            min_index = markers_distances.index(min(markers_distances))
            self.AR_ID = ar_markers.markers[min_index].id
        print self.AR_ID

	while not rospy.is_shutdown():
	    #if not starlight:
            #CURRENTstate
            if self.AR_ID == 1:
		current_state = STATES["LeftWall"]
	    elif self.AR_ID == 4:
		current_state = STATES["RightWall]
	if current_state == 0:
	    print"Right"
	    right.rightw()
	elif current_state ==1:
	    print "Left"              
	    left.leftw()
if __name__ == "__main__":
    try:
        rospy.init_node("State_Machine")
        node = State_Machine()
        rospy.spin()
