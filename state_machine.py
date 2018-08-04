#!/usr/bin/env python
import rospy
from left import Left
from right import Right
#from start import Start
from enum import IntEnum
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_switch import Ar_Find
import math


STATES = {-1:'MidWall',1:'LeftWall',4:'RightWall',2:"Start",3:"}
	  
		  
def ar_state(id):
    return STATES[id]
		  
def main():
    left=Left()
    right=Right()
    #start=Start()
    ar=Ar_Find()
	  
    rospy.init_node("State_Machine")
    rate=rospy.Rate(10)
    current_state="RightWall"
    rospy.loginfo("Buscando")
	  
    while not rospy.is_shutdown():
	#if not starlight:
        #CURRENTstate
	current_state="RightWall"
        try:
            current_state = ar_state(my_ar_find.id)
        except:
            pass
	  
	if current_state == "RightWall":
	    print"Right"
	    right.rightw()
	    ar.scan()
	elif current_state =="leftWall":
	    print "Left"              
	    left.leftw()
	    start_light.look()
	    ar.scan()
        rate.sleep()
main()	
