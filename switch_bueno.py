#!/usr/bin/env python
import rospy

from switch import Switch
from left import Left
from right import Right

STATES = {'MidWall':0, 'LeftWall': 1, 'RightWall': 2,}

rospy.init_node("switch_final")
current_state = STATES['LeftWall']
rate = rospy.Rate(10)

left=Left()
right=Right()
switch

rospy.loginfo("Buscando")

while not rospy.is_shutdown():
    if switch.id == 1:
        current_state = STATES["LeftWall"]
    elif switch.id == 4:
        current_state = STATES["RightWall]
if current_state == 1:
    print"Right"
    right.laser_callback()
    switch.Ar_tags()
elif current_state ==2:
    print "Left"              
    left.laser_callback()
    switch.Ar_tags()
rate.sleep()                   
