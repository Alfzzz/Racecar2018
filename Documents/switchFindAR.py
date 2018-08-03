#!/usr/bin/env python
from right import Wall_Follow_Right
from left import Wall_Follow_Left
from finAR import Find_AR

AR_TOPIC = "/ar_pose_marker"
master = None

def switch():
    global master
    master = Find_AR.getId()

    if master == 18:
        Wall_Follower_Right()
    
    elif master == 23:
        Wall_Follow_Left()

if __name__ == "__main__":
    try:
        rospy.init_node("switch")
        rospy.spin()
    except rospy.ROSInterruptException:
exit()
