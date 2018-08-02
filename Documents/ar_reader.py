#!/usr/bin/env python
# Written by Leo Belyi

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

AR_TOPIC = "/ar_pose_marker"

def ar_callback(ar_markers):
    for marker in ar_markers.markers:
        ar_id = marker.id
        print marker.pose.pose.x
        print ar_id

if __name__ == "__main__":
    try:
        rospy.init_node("ar_pose_node")
        ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, ar_callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        exit()
