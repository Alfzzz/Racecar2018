#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

AR_TOPIC = "/ar_pose_marker"

def ar_callback(ar_markers):
    for marker in ar_markers.markers:
        ar_id = marker.id
        marker.pose.pose.position.x
        last_id = ar_id
        print last_id
    if len(marker.markers) >= 1:
        for i in range(1, len(marker.markers)):
            minimum = ar_markers.markers[0].id
            if ar_markers.markers[i].pose.x < minimum:
                minimum = ar_markers.markers[i].id
        print minimum

if __name__ == "__main__":
    try:
        rospy.init_node("ar_pose_node")
        ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, ar_callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        exit()
