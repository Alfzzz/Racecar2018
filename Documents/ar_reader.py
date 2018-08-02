#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

AR_TOPIC = "/ar_pose_marker"
flag = None
def ar_callback(ar_markers):
    global flag
    #for marker in ar_markers.markers:
        #ar_id = marker.id
        #marker.pose.pose.position.y
        #last_id = ar_id
    if len(ar_markers.markers) > 1:
        for i in range(1, len(ar_markers.markers)):
            minimum = ar_markers.markers[0].id
            if ar_markers.markers[i].pose.pose.position.y < minimum:
                minimum = ar_markers.markers[i].id
        flag = 1
        last_ar = minimum
    elif len(ar_markers.markers) == 1:
        minimum = ar_markers.markers[0].id
        flag = 1
        last_ar = minimum
    else:
        pass


    if flag = None:
        minimum = 0
    else:
        minimum = last_ar
    print minimum

if __name__ == "__main__":
    try:
        rospy.init_node("ar_pose_node")
        ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, ar_callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        exit()
