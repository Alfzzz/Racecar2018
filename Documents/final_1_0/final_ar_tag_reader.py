from ar_track_alvar_msgs.msg import AlvarMarkers

class arTagReader():
  def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 10)

    def callback(self,marker):
        if len(marker.markers) > 0:
            if marker.markers[0].pose.id
                return marker.markers[0].id

if __name__ == "__main__":
    rospy.init_node("ar_switch", anonymous = True)
    node = ar_switch()
    rospy.spin()
