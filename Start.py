import cv2
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan


class Start():
    def __init__(self):
        rospy.Subscriber("/zed/left/image_rect_color", Image, self.image_callback, queue_size=10)
        self.bridge = CvBridge()
        self.state_light = True

    def image_callback(self, data):
        self.image_placeholder = data

    def image_callback(self,msg):
        self.msg=msg
        img = self.bridge.imgmsg_to_cv2(self.msg)
        cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        thresh = cv2.inRange(readImage, np.array([200, 245, 210]), np.array([250, 255, 230]))
        im2, contours, heirarchy = cv2.findContours(hsl, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        areas = [cv2.contourArea(c) for c in contours]
        try:
            max_index = np.argmax(areas)
            rospy.loginfo(areas[max_index])
            if areas[max_index] < 0.1:
                self.state_light = True
            else:
                self.state_light= True
        except ValueError:
            self.state_light = False

if __name__ == '__main__':
    rospy.init_node("Start",anonymous = 0)
    node =Start()
    rospy.spin()
    
