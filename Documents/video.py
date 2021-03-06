#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
import time
from wf import PID

class YellowNode:
    def __init__( self ):
        rospy.Subscriber("/zed/rgb/image_rect_color",Image, self.callback, queue_size = 1)
        self.bridge=CvBridge()
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        
        # publish to Ackermann
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 20)

    
        self.kp = -0.35
        self.kd = -.25
        self.ki = .0

        '''
        self.kp = 1.5
        self.kd = 0.8
        self.ki = 0.02
        chidas:
        1.2
        0.4
        0.02
        '''

        self.average = 0
        self.av = 0
        self.idealDis = 0.5
        self.error = 0

        self.prop = 0
        self.deriv = 0
        self.integ = 0
        self.prev_error = 0
        self.output = 0

        self.prev_time = 0
        self.current_time = 0

        
    def callback(self,msg):
        self.msg=msg
        frame = self.bridge.imgmsg_to_cv2(msg)
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   
        lower_yellow = np.array([10,130,220])
        upper_yellow = np.array([25,255,255])
        self.mask=cv2.inRange(self.hsv,lower_yellow,upper_yellow)

        self.rows=self.mask.shape[0]
        self.cols=self.mask.shape[1]
        #area1=np.array([[[self.cols,0], [self.cols/2,0], [self.cols,self.rows],[self.cols,self.rows-1]]],dtype=np.int32)
        #area2=np.array([[[self.cols/2,0], [0,0], [0,self.rows],[1,self.rows]]],dtype=np.int32)
        #cv2.fillPoly(self.mask,area1,0)
        #cv2.fillPoly(self.mask,area2,0)
        cv2.imshow('mask',self.mask)
        contours,hierachy=cv2.findContours(self.mask.copy(), 1, cv2.CHAIN_APPROX_NONE)    
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            self.cx = int(M['m10']/M['m00'])
            self.cy = int(M['m01']/M['m00'])
            cv2.line(self.mask,(self.cx,0),(self.cx,720),(255,0,0),0)
            cv2.line(self.mask,(0,self.cy),(1280,self.cy),(255,0,0),0)
            
        elif ZeroDivisionError:
            self.cx=self.cols/2
            self.cy=self.rows/2
        
        self.error= 320 - cx
        self.wf.PID(self.error)
       
    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = 0.7
        msg.drive.steering_angle = self.control
        msg.drive.steering_angle_velocity = 1.42
        self.cmd_pub.publish(msg)

              
if __name__ == "__main__":
    rospy.init_node("Yellow_Node", anonymous = True)
    node =YellowNode()
    rospy.spin()