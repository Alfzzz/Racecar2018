#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import cv2

AR_TOPIC = "/ar_pose_marker"
flag = None
last_ar = None
minimum = None

mode = "Right"

tags_left = [14,23,22]
tags_right = [18,30,24]

def ar_callback(ar_markers):
    global mode 
    global flag
    global last_ar
    global minimum

    if len(ar_markers.markers) > 1:
        for i in range(1, len(ar_markers.markers)):
            minimum = ar_markers.markers[0].id
            if ar_markers.markers[i].pose.pose.position.y > minimum:
                minimum = ar_markers.markers[i].id
        flag = 1
    elif len(ar_markers.markers) == 1:
        minimum = ar_markers.markers[0].id
        flag = 1
    else:
        pass


    if flag == None:
        last_ar = 0
    
    if minimum:
        if minimum in tags_left:
            mode = "Right"
        elif minimum in tags_right:
            mode = "Left"
        
    
    """   
    else:
	   if minimum < 50:
    	    last_ar = minimum
	   else:
        pass
      print last_ar
     """

class Follow_Wall():
     
    global mode

    def __init__(self):
        
        #::::::::::::::::::::::::::::::::::::: SUBSCRIBERS :::::::::::::::::::::::::::::::::
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
 
        #::::::::::::::::::::::::::::::::::::: PUBLISHERS ::::::::::::::::::::::::::::::::::
        self.cmd_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)


        self.maxSpeed = 1

        #::::::::::::::::::::::::::::::::::::: WALL FOLLOWER :::::::::::::::::::::::::::::::
        self.velCoeff = 1
        # self.futCon = 0

        self.prev_error = 0

        self.averageL = 0
        self.futureL = 0

        self.wall = 0

        self.averageR = 0
        self.futureR = 0
        self.idealDis = 0.5 # En el caso de L or R

        self.output = 0

        #:::::::::::::::::::::::::::::::::::: LINE :::::::::::::::::::::::::::::::::::::::::

        self.contours = 0

        self.current_time = time.time() #No estpy seguro pero nos puede ayudar en un futuro
        self.prev_time = 0
    def laser_callback(self,msg):
        
        ranges = msg.ranges

        #Right average
        self.averageR = np.mean(ranges[180 : 340])
        self.futureR = np.mean(ranges[340 : 480])
        #Front average
        self.wall = np.mean(ranges[480 : 600])
        #Left average
        self.futureL = np.mean(ranges[600: 740])
        self.averageL = np.mean(ranges[740 : 900])

        self.PID(1.0, 1.2, 0.0, 0.4, 'Close')

        #3 possible wall followers, Left, Right, LR, Close Line
    def PID(self, maxSpeed, kp, ki, kd, mode):
        dir = 1
        if  mode == 'Right':
            error = self.averageR - self.idealDis
            dir = -1 
        elif mode == 'Left':
            error = self.averageL - self.idealDis
            print(error)
        elif mode == 'LR':
            error = (((self.averageL + self.averageR) / 2) - self.idealDis)
        elif mode == 'Close':
            if self.averageR < self.averageL:
                error = self.averageR - self.idealDis
                dir = -1 
            else:
                error = self.averageL - self.idealDis
        else:
            if len(self.contours) > 0:
                M = cv2.moments(max(contours, key = cv2.contourArea))
                cx = int(M['m10'] / M['m00'])
            
            elif ZeroDivisionError:
                cx = 640
            
            error = (float(self.cx) - 640) / (640)

        self.maxSpeed = maxSpeed
        # Improve and future
        
        self.current_time = time.time()

        #P
        prop = kp * error

        #I
        integ = ki * ((error + self.prev_error) * (self.current_time - self.prev_time))

        #D
        deriv = kd * ((error - self.prev_error) / (self.current_time - self.prev_time))

        self.prev_error = error
        self.prev_time = self.current_time

        '''
        Probar con el nuevo angulo
        if (self.futureL - self.averageL) >= 0.235:
            self.futCon = - 0.1
        elif (self.futureL - self.averageL) <= 0.185:
            self.futCon = 0.1
        '''

        self.output = (prop + integ + deriv) * dir # + futCon

        if abs(self.output) >= 0.34:
            self.velCoeff = 0.7
        elif abs(self.output) >= 0.25:
            self.velCoeff = 0.9
        else:
            self.velCoeff = 1

        print("P = {} I = {} D = {}".format(round(prop, 4), round(integ, 4), round(deriv, 4)))
        print("Angle = {}".format(self.output))

        self.ackermann_cmd_input_callback(AckermannDriveStamped())

    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = self.maxSpeed * self.velCoeff
        msg.drive.steering_angle = self.output
        msg.drive.steering_angle_velocity = 1
        self.cmd_pub.publish(msg)
        
if __name__ == "__main__":
    try:
        rospy.init_node("ar_pose_node")
        ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, ar_callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
      exit()
    rospy.init_node("Follow_Wall")
    node = Follow_Wall()
rospy.spin()