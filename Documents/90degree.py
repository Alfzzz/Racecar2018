#!/usr/bin/python

import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers


class Follow_Wall():

    def __init__(self):
        
        #::::::::::::::::::::::::::::::::::::: SUBSCRIBERS :::::::::::::::::::::::::::::::::
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size = 1)

 
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
	
	self.averageF = 0

        self.output = 0

        #:::::::::::::::::::::::::::::::::::: LINE :::::::::::::::::::::::::::::::::::::::::

        self.contours = 0

        self.current_time = time.time() #No estpy seguro pero nos puede ayudar en un futuro
        self.prev_time = 0
        self.flag = None
        self.last_ar = None
        self.minimum = None

    def laser_callback(self,msg):
        
        self.degrees = False
        
        ranges = msg.ranges
        x = []
        y = []

        #Right average
        self.averageR = np.mean(ranges[180 : 340])
        print("future R = {}".format(self.futureR))
        self.futureR = np.mean(ranges[340 : 480])
        print("average R = {}".format(self.averageR))
        #Front average
        self.wall = np.mean(ranges[480 : 600])
        #Left average
        self.futureL = np.mean(ranges[600: 740])
        print("future L = {}".format(self.futureL)) ####
        self.averageL = np.mean(ranges[740 : 900])
        print("average L = {}".format(self.averageL))
	
	    self.averageF = np.mean(ranges[500 : 600])
        print("average F = {}".format(self.averageF))
        
        #90 degrees thing
        
        
        for i in range(len(msg[120:280])):
           x.append(msg.ranges[i] * (120 + (i*msg.angle_increment)))
           
        for i in range(len(msg[300:540]):
           y.append(msg.ranges[i] * (300 + (i*msg.angle_increment)))
           
        while np.mean(y)<np.mean(x):
           self.degrees = True
            



        #3 possible wall followers, Left, Right, LR, Close Line
    def ar_callback(ar_markers):

        if len(ar_markers.markers) > 1:
            for i in range(1, len(ar_markers.markers)):
                minimum = ar_markers.markers[0].id
                if ar_markers.markers[i].pose.pose.position.y > minimum:
                    minimum = ar_markers.markers[i].id
            self.flag = 1
        elif len(ar_markers.markers) == 1:
            minimum = ar_markers.markers[0].id
            self.flag = 1
        else:
            pass

        if self.flag == None:
            self.last_ar = 0
        else:
        if minimum < 50:
            self.last_ar = minimum
        else:
            pass

        print (self.last_ar)


        if self.last_ar == 1 or self.last_ar == 7:
            self.PID(1.0, 1.2, 0.0, 0.4, 'Right')

        elif self.last_ar == 18 or self.last_ar == 4:
            self.PID(1.0, 1.2, 0.0, 0.4, 'Left')

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
        
        else:
            error = 0


        '''elif mode == 'Close':
            if self.averageR < self.averageL:
                error = self.averageR - self.idealDis
                dir = -1 
            else:
                error = self.averageL - self.idealDis'''

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

        if self.futureL >= (2 * self.averageL):
            self.futCon = - 0.1

        elif self.futureL <= self.averageL:
            self.futCon = 0.1

        self.output = (prop + integ + deriv + futCon) * dir

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
        
	if averageF < 1.0 and averageL < 1.0:
		msg.drive.steering_angle = -.25
		msg.drive.speed = 0.4
        if self.angle == True:
          msg.drive.steering_angle = -.34
          
	elif averageF < 1.0 and averageR < 1.0:
		msg.drive.steering_angle = .25
		msg.drive.speed = 0.4
        if self.angle == True:
        msg.drive.steering_angle = .34
        
	else:
		msg.drive.speed = self.maxSpeed * self.velCoeff
        	msg.drive.steering_angle = self.output
        	msg.drive.steering_angle_velocity = 1
        self.cmd_pub.publish(msg)
        
if __name__ == "__main__":
    rospy.init_node("Follow_Wall")
    node = Follow_Wall()
rospy.spin()





current_wall = 0
