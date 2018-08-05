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
        self.cmd_pub = rospy.Publisher('vesc/high_level/ackermann_cmd_mux/input/nav_0||', AckermannDriveStamped, queue_size = 10)

        #::::::::::::::::::::::::::::::::::::: WALL FOLLOWER :::::::::::::::::::::::::::::::
        self.velCoeff = 1
        # self.futCon = 0

        self.prev_error = 0

        self.averageL = 0
        self.futureL = 0

        self.wall = 0

        self.prev_error = 0
        self.sum_error = 0
        self.averageR = 0
        self.futureR = 0
        self.idealDis = 0.7 # En el caso de L or R

        self.output = 0

        #:::::::::::::::::::::::::::::::::::: LINE :::::::::::::::::::::::::::::::::::::::::

        self.current_time = time.time()
        self.prev_time = 0
        self.flag = None
        self.last_ar = None
        self.minimum = None
	self.maxSpeed = 0.4
        self.output = 0
        self.flag = None
        self.last_ar = None
        self.minimum = None
        self.refDist = 0.55
        self.error = 0
	self.error_k1 = 0
        self.kp = 0.6
	self.kd = 0.01

        self.array_average = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # 20 values

        self.output = 0

    def laser_callback(self,msg):
        
        ranges = msg.ranges

        #Right average
        #180
        self.averageR = np.mean(ranges[108 : 332])
        #print("future R = {}".format(self.futureR))
        self.futureR = np.mean(ranges[340 : 480])
        #print("average R = {}".format(self.averageR))
        #Front average
        self.wall = np.mean(ranges[480 : 600])
        #Left average
        self.futureL = np.mean(ranges[600: 740])
        #print("future L = {}".format(self.futureL)) ####
        self.averageL = np.mean(ranges[648: 972])
        #print("average L = {}".format(self.averageL))


        #3 possible wall followers, Left, Right, LR, Close Line
    def ar_callback(self,ar_markers):

        if len(ar_markers.markers) > 1:
            for i in range(1, len(ar_markers.markers)):
                self.minimum = ar_markers.markers[0].id
                if ar_markers.markers[i].pose.pose.position.y > self.minimum:
                    self.minimum = ar_markers.markers[i].id
            self.flag = 1
        elif len(ar_markers.markers) == 1:
            self.minimum = ar_markers.markers[0].id
            self.flag = 1
        else:
            pass

        if self.flag == None:
            self.last_ar = 0
        else:
            if self.minimum < 50:
               self.last_ar = self.minimum
            else:
               pass

        print (self.last_ar)


        if self.last_ar == 2:
            self.PID(0.5, 1.1, 0.0, 0.4, 'Right')

        elif self.last_ar == 4:
            self.PID(0.5, 1.1, 0.0, 0.4, 'Left')
	
	elif self.last_ar == 1:
	    self.output = rospy.Subscriber("/scan", LaserScan, self.findSpace)

    def findSpace(self, msg):
        ranges = msg.ranges
	ranges = ranges[100:981]
        space = []
        distances = []
        for i in range(len(ranges)):
            if(ranges[i] > self.refDist):
                space.append(i)
            else:
                distances.append(ranges[i])
        
        ### steering
        ini = space[0]
        last = space[len(space) - 1]
        steering = 3*(2.0*(((last + ini) / 2.0)/881.0) - 1)

        print("init: ",ini,"last",last,"space: ",len(space), "steering: ", steering)
        
        if(steering > 0.34):
            steering = 0.34

        if(steering < -0.34):
            steering = -0.34
	
	self.error_k1 = self.error
	self.error = steering
        self.output = steering + self.kd*(self.error - self.error_k1) 
        self.ackermann_cmd_input_callback(AckermannDriveStamped())
	return self.output

    def PID(self, maxSpeed, kp, ki, kd, mode):
        dir = 1
        if  mode == 'Right':
            error = self.averageR - self.idealDis
            dir = -1 
        elif mode == 'Left':
            error = self.averageL - self.idealDis
        elif mode == 'LR':
            self.idealDis = (self.averageL + self.averageR) / 2
	    error = self.averageL - self.idealDis

        '''
        elif mode == 'Close':
            if self.averageR < self.averageL:
                error = self.averageR - self.idealDis
                dir = -1 
            else:
                error = self.averageL - self.idealDis
        '''

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
        
        self.output = (prop + integ + deriv ) * dir 

        if abs(self.output) >= 0.34:
	    self.output = 0.34 * dir 
            self.velCoeff = 0.7

        elif abs(self.output) >= 0.25:
            self.velCoeff = 0.85

        else:
            self.velCoeff = 1


        #print("P = {} I = {} D = {}".format(round(prop, 4), round(integ, 4), round(deriv, 4)))
        #print("Angle = {}".format(self.output))

        self.ackermann_cmd_input_callback(AckermannDriveStamped())

    def ackermann_cmd_input_callback(self, msg):
	if self.last_ar == 1:
	    msg.drive.speed = self.maxSpeed
            msg.drive.steering_angle = self.output
            msg.drive.steering_angle_velocity = 1.0
        else:
	    if self.wall < 1.0 and self.averageL < 1.0:
	        msg.drive.steering_angle = -.25
	        msg.drive.speed = 1.0
	    elif self.wall < 1.0 and self.averageR < 1.0:
	        msg.drive.steering_angle = .25
	        msg.drive.speed = 0.4
	    else:
		msg.drive.speed = .4
		msg.drive.steering_angle = self.output
		msg.drive.steering_angle_velocity = 1
	            
        self.cmd_pub.publish(msg)
        

if __name__ == "__main__":
    rospy.init_node("Follow_Wall")
    node = Follow_Wall()
rospy.spin()





    

        

    

