#!/usr/bin/python

import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
#import right
#import left

class Follow_Wall():

    def __init__(self):
        
        #::::::::::::::::::::::::::::::::::::: SUBSCRIBERS :::::::::::::::::::::::::::::::::
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size = 1)

 
        #::::::::::::::::::::::::::::::::::::: PUBLISHERS ::::::::::::::::::::::::::::::::::
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 10)


        self.maxSpeed = 1.5

        #::::::::::::::::::::::::::::::::::::: WALL FOLLOWER :::::::::::::::::::::::::::::::
        self.velCoeff = 1
        #self.futCon = 0

        self.prev_error = 0
        
        self.safety = .3

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
        self.flag = None
        self.last_ar = None
        self.minimum = None

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


        #3 possible wall followers, Left, Right, LR, Close Line
    def ar_callback(self,ar_markers):

        if len(ar_markers.markers) > 1:
            for i in range(1, len(ar_markers.markers)):
                self.minimum = ar_markers.markers[0].id
                if ar_markers.markers[i].pose.pose.position.z < self.minimum:
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


        if self.last_ar == 20 or self.last_ar == 18 or self.last_ar == 23 or self.last_ar == 17:
            self.PID(0.6, 1.2, 0.0, 0.4, 'Right')

        elif self.last_ar == 19:
            self.PID(0.6, 1.2, 0.0, 0.4, 'Left')
        elif self.last_ar == 20:
            self.PID(0.6, 1.2, 0.0, 0.4, 'LR')

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
            self.velCoeff = 0.8

        elif abs(self.output) >= 0.25:
            self.velCoeff = 0.9

        else:
            self.velCoeff = 1

        self.ackermann_cmd_input_callback(AckermannDriveStamped())

    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = self.maxSpeed * self.velCoeff * (self.wall/self.safety) 
        if  msg.drive.speed > 2:
         msg.drive.speed = 2
        print msg.drive.speed 
        msg.drive.steering_angle = self.output
        msg.drive.steering_angle_velocity = 1
        self.cmd_pub.publish(msg)
        
if __name__ == "__main__":
    rospy.init_node("Follow_Wall")
    node = Follow_Wall()
    rospy.spin()
