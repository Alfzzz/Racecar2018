#! /usr/bin/env python

import rospy
#from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
import os
import time
import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class ar_switch():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)
        #::::::::::::::::::::::::::::::::::::: SUBSCRIBERS :::::::::::::::::::::::::::::::::
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.callback)
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

        #3 possible wall followers, Left, Right, LR, Close Line


    def callback(self,marker,msg):
        if len(marker.markers) > 0:
            if marker.markers.id != None:
                if marker.markers[0].id == 18 or marker.markers[0].id == 22:
                    print "following left wall"
                    self.PID(1.0, 1.2, 0.0, 0.4, 'Left')
                    msg.drive.speed = self.maxSpeed * self.velCoeff
                    msg.drive.steering_angle = self.output
                    msg.drive.steering_angle_velocity = 1
                    self.cmd_pub.publish(msg)
                if marker.markers[0].id == 23 or marker.markers[0].id == 19:
                    print "following right wall"
                    self.PID(1.0, 1.2, 0.0, 0.4, 'Right')
                    msg.drive.speed = self.maxSpeed * self.velCoeff
                    msg.drive.steering_angle = self.output
                    msg.drive.steering_angle_velocity = 1
                    self.cmd_pub.publish(msg)
                if marker.markers[0].id == None:
                    self.PID(1.0, 1.2, 0.0, 0.4, 'Right')
                    msg.drive.speed = self.maxSpeed * self.velCoeff
                    msg.drive.steering_angle = self.output
                    msg.drive.steering_angle_velocity = 1
                    self.cmd_pub.publish(msg)


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

    #def ackermann_cmd_input_callback(self, msg):
        #msg.drive.speed = self.maxSpeed * self.velCoeff
        #msg.drive.steering_angle = self.output
        #msg.drive.steering_angle_velocity = 1
        #self.cmd_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("ar_switch", anonymous = True)
    node =ar_switch()
    rospy.spin()
