#! /usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import os
import time
import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

AR_TOPIC = "/ar_pose_marker"
flag = None
last_ar = None
selminimum = None

class Switch():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)
        #::::::::::::::::::::::::::::::::::::: SUBSCRIBERS :::::::::::::::::::::::::::::::::
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, ar_callback, queue_size=10)
            rospy.spin()
        #::::::::::::::::::::::::::::::::::::: PUBLISHERS ::::::::::::::::::::::::::::::::::
        self.cmd_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)
        self.maxSpeed = 1
        #::::::::::::::::::::::::::::::::::::: WALL FOLLOWER :::::::::::::::::::::::::::::::
        self.velCoeff = 1
        # self.futCon = 0
        self.prev_error = 0
        self.average = 0
        self.future = 0
        self.wall = 0
        self.idealDis = 0.5 # En el caso de L or R
        self.output = 0
        #:::::::::::::::::::::::::::::::::::: LINE :::::::::::::::::::::::::::::::::::::::::
        self.contours = 0
        self.current_time = time.time() #No estpy seguro pero nos puede ayudar en un futuro
        self.prev_time = 0
        self.state = 0

    def ar_callback(ar_markers):
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
        else:
        if minimum < 50 and ar_markers.markers[i].pose.pose.position.y > 0.5:
                last_ar = minimum
        else:
                pass
        print last_ar

    def laser_callback(self,msg):
        
        if last_ar == 19:
            ranges = msg.ranges
            #Left average
            self. = np.mean(ranges[600: 740])
            print("future = {}".format(self.future)) ####
            self.average = np.mean(ranges[740 : 900])
            print("average = {}".format(self.average))
            #Front average
            self.wall = np.mean(ranges[480 : 600])
            self.PID(1.0, 1.2, 0.0, 0.4, 'Left')

        if last_ar == 18:
            ranges = msg.ranges
            #Right average
            self.average = np.mean(ranges[180 : 340])
            print("future = {}".format(self.future))
            self.future = np.mean(ranges[340 : 480])
            print("average = {}".format(self.average))
            #Front average
            self.wall = np.mean(ranges[480 : 600])
            self.PID(1.0, 1.2, 0.0, 0.4, 'Right')

    def PID(self, maxSpeed, kp, ki, kd, mode):
        print("ENTRANDO A PID")
        error = self.average - self.idealDis
        dir = -1 
        if self.future >= (2 * self.average):
            self.futCon = - 0.1
        elif self.future <= self.average:
            self.futCon = 0.1
        else:
            elf.futCon = 0

        self.maxSpeed = maxSpeed
        self.current_time = time.time()

        #P
        prop = kp * error

        #I
        integ = ki * ((error + self.prev_error) * (self.current_time - self.prev_time))

        #D
        deriv = kd * ((error - self.prev_error) / (self.current_time - self.prev_time))

        self.prev_error = error
        self.prev_time = self.current_time
        self.output = (prop + integ + deriv + self.futCon) * dir

        if abs(self.output) >= 0.34:
            self.velCoeff = 0.7

        elif abs(self.output) >= 0.25:
            self.velCoeff = 0.9

        else:
            self.velCoeff = 1

        print("P = {} I = {} D = {}".format(round(prop, 4), round(integ, 4), round(deriv, 4)))
        print("Angle = {}".format(self.output))

        print("SALIENDO DE PID")
        # self.ackermann_cmd_input_callback(AckermannDriveStamped())

    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = self.maxSpeed * self.velCoeff
        msg.drive.steering_angle = self.output
        msg.drive.steering_angle_velocity = 1
        self.cmd_pub.publish(msg)


    if __name__ == "__main__":
        rospy.init_node("Switch")
        node = Switch()
        rospy.spin()
