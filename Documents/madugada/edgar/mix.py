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
        self.Kp = 2.3
        self.Kd = 1.3
        self.Ki = .01
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = 0.2
        self.pintegral = 0
        self.desiredd = 1
        self.averagel = 0
        self.averager = 0
        self.averagef = 0
	
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
        
        ranges = msg.ranges

        #Right average
        self.averageR = np.mean(ranges[180 : 340])
        #print("future R = {}".format(self.futureR))
        self.futureR = np.mean(ranges[340 : 480])
        #print("average R = {}".format(self.averageR))
        #Front average
        self.wall = np.mean(ranges[480 : 600])
        #Left average
        self.futureL = np.mean(ranges[600: 740])
        #print("future L = {}".format(self.futureL)) ####
        self.averageL = np.mean(ranges[740 : 900])
        #print("average L = {}".format(self.averageL))
	
	self.averageF = np.mean(ranges[500 : 600])
        #print("average F = {}".format(self.averageF))



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
            PID(self.averageR- self.desiredd)
           
        elif self.last_ar == 18 or self.last_ar == 4:
            PID(self.averageL - self.desiredd)
         
    def PID(self, error, reset_prev=False):
        proportional_gain = self.Kp*error
        derivative_gain = self.Kd * (error - self.prev_error) / self.dt
        integral_gain = self.pintegral
        integral_gain += self.Ki * error *self.dt
        

        #update value
        self.control = proportional_gain + integral_gain + derivative_gain
        self.prev_error = error
        self.pintegral = integral_gain

        self.ackermann_cmd_input_callback(AckermannDriveStamped())
        return self.control

    def ackermann_cmd_input_callback(self, msg):
	if self.averageF < 1.0 and self.averageL < 1.0:
	    msg.drive.steering_angle = -.25
	    msg.drive.speed = 1.0
	elif self.averageF < 1.0 and self.averageR < 1.0:
	    msg.drive.steering_angle = .25
	    msg.drive.speed = 0.4
	else:
	    msg.drive.speed = 1.0
            msg.drive.steering_angle = self.control
            msg.drive.steering_angle_velocity = 1
        self.cmd_pub.publish(msg)
        
if __name__ == "__main__":
    rospy.init_node("Follow_Wall")
    node = Follow_Wall()
rospy.spin()





current_wall = 0


    

