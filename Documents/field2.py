#!/usr/bin/env python

import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class Potential_Field():

    def __init__(self):
        # subscribe to Ackermann
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_callback)
        rospy.Subscriber("/scan", LaserScan, self.field)
 
        # publish to Ackermann
        self.steering = None
        self.publish = rospy.Publisher('ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 10)

    def field(self, msg):
        k= .7
        fx1 = []
        fy1 = []
        fx2 = [] 
        fy2 = []
        self.angle = [0,0,0]
        self.angle2 = [0,0,0]
        for i in range(len(msg.ranges)):
          x = msg.ranges[i]*np.cos(i*msg.angle_increment)
          y = msg.ranges[i]*np.sin(i*msg.angle_increment)
          r = (((k-x)**2)+((k-y)**2)**(1/2))
          xa = (x-k)/r**2
          ya = (x-k)/r**2 
          xr = -((x)/(msg.ranges[i]**2))
          yr = -((y)/(msg.ranges[i]**2))
          fx1.append(xa)
          fx2.append(xr)
          fy1.append(ya)
          fy2.append(yr)
        fxa = sum(fx1)
        fya = sum(fy1) 
        fxb = sum(fx2)
        fyb = sum(fy2)
        self.anglea = np.arctan2(fya,fxa)
        self.angleb = np.arctan2(fyb, fxb)
        #if abs(self.angle1)>(np.pi/3):
         #self.angle.append((np.pi/3)) 

        # if len(self.angle)>20 and len(self.angle) is not 0:
         #self.angle.pop(0)
        self.ackermann_callback(AckermannDriveStamped())
        
    def ackermann_callback(self, msg):
        msg.drive.speed = 1.0
        self.final = -(self.anglea + self.angleb) 
        self.yee = (self.final * .34)/((270*np.pi)/180)
        msg.drive.steering_angle =  self.yee
        msg.drive.steering_angle_velocity = 1.0
        print msg.drive.steering_angle
        self.publish.publish(msg)

   

if __name__ == "__main__":
    rospy.init_node("Potential_Field")
    node  = Potential_Field()

rospy.spin()

