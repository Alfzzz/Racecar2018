#!/usr/bin/env python

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan 

class Potential_Field_Controller():

    def __init__(self):
        self.x = 0
        self.y = 0
        self.k = 0.1
        self.ks = 0.1

        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        self.cmd_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)

    def laser_callback(self,msg):
        self.angle = 0
        self.x = 0
        self.y = 100
        for i in range(0,len(msg.ranges)):
            force = self.k/msg.ranges[i]**2
            self.x -= force*np.cos(theta(i))
            self.y -= force*np.sin(theta(i))


    def translate(value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)
        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    def theta(shot):
        result = transate(shot, 1, 1081, -0.785398, 3.92699)
        return result

    def mapping(temp):
        steeting = translate(temp, 0, 6.28319, -0.34, 0.34)


    def callback(self):
    self.speed = self.ks * (self.x**2+self.y**2)**1/2
    self.phi = np.atan2(self.y,self.x)
    #self.steer = mapping(self.phi)

            
    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.phi
        msg.drive.steering_angle_velocity = 1
        self.cmd_pub.publish(msg)
        print self.phi
        
if __name__ == "__main__":
    rospy.init_node("Follow_Wall")
    node = Follow_Wall()
rospy.spin()
