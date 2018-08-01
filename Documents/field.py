
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
        self.publish = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)

    def field(self, msg):
        k= .5 
        fx1 = []
        fy1 = []
        for i in range(len(msg.ranges)):
          x = msg.ranges[i]*np.cos(i*msg.angle_increment)
          y = msg.ranges[i]*np.sin(i*msg.angle_increment)
          r = (((k-x)**2)+((k-y)**2)**(1/2))
          xa = (x-k)/r**2
          ya = (x-k)/r**2 
          xr = -((x)/(msg.ranges[i]**2))
          yr = -((y)/(msg.ranges[i]**2))
          fx1.append(xa)
          fx1.append(xr)
          fy1.append(ya)
          fy1.append(yr)
        fx = sum(fx1)
        fy = sum(fy1)
        print fx
        print fy
        self.angle = np.arctan((fy/fx))
        if abs(self.angle)>(np.pi/3):
         self.angle = (np.pi/3)
        print self.angle
        self.ackermann_callback(AckermannDriveStamped())
  
    def ackermann_callback(self, msg):
        msg.drive.speed = 0.7 
        msg.drive.steering_angle = self.angle
        msg.drive.steering_angle_velocity = 1
        self.publish.publish(msg)

if __name__ == "__main__":
    rospy.init_node("Potential_Field")
    node  = Potential_Field()

rospy.spin()



