
#!/usr/bin/env python

'''
Author: Edgar LV
02/06/2018
'''
import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
#from ar_track_alvar_msgs.msg import AlvarMarkers

class Wall_Follow():
    def __init__(self):
        # subscribe to Ackermann
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)


        # publish to Ackermann
        self.cmd_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)

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
    
    

    def laser_callback(self,msg):
        #Read from a specific set fo laser shots
        ranges = msg.ranges
        #get average

        self.averageL = np.mean(ranges[740 : 900])
        print("average L = {}".format(self.averageL))



        self.PID(self.averageL-self.desiredd)
        #3 possible wall followers, Left, Right, LR, Close Line

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
        msg.drive.speed = 1.0
        msg.drive.steering_angle = self.control
        msg.drive.steering_angle_velocity = 1
        self.cmd_pub.publish(msg)
                    
 
if __name__ == "__main__":
    rospy.init_node("Wall_Follow")
    node = Wall_Follow()
rospy.spin()