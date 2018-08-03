
#!/usr/bin/env python
import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
#import switch

class Follow_Wall_Left():

    def __init__(self):
        
        #::::::::::::::::::::::::::::::::::::: SUBSCRIBERS :::::::::::::::::::::::::::::::::
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
 
        #::::::::::::::::::::::::::::::::::::: PUBLISHERS ::::::::::::::::::::::::::::::::::
        self.cmd_pub = rospy.Publisher('ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 10)


        self.maxSpeed = 1

        #::::::::::::::::::::::::::::::::::::: WALL FOLLOWER :::::::::::::::::::::::::::::::
        self.velCoeff = 1
        self.futCon = 0

        self.prev_error = 0

        self.averageL = 0
        self.futureL = 0

        self.wall = 0

        self.averageR = 0
        self.futureR = 0
        self.idealDis = 0.5 # En el caso de L or R

        self.output = 0

        self.current_time = time.time() #No estpy seguro pero nos puede ayudar en un futuro
        self.prev_time = 0

    def laser_callback(self,msg):
        
        ranges = msg.ranges

        #Right average
        self.futureL = np.mean(ranges[600: 740])
        #print("future L = {}".format(self.futureL)) ####
        self.averageL = np.mean(ranges[740 : 900])
        #print("average L = {}".format(self.averageL))
        #Front average
        self.wall = np.mean(ranges[480 : 600])

        self.PID(1.0, 1.2, 0.0, 0.4, 'Left')

        #3 possible wall followers, Left, Right, LR, Close Line
    def PID(self, maxSpeed, kp, ki, kd, mode):

        #print("ENTRANDO A PID")
        if mode == 'Left':
            dir = 1
            error = self.averageL - self.idealDis
            #print(error)
            if self.futureL >= (2 * self.averageL):
                self.futCon = - 0.1
            elif self.futureL <= self.averageL:
                self.futCon = 0.1
            else:
                self.futCon = 0

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
        '''
        if self.futureR >= (2 * self.averageR):
            self.futCon = - 0.1
        elif self.futureR <= self.averageR:
            self.futCon = 0.1
        '''
        self.output = (prop + integ + deriv + self.futCon) * dir

        if abs(self.output) >= 0.34:
            self.velCoeff = 0.7

        elif abs(self.output) >= 0.25:
            self.velCoeff = 0.9

        else:
            self.velCoeff = 1

        #print("P = {} I = {} D = {}".format(round(prop, 4), round(integ, 4), round(deriv, 4)))
        #print("Angle = {}".format(self.output))
        
        #print("SALIENDO PID")

        #self.ackermann_cmd_input_callback(AckermannDriveStamped())

    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = self.maxSpeed * self.velCoeff
        msg.drive.steering_angle = self.output
        msg.drive.steering_angle_velocity = 1
	print("Left")
        self.cmd_pub.publish(msg)
        
if __name__ == "__main__":
    try:
        rospy.init_node("Follow_Wall_Left")
        node = Follow_Wall_Left()
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    #rospy.spin()
    except rospy.ROSInterruptException:
        pass
