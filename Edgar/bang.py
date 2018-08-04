import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers

class finalProgram():

    def __init__(self):
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.findSpace)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size = 1)
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 10)
        self.maxSpeed = 1
        self.output = 0
        self.flag = None
        self.last_ar = None
        self.minimum = None
        self.refDist = 0.7
        self.error = 0
        self.kp = 0.6

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
    
    def findSpace(self):
        ranges = msg.ranges
        space = []
        distances = []
        for i in range(len(ranges)):
            if(ranges[i] > 1):
                space.append(i)
            else:
                distances.append(ranges[i])

        ### steering
        ini = space[0]
        last = space[len(space) - 1]
        steering = ((last + ini) / 2)/len(ranges) - 0.5
        if(steering > 0.34):
            steering = 0.34

        if(steering < -0.34):
            steering = -0.34
        self.output = steering
        self.ackermann_cmd_input_callback(AckermannDriveStamped())
        

    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = self.maxSpeed
        msg.drive.steering_angle = self.output
        msg.drive.steering_angle_velocity = 1
        self.cmd_pub.publish(msg)
        
if __name__ == "__main__":
    rospy.init_node("finalProgram")
    node = finalProgram()
    rospy.spin()
