#!/usr/bin/env python
import rospy, numpy as np, tf
from math import pi
from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import LaserScan, Imu
from rolling_average import RollingAverage

class WallFollowerNode:
    #PD controller used to follow walls on the left or right side
    def __init__(self):
        # For self.follow_wall
        self.follow = "right"
        self.speed = 2
        # IMU gyroscope angle beyond which we follow right wall again. See laser_callback logic.
        self.yaw = None
        self.left_follow_limit = None
        # For initializing some constants.
        self.first_scan = True
        # Defined in laser_callback upon first scan.
        self.laser_index_left = None
        self.laser_index_right = None
        self.laser_index_front = None
        # Data points used in calculation of PID errors.
        # Smooth out data at expense of some latency. Primarily for derivative term of PID steering control.
        self.laser_rolling_average_left = RollingAverage(6)
        self.laser_rolling_average_right = RollingAverage(6)
        # The data point from last scan. Used to approximate derivative of error.
        self.previous_average_left = None
        self.previous_average_right = None
        # Distance from wall to follow.
        self.setpoint = 1.0
        # Message to be picked up by state machine.
        self.wall_follower_left_pub = rospy.Publisher("/wall_follower_left_message", AckermannDriveStamped, queue_size=1)
        self.wall_follower_right_pub = rospy.Publisher("/wall_follower_right_message", AckermannDriveStamped, queue_size=1)
        self.wall_follower_pub = rospy.Publisher("/wall_follower_message", AckermannDriveStamped, queue_size=1)
        # Callbacks.
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)

    def follow_wall(self, scan_data, follow, speed):
        #Returns drive message
        #print("in follow wall")
        # Left and right 
        laser_distance_left = min(scan_data.ranges[self.laser_index_left - 5 : self.laser_index_left + 6])
        laser_distance_right = min(scan_data.ranges[self.laser_index_right - 5 : self.laser_index_right + 6])
        self.laser_rolling_average_left.update(laser_distance_left)
        self.laser_rolling_average_right.update(laser_distance_right)
        if self.laser_rolling_average_left.populated:
            current_average_left = self.laser_rolling_average_left.calculate_average()
            #print("current_average: ", current_average)
            if self.previous_average_left != None:
                error = current_average_left - self.setpoint
                #print("error: ", error)
                error_derivative = (current_average_left - self.previous_average_left) / scan_data.scan_time
                left_drive_msg = AckermannDriveStamped()
                left_drive_msg.drive.speed = speed
                K_P = 1.2 / speed
                K_D = 0.5 / speed
                # PID control of steering angle with no integral term.
                left_drive_msg.drive.steering_angle = (K_P * error + K_D * error_derivative)
                self.wall_follower_left_pub.publish(left_drive_msg)
                if follow == "left":
                    self.wall_follower_pub.publish(left_drive_msg)
            self.previous_average_left = self.laser_rolling_average_left.calculate_average()
        if self.laser_rolling_average_right.populated:
            current_average_right = self.laser_rolling_average_right.calculate_average()
            #print("current_average: ", current_average)
            if self.previous_average_right != None:
                error = current_average_right - self.setpoint
                #print("error: ", error)
                error_derivative = (current_average_right - self.previous_average_right) / scan_data.scan_time
                right_drive_msg = AckermannDriveStamped()
                right_drive_msg.drive.speed = speed
                K_P = 1.2 / speed
                K_D = 0.5 / speed
                # PID control of steering angle with no integral term.
                right_drive_msg.drive.steering_angle = -(K_P * error + K_D * error_derivative) # Negative sign turns right.
                self.wall_follower_right_pub.publish(right_drive_msg)
                if follow == "right":
                    self.wall_follower_pub.publish(right_drive_msg)
            self.previous_average_right = self.laser_rolling_average_right.calculate_average()

    def laser_callback(self, scan_data):
        """Use laser value 90 degrees to the left or right of racecar to follow wall."""
        #print("in laser callback")
        # Determine indices in scan_data.ranges for 90 degrees left and right.
        if self.first_scan:
            self.laser_index_left = int(round((scan_data.angle_min + pi/2) / scan_data.angle_increment))
            self.laser_index_right = int(round((scan_data.angle_max - pi/2) / scan_data.angle_increment))
            self.laser_index_front = len(scan_data.ranges) // 2
            self.first_scan = False
        #print("laser_index_left: ", self.laser_index_left)
        #print("laser_index_right: ", self.laser_index_right)
        # Logic for which wall to follow goes here.
        if self.follow == "right":
            laser_distance_front = min(scan_data.ranges[self.laser_index_front - 5 : self.laser_index_front + 6])
            laser_distance_right = min(scan_data.ranges[self.laser_index_right - 5 : self.laser_index_right + 6])
            if laser_distance_front < 1.35 and laser_distance_right < self.setpoint + 0.5:
                if self.yaw == None:
                    return
                self.follow = "left"
                self.left_follow_limit = self.yaw + pi/2 * 7 / 9 # Switch back to right wall follower at 70 degrees
                if self.left_follow_limit > pi:
                    self.left_follow_limit -= 2*pi
                if self.left_follow_limit < -pi:
                    self.left_follow_limit += 2*pi
        if self.follow == "left":
            if self.yaw > self.left_follow_limit and \
               self.yaw < self.left_follow_limit + pi/2:
                self.follow = "right"
            print(self.yaw - self.left_follow_limit)
        self.follow_wall(scan_data, self.follow, self.speed)
        print(self.follow)

    def imu_callback(self, imu_data):
        o = imu_data.orientation
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion((o.x, o.y, o.z, o.w))
        
if __name__ == "__main__":
    try:
        rospy.init_node("WallFollower")
        wall_follower = WallFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        exit()
