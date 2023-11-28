#!/usr/bin/env python
import sys
import math
import numpy as np  

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

#PARAMS 
VELOCITY = 2.0 # meters per second
MIN_ACC = 1.0


class Drive:
    def __init__(self):
        self.stop_signal = 0
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=10)
        self.velocity_sub = rospy.Subscriber("/vesc/odom", Odometry, self.callback_vel)#: Subscribe to VESC
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.callback)#: Subscribe to LIDAR
        
        
    def callback_vel(self,data):
        #TO DO: Subscribe Current Velocity
        self.current_velocity = data.twist.twist.linear.x  # Assuming 'velocity' is the field in 'data' containing the velocity value
        
    def callback(self, data):
        start_degree = 133
        end_degree = -133
        start_radian = (start_degree * 3.14159265359) / 180
        end_radian = (end_degree * 3.14159265359) / 180
        start_index = int((start_radian - data.angle_min) / data.angle_increment)
        end_index = int((end_radian - data.angle_min) / data.angle_increment)
        if start_index > end_index:
	        start_index, end_index = end_index, start_index
        min_distance = min(data.ranges[start_index:end_index])
        print(start_index)
        if self.current_velocity !=0:
        	ttc = min_distance / self.current_velocity
        else:
            ttc = 1
        
        drive_msg = AckermannDriveStamped()
        if ttc<0.35 or min_distance<0.5:
            drive_msg.drive.speed = 0
        else:
            drive_msg.drive.speed = VELOCITY
        self.drive_pub.publish(drive_msg)
        print(min_distance)
        print("ttc : ", ttc)

def main(args):
    rospy.init_node("Drive", anonymous=False)
    driver = Drive()
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)
