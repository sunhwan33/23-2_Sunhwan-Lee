#!/usr/bin/env python
import sys
import numpy as np  

#ROS Imports
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

#PARAMS 
VELOCITY = 2.0 # meters per second

class Drive:
    def __init__(self):
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=10)
        # Set a timer to call the 'publish_drive_msg' method every 0.1 seconds
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_drive_msg)

    def publish_drive_msg(self, event):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = VELOCITY
        self.drive_pub.publish(drive_msg)
        print("hello")

def main(args):
    rospy.init_node("Drive", anonymous=False)
    driver = Drive()
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)
