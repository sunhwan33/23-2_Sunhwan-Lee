#!/usr/bin/env python3

# ros package
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
# from nav_msgs.msg import Odometry

class WallFollower:
    def __init__(self):
        # Topics & Subs, Pubs
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.drive_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size = 10 )
        # self.velocity_sub = rospy.Subscriber("/vesc/odom", Odometry, self.callback_vel)#: Subscribe to VESC
        self.wall_distance = 0.7
        self.forward_speed = 3
        self.angular_speed = 1.0

    def scan_callback(self, scan_msg):
        if len(scan_msg.ranges) == 0:
            return

        left = scan_msg.ranges[720]
        drive_msg = AckermannDriveStamped()
        if left > 1.5:      # If you move away from the left wall, you will get closer to the left wall.
            drive_msg.drive.steering_angle = 0.3
            drive_msg.drive.speed = 3
        elif left < 0.8:    # If you get too close to the left wall, you move to the right.
            drive_msg.drive.steering_angle = -0.3
            drive_msg.drive.speed = 3
        else: # When you keep a certain distance from the wall, you drive at high speed.
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = 3
        
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    rospy.init_node("wall_follower")
    print("wall_follower Initialized")
    wf_node = WallFollower()
    rospy.spin()
