#!/usr/bin/env python3

# ros package
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollower:
    def __init__(self):
        # Topics & Subs, Pubs
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.drive_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size = 10 )

        self.wall_distance = 0.7
        self.forward_speed = 5.0
        self.angular_speed = 1.0

    def scan_callback(self, scan_msg):
        if len(scan_msg.ranges) == 0:
            return

        drive_msg = AckermannDriveStamped()
        print(scan_msg.ranges[0], "/", scan_msg.ranges[60], "/", scan_msg.ranges[120], "/", scan_msg.ranges[180], "/", scan_msg.ranges[240], "/", scan_msg.ranges[300], "/", scan_msg.ranges[360], "/", scan_msg.ranges[420], "/", scan_msg.ranges[480], "/", scan_msg.ranges[540], "/", scan_msg.ranges[600], "/", scan_msg.ranges[660], "/", scan_msg.ranges[720], "/", scan_msg.ranges[780], "/", scan_msg.ranges[840], "/", scan_msg.ranges[900], "/", scan_msg.ranges[960], "/", scan_msg.ranges[1020], "/", scan_msg.ranges[1080])
            
        drive_msg.drive.speed = 0
        
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    rospy.init_node("wall_follower")
    print("wall_follower Initialized")
    wf_node = WallFollower()
    rospy.spin()