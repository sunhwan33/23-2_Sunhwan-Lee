#!/usr/bin/env python3

# python package
import numpy as np
import copy

# ros package
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class ReactiveFollowGap:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_SPEED = 4
    CORNERS_SPEED_1 = 3
    CORNERS_SPEED_2 = 1.5
    STRAIGHTS_STEERING_ANGLE_1 = np.pi / 18  # 10 degrees
    STRAIGHTS_STEERING_ANGLE_2 = np.pi / 9  # 20 degrees

    def __init__(self):
        # Topics & Subs, Pubs
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        self.velocity_sub = rospy.Subscriber("/vesc/odom", Odometry, self.callback_vel)
        self.drive_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size = 10 )
        
        self.ackermann_data = AckermannDriveStamped()
        self.ackermann_data.drive.acceleration = 0
        self.ackermann_data.drive.jerk = 0
        self.ackermann_data.drive.steering_angle = 0
        self.ackermann_data.drive.steering_angle_velocity = 0

    def callback_vel(self,data):
        #TO DO: Subscribe Current Velocity
        self.current_velocity = data.twist.twist.linear.x  # Assuming 'velocity' is the field in 'data' containing the velocity value
        
    def scan_callback(self, scan_msg):
        self.radians_per_elem = (2 * np.pi) / len(scan_msg.ranges)
        proc_ranges = np.array(scan_msg.ranges[135:-135])
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)

        # Find closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        
        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Publish Drive message
        angle = self.get_angle(best, len(proc_ranges))
            
        
        if abs(angle) > self.STRAIGHTS_STEERING_ANGLE_2:
            velocity = self.CORNERS_SPEED_2
        elif abs(angle) > self.STRAIGHTS_STEERING_ANGLE_1:
            velocity = self.CORNERS_SPEED_1
        else:
            velocity = self.STRAIGHTS_SPEED
            
            
        if self.current_velocity !=0:
        	ttc = min(scan_msg.ranges[480:600]) / self.current_velocity
        else:
            ttc = 1
            
        if ttc<0.5 or min(scan_msg.ranges[480:600]) < 0.4:
            velocity = 0
        # publish drive message
        self.ackermann_data.drive.speed = velocity
        self.ackermann_data.drive.steering_angle = angle
        self.ackermann_data.drive.steering_angle_velocity = 0
        self.ackermann_data.drive.acceleration = 0
        self.ackermann_data.drive.jerk = 0
        self.drive_pub.publish(self.ackermann_data)


    def find_max_gap(self, free_space_ranges):
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop
    
    def find_best_point(self, start_i, end_i, ranges):
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

if __name__ == '__main__':
    rospy.init_node("reactive_node")
    print("FGM Initialized")
    fgm_node = ReactiveFollowGap()
    rospy.spin()
