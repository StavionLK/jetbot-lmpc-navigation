#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry # Added for positioning
from geometry_msgs.msg import Twist
import math
import time

class WallFollowerPID:
    def __init__(self):
        rospy.init_node('wall_follower_pid')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # --- LAP TIMER & SPEED SETUP ---
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.start_pos = None
        self.lap_start_time = None
        self.lap_count = 0
        self.min_lap_time = 20.0 
        self.lap_threshold = 0.8  
        self.velocities = []  # List to store speeds for averaging
        # -------------------------------

        # PID gains (stable)
        self.Kp = 1.3
        self.Ki = 0.0
        self.Kd = 0.5

        self.desired_distance = 0.5
        self.front_threshold = 0.5
        self.wall_detect_threshold = 0.9

        self.linear_speed = 0.22
        self.slow_speed = 0.12

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

        self.wall_detected = False
        self.wall_detect_count = 0
        self.wall_detect_required = 8   # scans

        rospy.loginfo("✅ Robust PID Wall Follower Started with Lap & Speed Timer")
        rospy.spin()

    # --- ODOMETRY CALLBACK FOR LAP TIMING & SPEED ---
    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        # Get current linear velocity from odom
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        curr_speed = math.sqrt(vx**2 + vy**2)
        
        # Track velocities during the lap
        if self.lap_start_time is not None:
            self.velocities.append(curr_speed)

        # Initialize start position on first odom message
        if self.start_pos is None:
            self.start_pos = (curr_x, curr_y)
            self.lap_start_time = rospy.get_time()
            return

        # Calculate distance to start point
        dist_to_start = math.sqrt((curr_x - self.start_pos[0])**2 + (curr_y - self.start_pos[1])**2)
        current_time = rospy.get_time()
        
        # Check for lap completion
        if self.lap_start_time is not None:
            elapsed = current_time - self.lap_start_time
            if dist_to_start < self.lap_threshold and elapsed > self.min_lap_time:
                self.lap_count += 1
                
                # Calculate average speed
                avg_speed = sum(self.velocities) / len(self.velocities) if self.velocities else 0.0
                
                print("\n" + "="*30)
                print(f"🏁 PID LAP {self.lap_count} COMPLETE")
                print(f"⏱️  Time: {elapsed:.2f} seconds")
                print(f"🚀 Avg Speed: {avg_speed:.2f} m/s")
                print("="*30 + "\n")
                
                # Reset for next lap
                self.lap_start_time = current_time 
                self.velocities = [] 
    # ------------------------------------------------

    def get_range(self, msg, angle_deg):
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)

        if index < 0 or index >= len(msg.ranges):
            return float('inf')

        r = msg.ranges[index]
        if math.isinf(r) or math.isnan(r):
            return float('inf')

        return r

    def reset_pid(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def laser_callback(self, msg):
        left_dist = self.get_range(msg, 90)
        front_dist = self.get_range(msg, 0)

        cmd = Twist()

        # Phase 1: Go straight until wall is stably detected
        if not self.wall_detected:
            if left_dist < self.wall_detect_threshold:
                self.wall_detect_count += 1
            else:
                self.wall_detect_count = 0

            if self.wall_detect_count >= self.wall_detect_required:
                self.wall_detected = True
                self.reset_pid()
                rospy.loginfo("🟢 Left wall locked")

            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # Strong front obstacle avoidance
        if front_dist < self.front_threshold:
            cmd.linear.x = 0.0
            cmd.angular.z = -1.0
            self.reset_pid()
            self.cmd_pub.publish(cmd)
            return

        # CORRECT LEFT-WALL PID ERROR
        error = left_dist - self.desired_distance

        # Dead zone
        if abs(error) < 0.03:
            error = 0.0

        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        control = (self.Kp * error +
                   self.Ki * self.integral +
                   self.Kd * derivative)

        # Saturation
        angular_z = max(min(control, 1.0), -1.0)

        # Speed adaptation
        cmd.linear.x = self.slow_speed if abs(angular_z) > 0.35 else self.linear_speed
        cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        WallFollowerPID()
    except rospy.ROSInterruptException:
        pass
