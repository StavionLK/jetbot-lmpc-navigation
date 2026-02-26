#!/usr/bin/env python3
import rospy
import numpy as np
import cvxpy as cp
import csv
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# --- CONFIGURATION ---
DRIVE_TOPIC = '/cmd_vel'
ODOM_TOPIC = '/odom'
FILENAME = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ref_path.csv')

# Dynamic Parameters
BASE_HORIZON = 18
DT = 0.1
BASE_SPEED = 0.5
MAX_SPEED = 1.6
LAP_THRESHOLD = 0.8
MIN_LAP_STEPS = 150

class LMPCController:
    def __init__(self):
        rospy.init_node('lmpc_node')
        
        self.centerline = self.load_waypoints(FILENAME)
        if self.centerline is None: return

        self.safe_set = self.centerline.copy() 
        self.current_lap_data = []
        self.lap_count = 0
        self.current_target_speed = BASE_SPEED

        # ROS Setup
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_cb)
        
        self.state = np.zeros(4) # [x, y, v, yaw]
        self.odom_received = False
        
        rospy.Timer(rospy.Duration(DT), self.control_loop)
        rospy.loginfo("LMPC Refined: Ready for Lap 1.")

    def load_waypoints(self, path):
        # ... (Keep your existing CSV loading logic here) ...
        pass

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        v = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.state = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, v, yaw])
        self.odom_received = True

    def normalize_angle(self, angle, reference):
        return reference + (angle - reference + np.pi) % (2 * np.pi) - np.pi

    def check_lap_completion(self):
        dist_to_start = np.linalg.norm(self.state[:2] - self.centerline[0, :2])
        
        # Only trigger if close to start AND enough steps have passed (prevents double-triggering)
        if dist_to_start < LAP_THRESHOLD and len(self.current_lap_data) > MIN_LAP_STEPS:
            # Calculate Time
            current_time = rospy.get_time()
            lap_duration = current_time - self.lap_start_time
            
            self.lap_count += 1
            
            # Print to Terminal
            print("\n" + "*"*30)
            print(f"🏁 LAP {self.lap_count} FINISHED!")
            print(f"⏱️  Time: {lap_duration:.2f} seconds")
            print(f"🚀 New Target Speed: {self.current_target_speed + 0.15:.2f} m/s")
            print("*"*30 + "\n")
            
            # --- Learning Update ---
            # Save the successful path we just took
            self.safe_set = np.array(self.current_lap_data)[::2] 
            
            # --- Reset for next lap ---
            self.current_lap_data = []
            self.lap_start_time = current_time # Reset the clock for the new lap
            self.current_target_speed = min(self.current_target_speed + 0.15, MAX_SPEED)

    def control_loop(self, event):
        if not self.odom_received: return

        self.current_lap_data.append(self.state.copy())
        self.check_lap_completion()

        # --- 1. DYNAMIC HORIZON ---
        # Reduce horizon as speed increases to avoid "looking" too far past corners
        # Distance = v * horizon * DT. We want to keep this distance relatively stable.
        current_horizon = int(max(10, BASE_HORIZON - (self.current_target_speed * 3)))

        # --- 2. REFERENCE ---
        dists = np.linalg.norm(self.centerline[:, :2] - self.state[:2], axis=1)
        nearest_idx = np.argmin(dists)
        
        ref = []
        for i in range(current_horizon):
            idx = (nearest_idx + i + 1) % len(self.centerline)
            p = self.centerline[idx].copy()
            p[3] = self.normalize_angle(p[3], self.state[3])
            ref.append(p)
        ref = np.array(ref)

        # --- 3. TERMINAL TARGET (The "Learning" part) ---
        ss_dists = np.linalg.norm(self.safe_set[:, :2] - self.state[:2], axis=1)
        # Terminal target should be slightly further than current horizon
        ss_idx = (np.argmin(ss_dists) + current_horizon + 5) % len(self.safe_set)
        terminal_target = self.safe_set[ss_idx].copy()
        terminal_target[3] = self.normalize_angle(terminal_target[3], self.state[3])

        # --- 4. OPTIMIZATION ---
        x = cp.Variable((4, current_horizon + 1))
        u = cp.Variable((2, current_horizon))
        cost = 0
        constraints = [x[:, 0] == self.state]

        # Dynamic Scaling Factor: Increase tracking cost as we go faster
        speed_factor = (self.current_target_speed / BASE_SPEED)

        for t in range(current_horizon):
            # Kinematics
            constraints.append(x[0, t+1] == x[0, t] + x[2, t] * np.cos(ref[t, 3]) * DT)
            constraints.append(x[1, t+1] == x[1, t] + x[2, t] * np.sin(ref[t, 3]) * DT)
            constraints.append(x[2, t+1] == x[2, t] + u[0, t] * DT)
            constraints.append(x[3, t+1] == x[3, t] + u[1, t] * DT)

            # Cost Weights
            w = 1.0 / (t + 1)
            # Increase weight on staying on line as speed goes up
            cost += (3500 * speed_factor * w) * cp.sum_squares(x[0:2, t+1] - ref[t, 0:2])
            cost += (800 * w) * cp.sum_squares(x[3, t+1] - ref[t, 3])
            cost += 150 * cp.sum_squares(x[2, t+1] - self.current_target_speed)
            cost += 5 * cp.sum_squares(u[:, t]) # Penalize jerky movements

        # Terminal weight
        cost += 4000 * cp.sum_squares(x[0:2, current_horizon] - terminal_target[0:2])
        
        # Constraints: u[0] is acceleration, u[1] is yaw rate
        constraints += [cp.abs(u[0, :]) <= 0.8, cp.abs(u[1, :]) <= 2.5]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, verbose=False, warm_start=True)

        if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            cmd = Twist()
            # Calculate next velocity based on optimized acceleration
            v_next = self.state[2] + u[0, 0].value * DT
            cmd.linear.x = np.clip(v_next, 0, MAX_SPEED)
            cmd.angular.z = u[1, 0].value
            self.drive_pub.publish(cmd)
        else:
            self.drive_pub.publish(Twist()) # Safety stop

if __name__ == '__main__':
    try:
        LMPCController()
        rospy.spin()
    except rospy.ROSInterruptException: pass
