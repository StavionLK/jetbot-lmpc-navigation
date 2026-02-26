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
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
FILENAME = os.path.join(SCRIPT_DIR, 'ref_path.csv')

# MPC Parameters
HORIZON = 5                # Number of steps to look ahead
DT = 0.1                    # Time step (seconds)
MAX_STEER = 0.5             # Max angular velocity (rad/s)
MAX_ACCEL = 0.3             # Max acceleration (m/s^2)
BASE_SPEED = 1.0            # Target speed (m/s)

class MPCController:
    def __init__(self):
        rospy.init_node('mpc_controller')
        
        # Load waypoints (x, y, yaw)
        self.waypoints = self.load_waypoints(FILENAME)
        if self.waypoints is None:
            return

        # --- LAP TIMER & SPEED SETUP ---
        self.start_pos = None
        self.lap_start_time = None
        self.lap_count = 0
        self.min_lap_time = 15.0    # Minimum time before a lap can end
        self.lap_threshold = 0.8     # Distance to start point to trigger lap
        self.velocities = []         # To calculate average speed
        # -------------------------------

        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_cb)
        
        self.state = np.zeros(4) # [x, y, v, yaw]
        self.odom_received = False
        
        rospy.Timer(rospy.Duration(DT), self.control_loop)
        rospy.loginfo("MPC Controller Initialized with Lap & Speed Tracking.")

    def load_waypoints(self, path):
        try:
            points = []
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    points.append([
                        float(row['x']), 
                        float(row['y']), 
                        float(row['yaw'])
                    ])
            rospy.loginfo(f"Loaded {len(points)} waypoints from {path}")
            return np.array(points)
        except Exception as e:
            rospy.logerr(f"CSV Loading Error: {e}. Ensure headers are 'x,y,yaw'")
            return None

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Compute magnitude of linear velocity
        v = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        
        # Convert orientation to Euler yaw
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.state = np.array([x, y, v, yaw])
        self.odom_received = True

        # --- LAP TIMING & SPEED LOGIC ---
        if self.start_pos is None:
            self.start_pos = (x, y)
            self.lap_start_time = rospy.get_time()
            return

        # Record speed for averaging
        if self.lap_start_time is not None:
            self.velocities.append(v)

        # Calculate distance to start point
        dist_to_start = np.sqrt((x - self.start_pos[0])**2 + (y - self.start_pos[1])**2)
        current_time = rospy.get_time()
        
        if self.lap_start_time is not None:
            elapsed = current_time - self.lap_start_time
            if dist_to_start < self.lap_threshold and elapsed > self.min_lap_time:
                self.lap_count += 1
                avg_speed = sum(self.velocities) / len(self.velocities) if self.velocities else 0.0
                
                print("\n" + "="*35)
                print(f"🏁 MPC LAP {self.lap_count} COMPLETE")
                print(f"⏱️  Lap Time: {elapsed:.2f} seconds")
                print(f"🚀 Avg Speed: {avg_speed:.2f} m/s")
                print("="*35 + "\n")
                
                # Reset for next lap
                self.lap_start_time = current_time
                self.velocities = []
        # --------------------------------

    def get_linear_model(self, v_ref, yaw_ref):
        """Linearized unicycle model state-space matrices."""
        A = np.eye(4)
        A[0, 2] = np.cos(yaw_ref) * DT
        A[0, 3] = -v_ref * np.sin(yaw_ref) * DT
        A[1, 2] = np.sin(yaw_ref) * DT
        A[1, 3] = v_ref * np.cos(yaw_ref) * DT
        
        B = np.zeros((4, 2))
        B[2, 0] = DT    
        B[3, 1] = DT    
        return A, B

    def control_loop(self, event):
        if not self.odom_received:
            return

        # 1. Find nearest waypoint index
        dists = np.linalg.norm(self.waypoints[:, :2] - self.state[:2], axis=1)
        nearest_idx = np.argmin(dists)
        
        # 2. Extract Reference Horizon [x, y, yaw]
        ref = []
        for i in range(HORIZON):
            idx = (nearest_idx + i + 1) % len(self.waypoints)
            ref.append(self.waypoints[idx])
        ref = np.array(ref)

        # 3. Setup CVXPY Optimization
        x = cp.Variable((4, HORIZON + 1))
        u = cp.Variable((2, HORIZON))
        cost = 0
        constraints = [x[:, 0] == self.state]

        for t in range(HORIZON):
            v_ref = BASE_SPEED
            yaw_ref = ref[t, 2]
            
            A, B = self.get_linear_model(v_ref, yaw_ref)
            constraints.append(x[:, t+1] == A @ x[:, t] + B @ u[:, t])
            
            cost += cp.sum_squares(x[0:2, t+1] - ref[t, 0:2]) * 1000 
            cost += cp.sum_squares(x[3, t+1] - ref[t, 2]) * 400
            cost += cp.sum_squares(x[2, t+1] - BASE_SPEED) * 100
            cost += cp.sum_squares(u[1, t]) * 10

            constraints.append(cp.abs(u[0, t]) <= MAX_ACCEL)
            constraints.append(cp.abs(u[1, t]) <= MAX_STEER)

        # 4. Solve Optimization
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, verbose=False)

        if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            accel, omega = u[:, 0].value
            
            cmd = Twist()
            cmd.linear.x = np.clip(self.state[2] + accel * DT, 0.0, 1.0)
            cmd.angular.z = omega 
            self.drive_pub.publish(cmd)
        else:
            rospy.logwarn("MPC Solver Failed. Stopping.")
            self.drive_pub.publish(Twist())

if __name__ == '__main__':
    try:
        MPCController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
