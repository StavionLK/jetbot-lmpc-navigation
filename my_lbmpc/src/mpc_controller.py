#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf

# ------------------------------
# PARAMETERS
# ------------------------------
LOOKAHEAD = 0.3   # meters ahead to track
MAX_SPEED = 0.22
MAX_ANGULAR = 2.0
K_ANG = 2.0       # angular gain

# ------------------------------
# LOAD CSV PATH USING NUMPY
# ------------------------------
ref_path = np.loadtxt("/home/chamith/catkin_ws/src/my_lbmpc/src/ref_path.csv", delimiter=",")

# Filter any 0,0 points
ref_path = ref_path[(np.abs(ref_path[:,0]) > 0.01) | (np.abs(ref_path[:,1]) > 0.01)]

# ------------------------------
# ROBOT STATE
# ------------------------------
state = np.zeros(3)  # x, y, yaw

def odom_callback(msg):
    global state
    state[0] = msg.pose.pose.position.x
    state[1] = msg.pose.pose.position.y
    
    q = msg.pose.pose.orientation
    yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    state[2] = yaw

# ------------------------------
# ROS NODE
# ------------------------------
rospy.init_node("mpc_controller_simple_numpy")
rospy.Subscriber("/odom", Odometry, odom_callback)
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rate = rospy.Rate(10)

# ------------------------------
# UTILITY FUNCTIONS
# ------------------------------
def find_target(state, ref_path, lookahead):
    x, y = state[0], state[1]
    dists = np.sqrt((ref_path[:,0]-x)**2 + (ref_path[:,1]-y)**2)
    for i in range(len(dists)):
        if dists[i] > lookahead:
            return ref_path[i]
    return ref_path[-1]

def compute_control(state, target):
    x, y, yaw = state
    dx = target[0] - x
    dy = target[1] - y
    alpha = np.arctan2(dy, dx)
    
    e_theta = alpha - yaw
    e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))  # normalize
    
    v = MAX_SPEED
    w = K_ANG * e_theta
    w = np.clip(w, -MAX_ANGULAR, MAX_ANGULAR)
    return v, w

# ------------------------------
# MAIN LOOP
# ------------------------------
while not rospy.is_shutdown():
    target = find_target(state, ref_path, LOOKAHEAD)
    v, w = compute_control(state, target)
    
    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w
    cmd_pub.publish(cmd)
    
    rate.sleep()
