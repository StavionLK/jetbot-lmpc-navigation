#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf.transformations import euler_from_quaternion

ROBOT_NAME = "ackermann_vehicle"

pub = rospy.Publisher("/odom", Odometry, queue_size=1)

def cb(msg):
    if ROBOT_NAME not in msg.name:
        return
    
    idx = msg.name.index(ROBOT_NAME)
    pose = msg.pose[idx]
    twist = msg.twist[idx]

    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    odom.pose.pose = pose
    odom.twist.twist = twist

    pub.publish(odom)

rospy.init_node("gazebo_odom_bridge")
rospy.Subscriber("/gazebo/model_states", ModelStates, cb)
rospy.spin()
