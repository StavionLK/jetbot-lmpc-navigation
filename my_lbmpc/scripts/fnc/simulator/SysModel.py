#!/usr/bin/env python3
import rospy
import numpy as np
import math
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetPhysicsProperties, SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from race.msg import drive_param

def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

class RosGazeboSimulator:
    """
    ROS/Gazebo-compatible simulator that maps Gazebo odometry into LMPC states.
    Usage:
      simulator = RosGazeboSimulator(map, dt=0.1, multiLap=False, flagLMPC=False,
                                     odom_topic="/odom", cmd_vel_topic="/cmd_vel")
      x_cl, u_cl, x_cl_glob, xF = simulator.sim([None,None], Controller, maxSimTime=100.0)
    Requirements:
      - self.map must implement getLocalPosition(x,y,psi) -> (s_cont, ey, epsi, flag)
        and getAngle(s, epsi) -> psi_ref, TrackLength, etc. Your Track.Map already does.
    """

    def __init__(self, map_obj,
                 dt=0.1,
                 multiLap=False,
                 flagLMPC=False,
                 odom_topic="/odom",
                 cmd_vel_topic="/cmd_vel",
                 max_speed=2,
                 max_yaw_rate=1.5,
                 backward_jump_threshold=0.5):
        self.map = map_obj
        self.dt = float(dt)
        self.multiLap = bool(multiLap)
        self.flagLMPC = bool(flagLMPC)

        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic

        self.max_speed = float(max_speed)
        self.max_yaw_rate = float(max_yaw_rate)

        # internal state
        self._latest_odom = None
        self.current_speed = 0.0
        self.s_history = []                  # keep last s values for monotonicity checks
        self.backward_jump_threshold = float(backward_jump_threshold)

        # ROS topics (publisher/subscriber)
        # Note: safe to create even if node isn't fully up; check for rospy.init_node before using
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=50)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.pub_ack = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=10)

        # Gazebo services (may raise if not available; user code should handle exceptions)
        self.reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)

    # ---------------------- ROS Callbacks ----------------------
    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg

    def publish_ackermann(self, delta, v):
        msg = AckermannDrive()

        # Use LMPC's steering angle directly
        msg.steering_angle = float(np.clip(delta, -0.6, 0.6))   # radians

        # Speed in m/s (NO /50 or /100)
        msg.speed = float(np.clip(v, 0.0, self.max_speed))

        self.pub_ack.publish(msg)



    def _wait_for_odom(self, timeout=5.0):
        start = rospy.Time.now()
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self._latest_odom is not None:
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                return False
            r.sleep()

    # ---------------------- Reset Function ----------------------
    def reset(self, model_name="turtlebot3_burger",
              x=0.0, y=0.0, yaw=0.0):
        """Reset Gazebo robot pose using /gazebo/set_model_state (best-effort)."""
        try:
            self.unpause_physics()
        except Exception:
            rospy.logwarn("Could not unpause Gazebo; continuing without unpause.")

        quat = tft.quaternion_from_euler(0, 0, yaw)
        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = 0.0
        state.pose.orientation.x = quat[0]
        state.pose.orientation.y = quat[1]
        state.pose.orientation.z = quat[2]
        state.pose.orientation.w = quat[3]
        state.twist.linear.x = 0.0
        state.twist.angular.z = 0.0
        state.reference_frame = "world"

        try:
            self.set_model_state(state)
        except Exception as e:
            rospy.logerr(f"[RosGazeboSimulator.reset] Failed to set_model_state: {e}")

        self.current_speed = 0.0
        rospy.sleep(1.0)

    # ---------------------- Helpers ----------------------
    def _odom_to_states(self, odom_msg):
        """
        Convert Odometry message to LMPC states:
          x_local = [vx, vy, wz, epsi, s, ey]
          x_glob  = length-6 array where idx3=psi, idx4=X, idx5=Y to match original LMPC code expectations.
        Returns x_local (np.array) and x_glob (np.array).
        """
        px = float(odom_msg.pose.pose.position.x)
        py = float(odom_msg.pose.pose.position.y)
        q = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = wrap_angle(float(yaw))

        # Body-frame velocities (many odom providers set linear.x; linear.y may be zero)
        vx_body = float(odom_msg.twist.twist.linear.x) if hasattr(odom_msg.twist.twist, 'linear') else 0.0
        vy_body = float(odom_msg.twist.twist.linear.y) if hasattr(odom_msg.twist.twist, 'linear') else 0.0
        wz = float(odom_msg.twist.twist.angular.z) if hasattr(odom_msg.twist.twist, 'angular') else 0.0

        # Convert to world-frame velocity (body->world)
        vwx = math.cos(yaw) * vx_body - math.sin(yaw) * vy_body
        vwy = math.sin(yaw) * vx_body + math.cos(yaw) * vy_body

        # Project robot pose into track coordinates
        # getLocalPosition returns (s_cont, ey, epsi, True/False)
        s_proj, ey_proj, epsi_proj, flag = self.map.getLocalPosition(px, py, yaw)
        

        # Compute reference tangent angle at projected s
        psi_ref = self.map.getAngle(s_proj, 0.0)

        # Rotate world velocity into track-aligned frame (velocity in track frame = R(-psi_ref) * v_world)
        cosr = math.cos(-psi_ref)
        sinr = math.sin(-psi_ref)
        vx_tr = cosr * vwx - sinr * vwy
        vy_tr = sinr * vwx + cosr * vwy

        # If projection failed (flag False) or returned an obviously bad s, fallback to integrating s
        bad_proj = (not bool(flag)) or (s_proj is None) or (abs(float(s_proj)) > 1e6)
        if bad_proj:
            prev_s = self.s_history[-1] if len(self.s_history) > 0 else 0.0
            ds = max(-0.5, min(0.5, vx_tr * self.dt))  # small step safeguard
            s = prev_s + ds
            ey = ey_proj if flag else 0.0
            epsi = epsi_proj if flag else wrap_angle(yaw - psi_ref)
            # print("Warning: Bad projection detected; integrating s instead.")
        else:
            s = float(s_proj)
            ey = float(ey_proj)
            epsi = float(epsi_proj)
        
        # Monotonicity protection: prevent large backward jumps due to projection noise
        # print("Projection s_proj before:", s)
        # print("s_history:", self.s_history)
        # if len(self.s_history) > 0:
        #     prev_s = self.s_history[-1]
        #     if s < prev_s - self.backward_jump_threshold:
        #         # ignore sudden backward jump and instead integrate with vx_tr (small positive progress)
        #         s = prev_s + max(-self.backward_jump_threshold,
                                #  min(self.backward_jump_threshold, vx_tr * self.dt))
        if s < 0.0:
            s = 0.0
        # print("Projection s_proj:", s)
        self.s_history.append(s)

        # Build LMPC-compatible arrays
        x_local = np.array([vx_tr, vy_tr, wz, epsi, s, ey], dtype=float)

        # x_glob layout (to match original code expectations in plotting & dynModel)
        x_glob = np.zeros(6, dtype=float)
        x_glob[3] = yaw
        x_glob[4] = px
        x_glob[5] = py
        # print("[Sys DEBUG]",
        # "vx=%.2f" % vx_tr,
        # "s=%.2f" % s_proj,
        # "ey=%.2f" % ey_proj,
        # "epsi=%.2f" % epsi_proj)
        return x_local, x_glob

    def _publish_control(self, u):
        # u = [steering, acceleration]
        delta = float(u[0])
        a     = float(u[1])

        # integrate acceleration to get speed
        self.current_speed += a * self.dt
        self.current_speed = float(np.clip(self.current_speed, 0.0, self.max_speed))

        # send command to ackermann controller
        self.publish_ackermann(delta, self.current_speed)



    # ---------------------- Main Simulation ----------------------
    def sim(self, x0, Controller, maxSimTime=100.0):
        """
        Drive closed-loop in Gazebo using provided Controller object.
        Controller must implement:
          - solve(x_current)  (sets Controller.uPred)
          - Controller.uPred must be shape (N, d) (here we read [0,:])
          - optionally addPoint(x,u) if flagLMPC True
        Returns: x_cl (Nx6), u_cl (Nx2), x_cl_glob (Nx6), xF
        """
        if not self._wait_for_odom():
            raise RuntimeError("[RosGazeboSimulator.sim] No /odom received from Gazebo.")

        # initialize from current odom
        odom0 = self._latest_odom
        x0_local, x0_glob = self._odom_to_states(odom0)

        print("\n================ DEBUG INFO ================")
        print("Robot spawn position (odom):",
            odom0.pose.pose.position.x,
            odom0.pose.pose.position.y)

        print("Initial projected s =", x0_local[4])
        print("Initial ey =", x0_local[5])
        print("Initial epsi =", x0_local[3])

        print("Track START  =", self.map.x_map[0], self.map.y_map[0])
        print("Track END    =", self.map.x_map[-1], self.map.y_map[-1])
        print("Track Length =", self.map.TrackLength)
        print("============================================\n")


        x_cl = [x0_local]
        x_cl_glob = [x0_glob]
        u_cl = []

        s_start = float(x0_local[4])
        track_len = float(self.map.TrackLength)

        rate = rospy.Rate(int(1.0 / self.dt) if self.dt > 0 else 10)
        start_time = rospy.Time.now()
        flagExt = False
        prv_s = 0
        while not rospy.is_shutdown() and not flagExt:
            # controller computes uPred based on last local state
            Controller.solve(x_cl[-1])
            u = Controller.uPred[0, :].copy()
            u_cl.append(u)

            # publish into Gazebo
            self._publish_control(u)
            rate.sleep()

            if self._latest_odom is None:
                continue

            # read odom and convert to LMPC local state
            x_local, x_glob = self._odom_to_states(self._latest_odom)
            x_cl.append(x_local)
            x_cl_glob.append(x_glob)

            # LMPC incremental safe-set addition
            if self.flagLMPC and hasattr(Controller, "addPoint"):
                Controller.addPoint(x_local, u)

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > float(maxSimTime):
                rospy.loginfo("Max sim time reached.")
                flagExt = True
                break

            progress_this_run = float(x_local[4]) #- s_start
            # print("s_value : ",x_local[4] , " progress_this_run: ", progress_this_run, " s_start: ", s_start)
            # print("progress_this_run: ", x_local[4], " s_start: ",  s_start)
            if (not self.multiLap) and((prv_s-progress_this_run)>20 or(progress_this_run > track_len)):
                rospy.loginfo("Lap complete.")
                rospy.loginfo(f"s_value={x_local[4]:.2f}, TrackLength={self.map.TrackLength:.2f}")
                flagExt = True
                break

            if self.multiLap and ((prv_s-progress_this_run)>20 or(progress_this_run > track_len)):
                rospy.loginfo("Lap done — wrapping s for multiLap.")
                x_local[4] -= self.map.TrackLength
                flagExt = True
            prv_s = x_local[4]
        # prepare final return values (mimic original LMPC interface)
        x_last = np.array(x_cl[-1])
        x_last_wrapped = x_last.copy()
        x_last_wrapped[4] -= self.map.TrackLength
        xF = [x_last_wrapped, np.array(x_cl_glob[-1])]

        if len(x_cl) > 0:
            x_cl.pop()
            x_cl_glob.pop()

        return np.array(x_cl), np.array(u_cl), np.array(x_cl_glob), xF
