
#!/usr/bin/env python3
# import numpy as np
# import pandas as pd
# import rospy
# from nav_msgs.msg import Odometry, Path
# from geometry_msgs.msg import PoseStamped
# from tf.transformations import euler_from_quaternion
# import math

# def wrap_angle(a):
#     """Keep angle within [-π, π]."""
#     return (a + np.pi) % (2 * np.pi) - np.pi

# class Map:
#     """
#     CSV-based path map for LMPC / ROS:
#       - Loads path from CSV with columns: x, y [, yaw]
#       - Continuous progress s with unwrapping & filtering
#       - Robust curvature/angle and (s,ey,epsi)<->(X,Y) transforms
#       - Publishes nav_msgs/Path for RViz
#       - Optionally consumes /odom to compute robot local pose
#     """

#     def __init__(self, csv_path, half_width=0.5, slack=0.45, frame_id="map"):
#         self.halfWidth = float(half_width)
#         self.slack = float(slack)
#         self.frame_id = frame_id

#         # ---------- Load CSV ----------
#         df = pd.read_csv(csv_path)
#         if not {"x", "y"}.issubset(df.columns):
#             raise ValueError("CSV must have at least columns: x, y")
#         self.x_map = df["x"].to_numpy(dtype=float)
#         self.y_map = df["y"].to_numpy(dtype=float)

#         if "yaw" in df.columns:
#             self.psi_map = df["yaw"].to_numpy(dtype=float)
#         else:
#             # Estimate yaw via gradient
#             dx_tmp = np.gradient(self.x_map)
#             dy_tmp = np.gradient(self.y_map)
#             self.psi_map = np.arctan2(dy_tmp, dx_tmp)

#         # ---------- Arc length s_map & track length ----------
#         # seg_dx = np.diff(self.x_map)
#         # seg_dy = np.diff(self.y_map)
#         # ds = np.hypot(seg_dx, seg_dy)
#         # self.s_map = np.insert(np.cumsum(ds), 0, 0.0)
#         # self.TrackLength = float(self.s_map[-1])
#         # if self.TrackLength <= 0.0:
#         #     raise ValueError("Track length must be positive")
#         # ---------- Arc length s_map & track length ----------
#         seg_dx = np.diff(self.x_map)
#         seg_dy = np.diff(self.y_map)
#         ds = np.hypot(seg_dx, seg_dy)
#         self.s_map = np.insert(np.cumsum(ds), 0, 0.0)

#         # Normalize so the first CSV point has s = 0 (prevents the 0↔L jump)
#         # self.s_map -= float(self.s_map[0])

#         # Now set TrackLength (last entry after normalization)
#         self.TrackLength = float(self.s_map[-1])
#         if self.TrackLength <= 0.0:
#             raise ValueError("Track length must be positive")


#         # ---------- Precompute gradients for curvature ----------
#         # Use smooth gradients once; then reuse in curvature()
#         self._dx = np.gradient(self.x_map)
#         self._dy = np.gradient(self.y_map)
#         self._ddx = np.gradient(self._dx)
#         self._ddy = np.gradient(self._dy)
#         self._curvatures = self._compute_curvature_array()

#         # ---------- ROS I/O ----------
#         self.robot_pose = None
#         self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=50)
#         self.path_pub = rospy.Publisher("/map_path", Path, queue_size=1, latch=True)

#         # Publish path for RViz (once latched)
#         rospy.sleep(0.2)
#         self.publish_path()

#         # ---------- Progress continuity state ----------
#         self._s_prev = 0.0
#         self._initialized_progress = False

#         rospy.loginfo(f"[Map] Loaded {len(self.x_map)} points from '{csv_path}', length={self.TrackLength:.2f} m")

#     # --------------------------------------------------------------------------
#     # ROS callbacks and publishers
#     # --------------------------------------------------------------------------
#     def _odom_cb(self, msg: Odometry):
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
#         self.robot_pose = (float(x), float(y), float(yaw))

#     def publish_path(self):
#         """Publish the loaded path as nav_msgs/Path for RViz."""
#         path_msg = Path()
#         path_msg.header.frame_id = self.frame_id
#         path_msg.header.stamp = rospy.Time.now()
#         for xi, yi, psi in zip(self.x_map, self.y_map, self.psi_map):
#             pose = PoseStamped()
#             pose.header.frame_id = self.frame_id
#             pose.header.stamp = path_msg.header.stamp
#             pose.pose.position.x = float(xi)
#             pose.pose.position.y = float(yi)
#             pose.pose.position.z = 0.0
#             # yaw -> quaternion z,w
#             pose.pose.orientation.z = np.sin(psi / 2.0)
#             pose.pose.orientation.w = np.cos(psi / 2.0)
#             path_msg.poses.append(pose)
#         self.path_pub.publish(path_msg)

#     # --------------------------------------------------------------------------
#     # Geometry and mapping utilities
#     # --------------------------------------------------------------------------
#     def _compute_curvature_array(self):
#         """Discrete curvature from gradients; stabilized at ends."""
#         denom = (self._dx**2 + self._dy**2)**1.5
#         kappa = np.zeros_like(self._dx)
#         good = denom > 1e-9
#         kappa[good] = (self._dx[good]*self._ddy[good] - self._dy[good]*self._ddx[good]) / denom[good]
#         # edge stabilization
#         if len(kappa) >= 3:
#             kappa[0] = kappa[1]
#             kappa[-1] = kappa[-2]
#         return kappa

#     # def curvature(self, s):
#     #     """Curvature κ(s). Lookup uses wrapped s in [0, L)."""
#     #     s_wrapped = float(s) % self.TrackLength
#     #     return float(np.interp(s_wrapped, self.s_map, self._curvatures))
#     def curvature(self, s):
#         """Curvature κ(s). Lookup uses wrapped s in [0, L). Clamped for robustness."""
#         s_wrapped = float(s) % self.TrackLength
#         kappa = float(np.interp(s_wrapped, self.s_map, self._curvatures))

#         # 🔒 Clamp curvature to avoid insane values from noisy CSV
#         kappa_max = 2.0   # [1/m] → minimum radius ≈ 0.5 m. Adjust if needed.
#         if kappa > kappa_max:
#             kappa = kappa_max
#         elif kappa < -kappa_max:
#             kappa = -kappa_max

#         return kappa


#     def getAngle(self, s, epsi=0.0):
#         """Reference heading ψ_ref(s) + epsi, lookup with wrapped s."""
#         s_wrapped = float(s) % self.TrackLength
#         psi_ref = float(np.interp(s_wrapped, self.s_map, self.psi_map))
#         return wrap_angle(psi_ref + float(epsi))

#     def getGlobalPosition(self, s, ey):
#         """
#         Convert (s, ey) to global (X, Y).
#         s is unwrapped but lookup is performed modulo TrackLength.
#         """
#         s_wrapped = float(s) % self.TrackLength
#         x_center = float(np.interp(s_wrapped, self.s_map, self.x_map))
#         y_center = float(np.interp(s_wrapped, self.s_map, self.y_map))
#         psi_center = float(np.interp(s_wrapped, self.s_map, self.psi_map))
#         X = x_center - float(ey) * math.sin(psi_center)
#         Y = y_center + float(ey) * math.cos(psi_center)
#         return X, Y

#     def getLocalPosition(self, x, y, psi):
#         """
#         Project global (x, y, psi) onto path → (s_cont, ey, epsi, True).
#         - s_cont is continuous (unwrapped)
#         - ey is signed lateral deviation
#         - epsi is heading error in [-pi, pi]
#         """
#         X = float(x); Y = float(y); PSI = float(psi)

#         # Find nearest segment by midpoint distance (efficient & stable)
#         mids = 0.5 * (np.column_stack((self.x_map, self.y_map))[1:] +
#                       np.column_stack((self.x_map, self.y_map))[:-1])
#         dist_mid = np.hypot(mids[:, 0] - X, mids[:, 1] - Y)
#         i = int(np.argmin(dist_mid))
#         x1, y1 = self.x_map[i], self.y_map[i]
#         x2, y2 = self.x_map[i + 1], self.y_map[i + 1]
#         dx = x2 - x1; dy = y2 - y1
#         seg_len = math.hypot(dx, dy) + 1e-9  # avoid div by zero

#         # Projection factor on segment
#         t = ((X - x1) * dx + (Y - y1) * dy) / (seg_len**2)
#         t = float(np.clip(t, 0.0, 1.0))
#         x_proj = x1 + t * dx
#         y_proj = y1 + t * dy

#         # Signed lateral deviation (left is +)
#         normal = np.array([-dy, dx], dtype=float) / seg_len
#         ey = float(np.dot([X - x_proj, Y - y_proj], normal))

#         # Measured progress along path in [0, L)
#         s_meas = float(self.s_map[i] + t * seg_len)

#         # Initialize progress at first call with nearest s
#         if not self._initialized_progress:
#             self._s_prev = s_meas
#             self._initialized_progress = True

#         # Unwrap and filter continuous progress
#         s_cont = self.unwrap_s(s_meas)
#         s_cont = self.filter_progress(s_cont)

#         # Heading error relative to segment direction
#         psi_ref = math.atan2(dy, dx)
#         epsi = wrap_angle(PSI - psi_ref)

#         if s_cont < 0.0:
#             rospy.logwarn_throttle(5.0, f"[Map] Negative s={s_cont:.3f}, clamping to 0.0 (x={X:.2f}, y={Y:.2f})")
#             s_cont = 0.0
#         elif s_cont > self.TrackLength:
#             # wrap-around at finish line
#             s_cont = s_cont % self.TrackLength

# # Store for next iteration
#         self._s_prev = s_cont
#         return s_cont, ey, epsi, True

#     # --------------------------------------------------------------------------
#     # Progress helpers
#     # --------------------------------------------------------------------------
#     def unwrap_s(self, s_meas):
#         """
#         Keep s continuous across start/finish:
#         choose the 2π-equivalent (±TrackLength) closest to previous s.
#         """
#         L = self.TrackLength
#         k = round((self._s_prev - float(s_meas)) / L)
#         return float(s_meas + k * L)

#     def filter_progress(self, s_cont, max_backstep=0.05):
#         """
#         Prevent small backward jumps from odom noise.
#         Allows at most `max_backstep` meters back in one step.
#         """
#         if s_cont < self._s_prev - max_backstep:
#             s_cont = self._s_prev - max_backstep
#         return float(s_cont)

#     # --------------------------------------------------------------------------
#     # Robot pose convenience
#     # --------------------------------------------------------------------------
#     def wait_for_pose(self, timeout=5.0):
#         """Wait until /odom received; returns (x, y, yaw)."""
#         start = rospy.Time.now()
#         r = rospy.Rate(100)
#         while not rospy.is_shutdown():
#             if self.robot_pose is not None:
#                 return self.robot_pose
#             if (rospy.Time.now() - start).to_sec() > timeout:
#                 raise TimeoutError("No /odom received within timeout")
#             r.sleep()

#     def get_robot_local_pose(self):
#         """Compute (s, ey, epsi) using latest /odom."""
#         x, y, psi = self.wait_for_pose()
#         return self.getLocalPosition(x, y, psi)

#     def reset_progress(self, s0=0.0):
#         self._initialized_progress = False
#         self._s_prev = float(s0)


import numpy as np
import pandas as pd
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math

def wrap_angle(a):
    """Keep angle within [-π, π]."""
    return (a + np.pi) % (2 * np.pi) - np.pi

class Map:
    """
    CSV-based path map for LMPC / ROS:
      - Loads path from CSV with columns: x, y [, yaw]
      - Continuous progress s with unwrapping & filtering
      - Robust curvature/angle and (s,ey,epsi)<->(X,Y) transforms
      - Publishes nav_msgs/Path for RViz
      - Optionally consumes /odom to compute robot local pose
    """

    def __init__(self, csv_path, half_width=0.5, slack=0.45, frame_id="map"):
        self.halfWidth = float(half_width)
        self.slack = float(slack)
        self.frame_id = frame_id

        # ---------- Load CSV ----------
        df = pd.read_csv(csv_path)
        if not {"x", "y"}.issubset(df.columns):
            raise ValueError("CSV must have at least columns: x, y")
        self.x_map = df["x"].to_numpy(dtype=float)
        self.y_map = df["y"].to_numpy(dtype=float)

        if "yaw" in df.columns:
            self.psi_map = df["yaw"].to_numpy(dtype=float)
        else:
            # Estimate yaw via gradient
            dx_tmp = np.gradient(self.x_map)
            dy_tmp = np.gradient(self.y_map)
            self.psi_map = np.arctan2(dy_tmp, dx_tmp)

        # ---------- Arc length s_map & track length ----------
        # seg_dx = np.diff(self.x_map)
        # seg_dy = np.diff(self.y_map)
        # ds = np.hypot(seg_dx, seg_dy)
        # self.s_map = np.insert(np.cumsum(ds), 0, 0.0)
        # self.TrackLength = float(self.s_map[-1])
        # if self.TrackLength <= 0.0:
        #     raise ValueError("Track length must be positive")
        # ---------- Arc length s_map & track length ----------
        seg_dx = np.diff(self.x_map)
        seg_dy = np.diff(self.y_map)
        ds = np.hypot(seg_dx, seg_dy)
        self.s_map = np.insert(np.cumsum(ds), 0, 0.0)

        # Normalize so the first CSV point has s = 0 (prevents the 0↔L jump)
        # self.s_map -= float(self.s_map[0])

        # Now set TrackLength (last entry after normalization)
        self.TrackLength = float(self.s_map[-1])
        if self.TrackLength <= 0.0:
            raise ValueError("Track length must be positive")


        # ---------- Precompute gradients for curvature ----------
        # Use smooth gradients once; then reuse in curvature()
        self._dx = np.gradient(self.x_map)
        self._dy = np.gradient(self.y_map)
        self._ddx = np.gradient(self._dx)
        self._ddy = np.gradient(self._dy)
        self._curvatures = self._compute_curvature_array()

        # ---------- ROS I/O ----------
        self.robot_pose = None
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=50)
        self.path_pub = rospy.Publisher("/map_path", Path, queue_size=1, latch=True)

        # Publish path for RViz (once latched)
        rospy.sleep(0.2)
        self.publish_path()

        # ---------- Progress continuity state ----------
        self._s_prev = 0.0
        self._initialized_progress = False

        rospy.loginfo(f"[Map] Loaded {len(self.x_map)} points from '{csv_path}', length={self.TrackLength:.2f} m")

    # --------------------------------------------------------------------------
    # ROS callbacks and publishers
    # --------------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_pose = (float(x), float(y), float(yaw))

    def publish_path(self):
        """Publish the loaded path as nav_msgs/Path for RViz."""
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = rospy.Time.now()
        for xi, yi, psi in zip(self.x_map, self.y_map, self.psi_map):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(xi)
            pose.pose.position.y = float(yi)
            pose.pose.position.z = 0.0
            # yaw -> quaternion z,w
            pose.pose.orientation.z = np.sin(psi / 2.0)
            pose.pose.orientation.w = np.cos(psi / 2.0)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    # --------------------------------------------------------------------------
    # Geometry and mapping utilities
    # --------------------------------------------------------------------------
    def _compute_curvature_array(self):
        """Discrete curvature from gradients; stabilized at ends."""
        denom = (self._dx**2 + self._dy**2)**1.5
        kappa = np.zeros_like(self._dx)
        good = denom > 1e-9
        kappa[good] = (self._dx[good]*self._ddy[good] - self._dy[good]*self._ddx[good]) / denom[good]
        # edge stabilization
        if len(kappa) >= 3:
            kappa[0] = kappa[1]
            kappa[-1] = kappa[-2]
        return kappa

    def curvature(self, s):
        """Curvature κ(s). Lookup uses wrapped s in [0, L)."""
        s_wrapped = float(s) % self.TrackLength
        return float(np.interp(s_wrapped, self.s_map, self._curvatures))

    def getAngle(self, s, epsi=0.0):
        """Reference heading ψ_ref(s) + epsi, lookup with wrapped s."""
        s_wrapped = float(s) % self.TrackLength
        psi_ref = float(np.interp(s_wrapped, self.s_map, self.psi_map))
        return wrap_angle(psi_ref + float(epsi))

    def getGlobalPosition(self, s, ey):
        """
        Convert (s, ey) to global (X, Y).
        s is unwrapped but lookup is performed modulo TrackLength.
        """
        s_wrapped = float(s) % self.TrackLength
        x_center = float(np.interp(s_wrapped, self.s_map, self.x_map))
        y_center = float(np.interp(s_wrapped, self.s_map, self.y_map))
        psi_center = float(np.interp(s_wrapped, self.s_map, self.psi_map))
        X = x_center - float(ey) * math.sin(psi_center)
        Y = y_center + float(ey) * math.cos(psi_center)
        return X, Y

    def getLocalPosition(self, x, y, psi):
        """
        Project global (x, y, psi) onto path → (s_cont, ey, epsi, True).
        - s_cont is continuous (unwrapped)
        - ey is signed lateral deviation
        - epsi is heading error in [-pi, pi]
        """
        X = float(x); Y = float(y); PSI = float(psi)

        # Find nearest segment by midpoint distance (efficient & stable)
        mids = 0.5 * (np.column_stack((self.x_map, self.y_map))[1:] +
                      np.column_stack((self.x_map, self.y_map))[:-1])
        dist_mid = np.hypot(mids[:, 0] - X, mids[:, 1] - Y)
        i = int(np.argmin(dist_mid))
        x1, y1 = self.x_map[i], self.y_map[i]
        x2, y2 = self.x_map[i + 1], self.y_map[i + 1]
        dx = x2 - x1; dy = y2 - y1
        seg_len = math.hypot(dx, dy) + 1e-9  # avoid div by zero

        # Projection factor on segment
        t = ((X - x1) * dx + (Y - y1) * dy) / (seg_len**2)
        t = float(np.clip(t, 0.0, 1.0))
        x_proj = x1 + t * dx
        y_proj = y1 + t * dy

        # Signed lateral deviation (left is +)
        normal = np.array([-dy, dx], dtype=float) / seg_len
        ey = float(np.dot([X - x_proj, Y - y_proj], normal))

        # Measured progress along path in [0, L)
        s_meas = float(self.s_map[i] + t * seg_len)

        # Initialize progress at first call with nearest s
        if not self._initialized_progress:
            self._s_prev = s_meas
            self._initialized_progress = True

        # Unwrap and filter continuous progress
        s_cont = self.unwrap_s(s_meas)
        s_cont = self.filter_progress(s_cont)

        # Heading error relative to segment direction
        psi_ref = math.atan2(dy, dx)
        epsi = wrap_angle(PSI - psi_ref)

        # print(
        #     "[Map DEBUG]",
        #     "X=%.2f" % X,
        #     "Y=%.2f" % Y,
        #     "yaw=%.3f" % PSI,
        #     "yaw_ref=%.3f" % psi_ref,
        #     "epsi=%.3f" % epsi,
        #     "s=%.2f" % s_cont,
        #     "ey=%.2f" % ey,
        # )

        if s_cont < 0.0:
            rospy.logwarn_throttle(5.0, f"[Map] Negative s={s_cont:.3f}, clamping to 0.0 (x={X:.2f}, y={Y:.2f})")
            s_cont = 0.0
        elif s_cont > self.TrackLength:
            # wrap-around at finish line
            s_cont = s_cont % self.TrackLength

# Store for next iteration
        self._s_prev = s_cont
        return s_cont, ey, epsi, True

    # --------------------------------------------------------------------------
    # Progress helpers
    # --------------------------------------------------------------------------
    def unwrap_s(self, s_meas):
        """
        Keep s continuous across start/finish:
        choose the 2π-equivalent (±TrackLength) closest to previous s.
        """
        L = self.TrackLength
        k = round((self._s_prev - float(s_meas)) / L)
        return float(s_meas + k * L)

    def filter_progress(self, s_cont, max_backstep=0.05):
        """
        Prevent small backward jumps from odom noise.
        Allows at most `max_backstep` meters back in one step.
        """
        if s_cont < self._s_prev - max_backstep:
            s_cont = self._s_prev - max_backstep
        return float(s_cont)

    # --------------------------------------------------------------------------
    # Robot pose convenience
    # --------------------------------------------------------------------------
    def wait_for_pose(self, timeout=5.0):
        """Wait until /odom received; returns (x, y, yaw)."""
        start = rospy.Time.now()
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.robot_pose is not None:
                return self.robot_pose
            if (rospy.Time.now() - start).to_sec() > timeout:
                raise TimeoutError("No /odom received within timeout")
            r.sleep()

    def get_robot_local_pose(self):
        """Compute (s, ey, epsi) using latest /odom."""
        x, y, psi = self.wait_for_pose()
        return self.getLocalPosition(x, y, psi)

    def reset_progress(self, s0=0.0):
        self._initialized_progress = False
        self._s_prev = float(s0)
