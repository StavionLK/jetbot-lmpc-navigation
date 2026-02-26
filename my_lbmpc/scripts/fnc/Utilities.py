import numpy as np
import pdb
import datetime

def Regression(x, u, lamb):
    """Estimates linear system dynamics
    x, u: date used in the regression
    lamb: regularization coefficient
    """

    # Want to solve W^* = argmin sum_i ||W^T z_i - y_i ||_2^2 + lamb ||W||_F,
    # with z_i = [x_i u_i] and W \in R^{n + d} x n
    Y = x[2:x.shape[0], :]
    X = np.hstack((x[1:(x.shape[0] - 1), :], u[1:(x.shape[0] - 1), :]))

    Q = np.linalg.inv(np.dot(X.T, X) + lamb * np.eye(X.shape[1]))
    b = np.dot(X.T, Y)
    W = np.dot(Q, b)

    A = W.T[:, 0:6]
    B = W.T[:, 6:8]

    ErrorMatrix = np.dot(X, W) - Y
    ErrorMax = np.max(ErrorMatrix, axis=0)
    ErrorMin = np.min(ErrorMatrix, axis=0)
    Error = np.vstack((ErrorMax, ErrorMin))

    return A, B, Error


def wrap(angle):
    if angle < -np.pi:
        w_angle = 2 * np.pi + angle
    elif angle > np.pi:
        w_angle = angle - 2 * np.pi
    else:
        w_angle = angle

    return w_angle

# class PID:
#     """Create the PID controller used for path following at constant speed
#     Attributes:
#         solve: given x0 computes the control action
#     """
#     def __init__(self, vt):
#         """Initialization
#         Arguments:
#             vt: target velocity
#         """
#         self.vt = vt
#         self.uPred = np.zeros([1,2])

#         startTimer = datetime.datetime.now()
#         endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
#         self.solverTime = deltaTimer
#         self.linearizationTime = deltaTimer
#         self.feasible = 1

#         # ===== NEW TUNED GAINS (smooth + stable) =====
#         self.k_ey   = -0.25     # lateral error
#         self.k_epsi = -0.50     # heading error
#         self.k_v    =  0.80     # speed control

#         # ===== Saturation limits =====
#         self.max_delta = 0.35          # 20 degrees
#         self.max_acc   = 0.60          # m/s^2 forward
#         self.min_acc   = -0.80         # m/s^2 braking

#     def solve(self, x0):
#         """Computes control action
#         Arguments:
#             x0: current state position
#             x0 = [vx, vy, wz, epsi, s, ey]
#         """

#         vx   = float(x0[0])
#         epsi = float(x0[3])
#         ey   = float(x0[5])

#         # ===============================  
#         # Steering PID  (NO RANDOM NOISE)
#         # ===============================
#         delta = self.k_ey * ey + self.k_epsi * epsi
#         delta = float(np.clip(delta, -self.max_delta, self.max_delta))

#         # ===============================  
#         # Longitudinal PID → acceleration
#         # ===============================
#         v_error = self.vt - vx
#         a = float(self.k_v * v_error)
#         a = float(np.clip(a, self.min_acc, self.max_acc))

#         # ===============================
#         # Final output
#         # ===============================
#         self.uPred[0, 0] = delta
#         self.uPred[0, 1] = a
#         return self.uPred
class PID:
    """
    Stable path-following PID controller.
    Outputs:
        u = [delta, v_cmd]
            delta = steering angle (rad)
            v_cmd = absolute target speed (m/s)
    """

    def __init__(self, vt):
        self.vt = vt
        self.uPred = np.zeros([1,2])

        # === Strong stable gains ===
        self.k_ey   = -1.20     # lateral error correction
        self.k_epsi = -1.80     # heading correction
        self.k_v    =  0.40     # speed correction (small)

        # Limits
        self.max_delta = 0.35          # rad  (~20°)
        self.min_speed = 0.20
        self.max_speed = 1.50

    def solve(self, x0):
        """
        x0 = [vx_tr, vy_tr, wz, epsi, s, ey]
        """

        vx   = float(x0[0])
        epsi = float(x0[3])
        ey   = float(x0[5])

        # -----------------------------
        # Steering
        # -----------------------------
        delta = self.k_ey * ey + self.k_epsi * epsi
        delta = float(np.clip(delta, -self.max_delta, self.max_delta))

        # -----------------------------
        # Speed control (ABSOLUTE!)
        # -----------------------------
        v_error = self.vt - vx
        v_cmd = self.vt + self.k_v * v_error

        v_cmd = float(np.clip(v_cmd, self.min_speed, self.max_speed))

        # -----------------------------
        # Final control
        # -----------------------------
        self.uPred[0, 0] = delta
        self.uPred[0, 1] = v_cmd
        return self.uPred
