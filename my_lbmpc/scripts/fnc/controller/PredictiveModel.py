from cvxopt import spmatrix, matrix, solvers
from numpy import linalg as la
from cvxopt.solvers import qp
import numpy as np
import datetime
import pdb
# This class is not generic and is tailored to the autonomous racing problem.
# The only method need the LT-MPC and the LMPC is regressionAndLinearization, which given a state-action pair
# compute the matrices A,B,C such that x_{k+1} = A x_k + Bu_k + C

class PredictiveModel():
    def __init__(self,  n, d, map, trToUse):
        self.map = map
        self.n = n # state dimension
        self.d = d # input dimention
        self.xStored = []
        self.uStored = []
        self.MaxNumPoint = 7 # max number of point per lap to use 
        self.h = 5 # bandwidth of the Kernel for local linear regression
        self.lamb = 0.0 # regularization
        self.dt = 0.1
        self.scaling = np.array([[0.1, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 1.0]])

        self.stateFeatures    = [0, 1, 2]
        self.inputFeaturesVx  = [1]
        self.inputFeaturesLat = [0]
        self.usedIt = [i for i in range(trToUse)]
        self.lapTime = []
    

    def addTrajectory(self, x, u):
        if self.lapTime == [] or x.shape[0] >= self.lapTime[-1]:
            self.xStored.append(x)
            self.uStored.append(u)
            self.lapTime.append(x.shape[0])
        else:
            for i in range(0, len(self.xStored)):
                if x.shape[0] < self.lapTime[i]:
                    self.xStored.insert(i, x) 
                    self.uStored.insert(i, u) 
                    self.lapTime.insert(i, x.shape[0]) 
                    break

    def regressionAndLinearization(self, x, u):
        Ai = np.zeros((self.n, self.n))
        Bi = np.zeros((self.n, self.d))
        Ci = np.zeros(self.n)

        # Compute Index to use for each stored lap
        xuLin = np.hstack((x[self.stateFeatures], u[:]))
        self.indexSelected = []
        self.K = []
        for ii in self.usedIt:
            indexSelected_i, K_i = self.computeIndices(xuLin, ii)
            self.indexSelected.append(indexSelected_i)
            self.K.append(K_i)
        # print("xuLin: ",xuLin)
        # print("aaa indexSelected: ", self.indexSelected)

        # =========================
        # ====== Identify vx ======
        Q_vx, M_vx = self.compute_Q_M(self.inputFeaturesVx, self.usedIt)

        yIndex = 0
        b_vx = self.compute_b(yIndex, self.usedIt, M_vx)
        Ai[yIndex, self.stateFeatures], Bi[yIndex, self.inputFeaturesVx], Ci[yIndex] = self.LMPC_LocLinReg(Q_vx, b_vx, self.inputFeaturesVx)

        # =======================================
        # ====== Identify Lateral Dynamics ======
        Q_lat, M_lat = self.compute_Q_M(self.inputFeaturesLat, self.usedIt)

        yIndex = 1  # vy
        b_vy = self.compute_b(yIndex, self.usedIt, M_lat)
        Ai[yIndex, self.stateFeatures], Bi[yIndex, self.inputFeaturesLat], Ci[yIndex] = self.LMPC_LocLinReg(Q_lat, b_vy, self.inputFeaturesLat)

        yIndex = 2  # wz
        b_wz = self.compute_b(yIndex, self.usedIt, M_lat)
        Ai[yIndex, self.stateFeatures], Bi[yIndex, self.inputFeaturesLat], Ci[yIndex] = self.LMPC_LocLinReg(Q_lat, b_wz, self.inputFeaturesLat)

        # ===========================
        # ===== Linearization =======
        vx = x[0]; vy   = x[1]
        wz = x[2]; epsi = x[3]
        s  = x[4]; ey   = x[5]
        dt = self.dt

        # if s < 0:
        #     print("s is negative, here the state: \n", x)

        startTimer = datetime.datetime.now()  # Start timer for LMPC iteration
        cur = self.map.curvature(s)
        cur = self.map.curvature(s)
        den = 1 - cur * ey

        min_den = 0.3  # safety threshold
        if abs(den) < min_den:
            den = min_den * np.sign(den) if den != 0 else min_den
        # ===========================
        # ===== Linearize epsi ======
        # epsi_{k+1} = epsi + dt * ( wz - (vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey) * cur )
        depsi_vx   = -dt * np.cos(epsi) / den * cur
        depsi_vy   = dt * np.sin(epsi) / den * cur
        depsi_wz   = dt
        depsi_epsi = 1 - dt * (-vx * np.sin(epsi) - vy * np.cos(epsi)) / den * cur
        depsi_s    = 0  # Because cur = constant
        depsi_ey   = dt * (vx * np.cos(epsi) - vy * np.sin(epsi)) / (den ** 2) * cur * (-cur)

        Ai[3, :] = [depsi_vx, depsi_vy, depsi_wz, depsi_epsi, depsi_s, depsi_ey]
        Ci[3]    = epsi + dt * (wz - (vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey) * cur) - np.dot(Ai[3, :], x)
        # ===========================
        # ===== Linearize s =========
        # s_{k+1} = s    + dt * ( (vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey) )
        ds_vx   = dt * (np.cos(epsi) / den)
        ds_vy   = -dt * (np.sin(epsi) / den)
        ds_wz   = 0
        ds_epsi = dt * (-vx * np.sin(epsi) - vy * np.cos(epsi)) / den
        ds_s    = 1  # + Ts * (Vx * cos(epsi) - Vy * sin(epsi)) / (1 - ey * rho) ^ 2 * (-ey * drho);
        ds_ey   = -dt * (vx * np.cos(epsi) - vy * np.sin(epsi)) / (den ** 2) * (-cur)

        Ai[4, :] = [ds_vx, ds_vy, ds_wz, ds_epsi, ds_s, ds_ey]
        Ci[4]    = s + dt * ((vx * np.cos(epsi) - vy * np.sin(epsi)) / (1 - cur * ey)) - np.dot(Ai[4, :], x)

        # ===========================
        # ===== Linearize ey ========
        # ey_{k+1} = ey + dt * (vx * np.sin(epsi) + vy * np.cos(epsi))
        dey_vx   = dt * np.sin(epsi)
        dey_vy   = dt * np.cos(epsi)
        dey_wz   = 0
        dey_epsi = dt * (vx * np.cos(epsi) - vy * np.sin(epsi))
        dey_s    = 0
        dey_ey   = 1

        Ai[5, :] = [dey_vx, dey_vy, dey_wz, dey_epsi, dey_s, dey_ey]
        Ci[5]    = ey + dt * (vx * np.sin(epsi) + vy * np.cos(epsi)) - np.dot(Ai[5, :], x)

        endTimer = datetime.datetime.now(); deltaTimer_tv = endTimer - startTimer

        return Ai, Bi, Ci

    def compute_Q_M(self, inputFeatures, usedIt):
        Counter = 0
        X0   = np.empty((0,len(self.stateFeatures)+len(inputFeatures)))
        Ktot = np.empty((0))

        for it in usedIt:
            X0 = np.append( X0, np.hstack((self.xStored[it][np.ix_(self.indexSelected[Counter], self.stateFeatures)],self.uStored[it][np.ix_(self.indexSelected[Counter], inputFeatures)])), axis=0 )
            Ktot    = np.append(Ktot, self.K[Counter])
            Counter += 1

        M = np.hstack( (X0, np.ones((X0.shape[0], 1))) )
        Q0 = np.dot(np.dot(M.T, np.diag(Ktot)), M)
        Q = matrix(Q0 + self.lamb * np.eye(Q0.shape[0]))

        return Q, M

    def compute_b(self, yIndex, usedIt, M):
        Counter = 0
        y = np.empty((0))
        Ktot = np.empty((0))

        for it in usedIt:
            y       = np.append(y, np.squeeze(self.xStored[it][self.indexSelected[Counter] + 1, yIndex]))
            Ktot    = np.append(Ktot, self.K[Counter])
            Counter += 1

        b = matrix(-np.dot(np.dot(M.T, np.diag(Ktot)), y))
        return b

    # def LMPC_LocLinReg(self, Q, b, inputFeatures):
    #     # Solve QP
    #     res_cons = qp(Q, b) # This is ordered as [A B C]
    #     # Unpack results
    #     result = np.squeeze(np.array(res_cons['x']))
    #     A = result[0:len(self.stateFeatures)]
    #     B = result[len(self.stateFeatures):(len(self.stateFeatures)+len(inputFeatures))]
    #     C = result[-1]
    #     return A, B, C

    def LMPC_LocLinReg(self, Q, b, inputFeatures):
        """
        Robust wrapper around cvxopt.qp to avoid rank-deficient failures.
        Tries progressively larger diagonal regularization eps until solver succeeds.
        Returns A, B, C as before.
        """

        from cvxopt import matrix as cvx_matrix
        from cvxopt import solvers as cvx_solvers
        import numpy as _np

        # Helper: convert cvxopt.matrix -> numpy array
        def _cvx_to_np(cvxm):
            try:
                return _np.array(cvxm, dtype=float)
            except Exception:
                # some fallback
                return _np.asarray(cvxm, dtype=float)

        # Try calling qp directly first (no regularization)
        try:
            sol = cvx_solvers.qp(Q, b)
            # success without regularization
            # print("[LMPC] qp success with no reg")
        except Exception as first_exc:
            # Diagnostics
            try:
                Q_np = _cvx_to_np(Q)
                # symmetric enforcement
                Q_np = 0.5 * (Q_np + Q_np.T)
                # eigen / rank diagnostics
                try:
                    eigs = _np.linalg.eigvalsh(Q_np)
                    min_eig = float(eigs[0])
                    max_eig = float(eigs[-1])
                except Exception:
                    min_eig = None
                    max_eig = None
                rank_est = int(_np.linalg.matrix_rank(Q_np, tol=1e-8))
                # Print useful debugging info
                # print("[LMPC_LocLinReg] qp initial failure -> diagnostics:")
                # print("  Q shape:", Q_np.shape)
                # if min_eig is not None:
                #     print("  Q eig min,max: {:.3e}, {:.3e}".format(min_eig, max_eig))
                # print("  Estimated rank(Q):", rank_est, " / ", Q_np.shape[0])
            except Exception as d_exc:
                print("[LMPC_LocLinReg] failed to compute diagnostics:", d_exc)

            # Try progressive regularization
            eps_list = [1e-12, 1e-10, 1e-9, 1e-8, 1e-7, 1e-6]
            sol = None
            last_exc = None

            for eps in eps_list:
                try:
                    # Build Q_reg as cvxopt.matrix
                    Q_reg_np = Q_np + eps * _np.eye(Q_np.shape[0])
                    Q_reg = cvx_matrix(Q_reg_np)
                    sol = cvx_solvers.qp(Q_reg, b)
                    # print(f"[LMPC_LocLinReg] qp succeeded with eps={eps:.0e}")
                    break
                except Exception as e:
                    last_exc = e
                    # continue to next eps
                    # optionally print short message
                    # print(f"[LMPC_LocLinReg] qp fail eps={eps:.0e}: {e}")

            if sol is None:
                # Final fallback: try a larger eps (less conservative)
                try:
                    eps = 1e-5
                    Q_reg_np = Q_np + eps * _np.eye(Q_np.shape[0])
                    Q_reg = cvx_matrix(Q_reg_np)
                    sol = cvx_solvers.qp(Q_reg, b)
                    # print(f"[LMPC_LocLinReg] qp succeeded with eps={eps:.0e} (fallback)")
                except Exception as final_e:
                    # Give clear error with diagnostics
                    print("[LMPC_LocLinReg] qp failed for all eps attempts. Raising last exception.")
                    raise last_exc if last_exc is not None else final_e

        # Unpack solution (same as original code)
        result = _np.squeeze(_np.array(sol['x']))
        A = result[0:len(self.stateFeatures)]
        B = result[len(self.stateFeatures):(len(self.stateFeatures)+len(inputFeatures))]
        C = result[-1]
        return A, B, C


    def computeIndices(self, x, it):
        """
        Compute indices of locally relevant data points for regression.
        Robust to cases where no points are found within kernel bandwidth h.
        """
        oneVec = np.ones((self.xStored[it].shape[0] - 1, 1))
        xVec = (np.dot(np.array([x]).T, oneVec.T)).T
        DataMatrix = np.hstack((
            self.xStored[it][0:-1, self.stateFeatures],
            self.uStored[it][0:-1, :]
        ))

        diff = np.dot((DataMatrix - xVec), self.scaling)
        norm = la.norm(diff, 1, axis=1)

        # Normal case: select points with norm < h
        idx = np.where(norm < self.h)[0]

        # ✅ Robustness: handle empty or tiny selections
        if idx.size == 0:
            # No close samples — fallback to nearest MaxNumPoint
            idx = np.argsort(norm)[:self.MaxNumPoint]
            # print(f"[computeIndices] ⚠️ No points within h={self.h}, using nearest {len(idx)} points.")
        elif idx.size < self.MaxNumPoint:
            # Less than desired number — use all found
            idx = np.argsort(norm)[:idx.size]
        else:
            # Normal case — take closest MaxNumPoint
            idx = np.argsort(norm[idx])[:self.MaxNumPoint]

        # Compute kernel weights
        norm_sel = norm[idx]
        K = (1 - (norm_sel / self.h) ** 2) * 3 / 4
        K = np.maximum(K, 1e-8)  # ensure nonzero

        return idx, K

    # def computeIndices(self, x, it):
    #     oneVec = np.ones( (self.xStored[it].shape[0]-1, 1) )
    #     xVec = (np.dot( np.array([x]).T, oneVec.T )).T
    #     DataMatrix = np.hstack((self.xStored[it][0:-1, self.stateFeatures], self.uStored[it][0:-1, :]))

    #     diff  = np.dot(( DataMatrix - xVec ), self.scaling)
    #     norm = la.norm(diff, 1, axis=1)
    #     indexTot =  np.squeeze(np.where(norm < self.h))
    #     if (indexTot.shape[0] >= self.MaxNumPoint):
    #         index = np.argsort(norm)[0:self.MaxNumPoint]
    #     else:
    #         index = indexTot

    #     K  = ( 1 - ( norm[index] / self.h )**2 ) * 3/4
    #     # if norm.shape[0]<500:
    #     #     print("norm: ", norm, norm.shape)

    #     return index, K
    
    def computeIndices(self, x, it):
        """
        Compute indices of locally relevant data points for regression.
        Robust to cases where no points are found within kernel bandwidth h.
        """
        oneVec = np.ones((self.xStored[it].shape[0] - 1, 1))
        xVec = (np.dot(np.array([x]).T, oneVec.T)).T
        DataMatrix = np.hstack((
            self.xStored[it][0:-1, self.stateFeatures],
            self.uStored[it][0:-1, :]
        ))

        diff = np.dot((DataMatrix - xVec), self.scaling)
        norm = la.norm(diff, 1, axis=1)

        # Normal case: select points with norm < h
        idx = np.where(norm < self.h)[0]

        # ✅ Robustness: handle empty or tiny selections
        if idx.size == 0:
            # No close samples — fallback to nearest MaxNumPoint
            idx = np.argsort(norm)[:self.MaxNumPoint]
            # print(f"[computeIndices] ⚠️ No points within h={self.h}, using nearest {len(idx)} points.")
        elif idx.size < self.MaxNumPoint:
            # Less than desired number — use all found
            idx = np.argsort(norm)[:idx.size]
        else:
            # Normal case — take closest MaxNumPoint
            idx = np.argsort(norm[idx])[:self.MaxNumPoint]

        # Compute kernel weights
        norm_sel = norm[idx]
        K = (1 - (norm_sel / self.h) ** 2) * 3 / 4
        K = np.maximum(K, 1e-8)  # ensure nonzero

        return idx, K