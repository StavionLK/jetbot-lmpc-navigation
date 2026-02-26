#!/usr/bin/env python3
import sys
sys.path.append('fnc/simulator')
sys.path.append('fnc/controller')
sys.path.append('fnc')

import rospy
import numpy as np
import math
import tf.transformations as tft
import matplotlib.pyplot as plt
import pdb
import pickle
import os 
# ROS imports
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetPhysicsProperties, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist


# LMPC project imports
from plot import plotTrajectory, plotClosedLoopLMPC, animation_xy
from initControllerParameters import initMPCParams, initLMPCParams
from PredictiveControllers import MPC, LMPC
from PredictiveModel import PredictiveModel
from Utilities import Regression, PID
from SysModel import RosGazeboSimulator   # Ensure this points to your simulator with cmd_vel logic
from Track import Map
from ackermann_msgs.msg import AckermannDrive
from race.msg import drive_param



# -------------------------- Gazebo Reset Function --------------------------
def reset_robot(model_name="ackermann_vehicle",
                x=5.163443, y=7.766847, yaw=-1.573762):
    """Reset Gazebo robot to a given pose and ensure physics is unpaused."""

    def unpause_gazebo():
        try:
            rospy.wait_for_service('/gazebo/unpause_physics', timeout=3)
            unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause()
            # rospy.loginfo("Gazebo unpaused.")
        except Exception as e:
            rospy.logwarn(f"Could not unpause Gazebo: {e}")

    def pause_gazebo():
        try:
            rospy.wait_for_service('/gazebo/pause_physics', timeout=3)
            pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            pause()
            # rospy.loginfo("Gazebo paused.")
        except Exception as e:
            rospy.logwarn(f"Could not pause Gazebo: {e}")

    def set_model_pose():
        try:
            rospy.wait_for_service('/gazebo/set_model_state', timeout=3)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            quat = tft.quaternion_from_euler(0, 0, yaw)

            state = ModelState()
            state.model_name = model_name
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = 0.1
            state.pose.orientation.x = quat[0]
            state.pose.orientation.y = quat[1]
            state.pose.orientation.z = quat[2]
            state.pose.orientation.w = quat[3]
            state.twist.linear.x = 0.0
            state.twist.angular.z = 0.0
            state.reference_frame = "world"

            set_state(state)
            # rospy.loginfo(f"Reset robot '{model_name}' to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        except Exception as e:
            rospy.logerr(f"Failed to reset model: {e}")

    # Stop and reset
    pause_gazebo()
    set_model_pose()
    unpause_gazebo()
    rospy.sleep(1.0)

def stop_robot(cmd_pub,pub_ack):
    cmd = Twist()
    cmd.linear.x = 0
    cmd.angular.z = 0
    cmd_pub.publish(cmd)

    msg = AckermannDrive()
    msg.speed = float(0)   # * 50    # scale like PID controller
    msg.steering_angle = float(0) #* 100      # scale like PID controller
    pub_ack.publish(msg)
    # rospy.loginfo("Robot stopped.")
# -------------------------- Main LMPC Function --------------------------
def main():
    rospy.init_node("LB_MPC_TestNode", anonymous=True)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    pub_ack = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)


    
    # wait_for_reset_pose(map)
    # ======================================================================
    # ============================ Init Parameters =========================
    # ======================================================================
    N = 14
    n, d = 6, 2
    x0 = np.array([0, 0, 0, 0, 0, 0])
    xS = [x0, x0]
    dt = 0.1

    path_1 = "/home/josh/catkin_ws/src/thesis/test_mpc_1/my_lbmpc/scripts/path_points_re.csv"
    map = Map(path_1)
    vt = 0.8

    # x0 = 9.625621740614353	 #map.x_map[0]
    # y0 = 4.5 #map.y_map[0]
    # yaw0 = 0 #map.psi_map[0] if hasattr(map, 'psi_map') else 0.0
    x0 = map.x_map[0]
    y0 = map.y_map[0]
    yaw0 = map.psi_map[0] if hasattr(map, 'psi_map') else 0.0


    stop_robot(cmd_pub,pub_ack)
    reset_robot(x=x0, y=y0, yaw=yaw0)
    map.reset_progress(0.0)

    # Controller parameters
    mpcParam, ltvmpcParam = initMPCParams(n, d, N, vt)
    numSS_it, numSS_Points, Laps, TimeLMPC, QterminalSlack, lmpcParameters = initLMPCParams(map, N)

    # Simulators (Gazebo-connected)
    simulator = RosGazeboSimulator(map, dt=0.1, multiLap=False, flagLMPC=False)
    LMPCsimulator = RosGazeboSimulator(map, dt=0.1, multiLap=True, flagLMPC=True)

    # ======================================================================
    # ============================ PID Controller ==========================
    # ======================================================================
    # rospy.loginfo("Starting PID controller...")


    # start_time = rospy.get_time()
    # PIDController = PID(vt)
    # xPID_cl, uPID_cl, xPID_cl_glob, _ = simulator.sim([None, None], PIDController, maxSimTime=400.0)
    # rospy.loginfo("PID finished.")

    if not os.path.exists("pid_data.pkl"):
        rospy.loginfo("Running PID for the first time...")
        start_time = rospy.get_time()
        PIDController = PID(vt)
        xPID_cl, uPID_cl, xPID_cl_glob, _ = simulator.sim([None, None], PIDController, maxSimTime=400.0)
        rospy.loginfo("PID finished.")
        stop_robot(cmd_pub,pub_ack)
        reset_robot(x=x0, y=y0, yaw=yaw0)
        map.reset_progress(0.0)
        end_time = rospy.get_time()
        rospy.loginfo(f"PID finished. Gazebo time = {end_time - start_time:.2f} sec")
        # SAVE RESULTS
        # with open("pid_data.pkl", "wb") as f:
        #     pickle.dump((xPID_cl, uPID_cl, xPID_cl_glob), f)
        # rospy.loginfo("Saved PID data to pid_data.pkl")

    else:
        rospy.loginfo("Loading saved PID data (skipping PID execution)...")
        with open("pid_data.pkl", "rb") as f:
            stop_robot(cmd_pub,pub_ack)
            reset_robot(x=x0, y=y0, yaw=yaw0)
            map.reset_progress(0.0)
            xPID_cl, uPID_cl, xPID_cl_glob = pickle.load(f)
            print(xPID_cl.shape, uPID_cl.shape, len(xPID_cl_glob))

    # Always print something
    rospy.loginfo("PID data ready.")


    
    # ======================================================================
    # ============================ Linear MPC ==============================
    # ======================================================================
    rospy.loginfo("Starting MPC controller...")
    start_time = rospy.get_time()
    lamb = 1e-9
    A, B, Error = Regression(xPID_cl, uPID_cl, lamb)
    mpcParam.A, mpcParam.B = A, B
    print("\n===== REGRESSION DEBUG =====")
    print("A =\n", A)
    print("B =\n", B)
    print("Eigenvalues of A:\n", np.linalg.eigvals(A))
    print("============================\n")


    mpc = MPC(mpcParam)
    xMPC_cl, uMPC_cl, xMPC_cl_glob, _ = simulator.sim([None, None], mpc, maxSimTime=400.0)
    rospy.loginfo("MPC finished.")
    stop_robot(cmd_pub,pub_ack)
    reset_robot(x=x0, y=y0, yaw=yaw0)
    map.reset_progress(0.0)
    end_time = rospy.get_time()
    rospy.loginfo(f"MPC finished. Gazebo time = {end_time - start_time:.2f} sec")


    # ======================================================================
    # ============================ TV-MPC ==================================
    # ======================================================================
    
    rospy.loginfo("Starting TV-MPC controller...")
    start_time = rospy.get_time()
    predictiveModel = PredictiveModel(n, d, map, 1)
    predictiveModel.addTrajectory(xPID_cl, uPID_cl)
    ltvmpcParam.timeVarying = True

    tvmpc = MPC(ltvmpcParam, predictiveModel)
    xTVMPC_cl, uTVMPC_cl, xTVMPC_cl_glob, _ = simulator.sim([None, None], tvmpc, maxSimTime=400.0)
    rospy.loginfo("TV-MPC finished.")
    stop_robot(cmd_pub,pub_ack)
    reset_robot(x=x0, y=y0, yaw=yaw0)
    map.reset_progress(0.0)
    end_time = rospy.get_time()
    rospy.loginfo(f"TV-MPC finished. Gazebo time = {end_time - start_time:.2f} sec")
    

    # ======================================================================
    # ============================ LMPC ====================================
    # ======================================================================
    rospy.loginfo("Starting LMPC controller...")
    lmpcpredictiveModel = PredictiveModel(n, d, map, 4)
    print(" dAFFSF ASFJSAFK ASJF")
    for _ in range(4):
        lmpcpredictiveModel.addTrajectory(xMPC_cl, uMPC_cl)

    lmpcParameters.timeVarying = True
    lmpc = LMPC(numSS_Points, numSS_it, QterminalSlack, lmpcParameters, lmpcpredictiveModel)
    for _ in range(4):
        lmpc.addTrajectory(xMPC_cl, uMPC_cl, xMPC_cl_glob)

    for it in range(numSS_it,10): #range(numSS_it, Laps):
        lap_start = rospy.get_time()
        xLMPC, uLMPC, xLMPC_glob, xS = LMPCsimulator.sim([None, None], lmpc, maxSimTime=400.0)
        lmpc.addTrajectory(xLMPC, uLMPC, xLMPC_glob)
        lmpcpredictiveModel.addTrajectory(xLMPC, uLMPC)
        # rospy.loginfo(f"Lap {it}: {np.round(lmpc.Qfun[it][0]*dt, 2)} sec")
        lap_end = rospy.get_time()
        lap_time = lap_end - lap_start
        rospy.loginfo(f"Lap {it}: {lap_time:.2f} sec (Gazebo time)")
        stop_robot(cmd_pub,pub_ack)
        reset_robot(x=x0, y=y0, yaw=yaw0)
        map.reset_progress(0.0)
        

    rospy.loginfo("LMPC finished successfully!")

    # ======================================================================
    # ============================ Plot Results ============================
    # ======================================================================
    for i in range(lmpc.it):
        print(f"Lap time {i}: {np.round(lmpc.Qfun[i][0]*dt, 2)} s")

    plotTrajectory(map, xPID_cl, xPID_cl_glob, uPID_cl, 'PID')
    plotTrajectory(map, xMPC_cl, xMPC_cl_glob, uMPC_cl, 'MPC')
    plotTrajectory(map, xTVMPC_cl, xTVMPC_cl_glob, uTVMPC_cl, 'TV-MPC')
    plotClosedLoopLMPC(lmpc, map)
    animation_xy(map, lmpc, Laps-1)
    plt.show()


# -------------------------- Entry Point --------------------------
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
