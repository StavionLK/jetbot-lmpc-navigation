#!/usr/bin/env python3
import rospy
import math
import csv
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point
import tf.transformations as tft
import matplotlib.pyplot as plt

def wrap(angle):
    if angle < -np.pi:
        return 2 * np.pi + angle
    elif angle > np.pi:
        return angle - 2 * np.pi
    else:
        return angle

def sign(a):
    return 1 if a >= 0 else -1

def build_track(spec):
    PointAndTangent = np.zeros((spec.shape[0] + 1, 6))
    all_x, all_y = [0.0], [0.0]
    for i in range(spec.shape[0]):
        L, R = spec[i, 0], spec[i, 1]
        if R == 0:
            if i == 0:
                ang, xs, ys = 0, 0, 0
            else:
                ang, xs, ys = PointAndTangent[i - 1, 2], PointAndTangent[i - 1, 0], PointAndTangent[i - 1, 1]
            xe = xs + L * np.cos(ang)
            ye = ys + L * np.sin(ang)
            psi = ang
            PointAndTangent[i, :] = [xe, ye, psi, 0, L, 0]
            x_segment = np.linspace(xs, xe, 60)
            y_segment = np.linspace(ys, ye, 60)
        else:
            direction = sign(R)
            if i == 0:
                ang, xs, ys = 0, 0, 0
            else:
                ang, xs, ys = PointAndTangent[i - 1, 2], PointAndTangent[i - 1, 0], PointAndTangent[i - 1, 1]
            cx = xs + abs(R) * np.cos(ang + direction * np.pi / 2)
            cy = ys + abs(R) * np.sin(ang + direction * np.pi / 2)
            spanAng = L / abs(R)
            psi = wrap(ang + spanAng * np.sign(R))
            angles = np.linspace(ang, ang + direction * spanAng, 80)
            x_segment = cx + abs(R) * np.cos(angles - direction * np.pi / 2)
            y_segment = cy + abs(R) * np.sin(angles - direction * np.pi / 2)
            PointAndTangent[i, :] = [x_segment[-1], y_segment[-1], psi, 0, L, 1 / R]
        all_x.extend(x_segment[1:])
        all_y.extend(y_segment[1:])
    return np.array(all_x), np.array(all_y)

def fit_to_gazebo_area(X, Y, bounds=((4.5, 11.5), (1.0, 10.4))):
    (x_min_t, x_max_t), (y_min_t, y_max_t) = bounds
    x_min, x_max, y_min, y_max = np.min(X), np.max(X), np.min(Y), np.max(Y)
    sx = (x_max_t - x_min_t) / (x_max - x_min)
    sy = (y_max_t - y_min_t) / (y_max - y_min)
    s = min(sx, sy) * 0.9
    Xf = (X - x_min) * s + x_min_t
    Yf = (Y - y_min) * s + y_min_t
    return Xf, Yf

def create_road_sdf(length, i):
    return f"""
    <sdf version='1.6'>
      <model name='road_seg_{i}'>
        <static>true</static>
        <link name='road_link'>
          <visual name='road_visual'>
            <geometry><box><size>{length} 0.5 0.01</size></box></geometry>
            <material>
              <script>
                <uri>model://road_material/materials/scripts</uri>
                <uri>model://road_material/materials/textures</uri>
                <name>MyRoad/Road</name>
              </script>
            </material>
          </visual>
          <collision name='road_collision'>
            <geometry><box><size>{length} 0.5 0.01</size></box></geometry>
          </collision>
        </link>
      </model>
    </sdf>
    """

def main():
    rospy.init_node("curved_track_spawner")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    lengthCurve = 4.5
    spec = np.array([
        [1.0, 0],
        [lengthCurve, lengthCurve / np.pi],
        [lengthCurve / 2, -lengthCurve / np.pi],
        [lengthCurve, lengthCurve / np.pi],
        [lengthCurve / np.pi * 2, 0],
        [lengthCurve / 2, lengthCurve / np.pi],
        [2.0, 0]
    ])

    # Build base track and fit to area
    X, Y = build_track(spec)
    # print(X)
    # print(Y)
    Xf, Yf = fit_to_gazebo_area(X, Y)

    # Apply translation (shift)
    Xf += 1.5
    Yf += 3.5

    rospy.loginfo("🚧 Spawning translated curvature road segments...")
    xs, ys, yaws = [], [], []
    for i in range(len(Xf) - 1):
        x1, y1 = Xf[i], Yf[i]
        x2, y2 = Xf[i + 1], Yf[i + 1]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        xm, ym = x1 + 0.5 * dx, y1 + 0.5 * dy
        quat = tft.quaternion_from_euler(0, 0, yaw)
        pose = Pose()
        pose.position = Point(xm, ym, 0)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        sdf = create_road_sdf(length, i)
        xs.append(xm)
        ys.append(ym)
        yaws.append(yaw)

        # Uncomment to spawn in Gazebo
        # spawn_model(f"road_seg_{i}", sdf, "", pose, "world")

    with open("path_points.csv", "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "yaw"])
        for x, y, yaw in zip(xs, ys, yaws):
            writer.writerow([x, y, yaw])


    plt.figure(figsize=(8, 8))
    plt.plot(Xf, Yf, 'b-', lw=2, label="Curved Track (Shifted)")
    plt.scatter(Xf[0], Yf[0], color='r', s=80, label='Start (Shifted)')
    plt.title("Curved Track Shifted by +1.5m X, +3.5m Y")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    main()
