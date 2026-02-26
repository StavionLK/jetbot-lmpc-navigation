#!/usr/bin/env python3
# import rospy
# import math
# import csv
# import numpy as np
# from gazebo_msgs.srv import SpawnModel
# from geometry_msgs.msg import Pose, Point
# import tf.transformations as tft
# import matplotlib.pyplot as plt
# import pandas as pd

# def create_wall_sdf(length, i, wall_width=0.05, wall_height=0.8):
#     """Create a simple thin vertical wall SDF segment."""
#     return f"""
#     <sdf version='1.6'>
#       <model name='wall_seg_{i}'>
#         <static>true</static>
#         <link name='wall_link'>
#           <visual name='wall_visual'>
#             <geometry><box><size>{length} {wall_width} {wall_height}</size></box></geometry>
#             <material>
#               <ambient>0.3 0.3 0.3 1</ambient>
#               <diffuse>0.6 0.6 0.6 1</diffuse>
#             </material>
#           </visual>
#           <collision name='wall_collision'>
#             <geometry><box><size>{length} {wall_width} {wall_height}</size></box></geometry>
#           </collision>
#         </link>
#       </model>
#     </sdf>
#     """

# def load_path_csv(csv_path):
#     """Load x,y,yaw from CSV (with or without headers)."""
#     df = pd.read_csv(csv_path, header=None, comment="#")
#     df = df.apply(pd.to_numeric, errors="coerce").dropna()
#     if df.shape[1] < 3:
#         raise ValueError("CSV must have at least 3 columns: x, y, yaw")
#     x, y, yaw = df[0].to_numpy(), df[1].to_numpy(), df[2].to_numpy()
#     return x, y, yaw

# def offset_path(x, y, yaw, offset):
#     """Offset path laterally by `offset` meters (positive = left)."""
#     x_off = x - offset * np.sin(yaw)
#     y_off = y + offset * np.cos(yaw)
#     return x_off, y_off

# def spawn_walls(x_path, y_path, yaw_path, offset, wall_tag):
#     """Spawn wall segments in Gazebo."""
#     rospy.wait_for_service('/gazebo/spawn_sdf_model')
#     spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

#     for i in range(len(x_path) - 1):
#         x1, y1 = x_path[i], y_path[i]
#         x2, y2 = x_path[i + 1], y_path[i + 1]
#         dx, dy = x2 - x1, y2 - y1
#         length = math.hypot(dx, dy)
#         yaw = math.atan2(dy, dx)
#         xm, ym = x1 + 0.5 * dx, y1 + 0.5 * dy
#         quat = tft.quaternion_from_euler(0, 0, yaw)
#         pose = Pose()
#         pose.position = Point(xm, ym, 0.4)  # wall base at z=0, height/2=0.4
#         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat

#         sdf = create_wall_sdf(length, f"{wall_tag}_{i}")
#         try:
#             spawn_model(f"wall_{wall_tag}_{i}", sdf, "", pose, "world")
#         except rospy.ServiceException as e:
#             rospy.logwarn(f"Failed to spawn wall_{wall_tag}_{i}: {e}")

# def main():
#     rospy.init_node("path_wall_spawner")
#     path_file = rospy.get_param("~path_file", "/home/josh/path.csv")
#     offset = rospy.get_param("~offset", 1.5)
#     spawn = rospy.get_param("~spawn", True)  # set False if you only want visualization

#     rospy.loginfo(f"Loading path from {path_file}")
#     x, y, yaw = load_path_csv(path_file)

#     # compute left and right wall coordinates
#     x_left, y_left = offset_path(x, y, yaw, offset)
#     x_right, y_right = offset_path(x, y, yaw, -offset)

#     # visualize
#     plt.figure(figsize=(8, 8))
#     plt.plot(x, y, "k-", lw=2, label="Centerline")
#     plt.plot(x_left, y_left, "r--", lw=2, label="Left wall (+1.5m)")
#     plt.plot(x_right, y_right, "b--", lw=2, label="Right wall (-1.5m)")
#     plt.axis("equal")
#     plt.xlabel("X [m]")
#     plt.ylabel("Y [m]")
#     plt.legend()
#     plt.grid(True)
#     plt.title("Walls offset ±1.5 m from path centerline")
#     plt.show(block=False)

#     if spawn:
#         rospy.loginfo("🚧 Spawning left walls...")
#         spawn_walls(x_left, y_left, yaw, offset, "left")
#         rospy.loginfo("🚧 Spawning right walls...")
#         spawn_walls(x_right, y_right, yaw, -offset, "right")
#         rospy.loginfo("✅ Walls spawned successfully!")

#     rospy.spin()

# if __name__ == "__main__":
#     main()



"""
Generate an SDF model containing left/right wall segments
offset ±1.5 m from a CSV path (x, y, yaw).
"""

import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import os

def load_path(csv_path):
    """Load x, y, yaw columns from a CSV (header optional)."""
    df = pd.read_csv(csv_path, header=None, comment="#")
    df = df.apply(pd.to_numeric, errors="coerce").dropna()
    if df.shape[1] < 3:
        raise ValueError("CSV must have at least 3 numeric columns: x, y, yaw")
    return df[0].to_numpy(), df[1].to_numpy(), df[2].to_numpy()

def offset_path(x, y, yaw, offset):
    """Compute offset path ±offset meters from centerline."""
    x_off = x - offset * np.sin(yaw)
    y_off = y + offset * np.cos(yaw)
    return x_off, y_off

def create_wall_segment_sdf(x1, y1, x2, y2, i, wall_tag,
                            wall_width=0.05, wall_height=0.8):
    """Generate SDF <model> XML for a single wall segment."""
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    xm, ym = x1 + 0.5 * dx, y1 + 0.5 * dy

    # Quaternion from yaw (Z-rotation only)
    qw = math.cos(yaw / 2)
    qz = math.sin(yaw / 2)

    return f"""
    <model name='wall_{wall_tag}_{i}'>
      <static>true</static>
      <pose>{xm} {ym} {wall_height/2:.2f} 0 0 {yaw}</pose>
      <link name='wall_link'>
        <visual name='wall_visual'>
          <geometry><box><size>{length} {wall_width} {wall_height}</size></box></geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
        <collision name='wall_collision'>
          <geometry><box><size>{length} {wall_width} {wall_height}</size></box></geometry>
        </collision>
      </link>
    </model>
    """

def build_walls_sdf(path_file, offset=1.5, wall_width=0.05, wall_height=0.8, out_dir="walls_model"):
    """Generate a combined SDF for left and right walls."""
    x, y, yaw = load_path(path_file)
    x_left, y_left = offset_path(x, y, yaw, offset)
    x_right, y_right = offset_path(x, y, yaw, -offset)

    os.makedirs(out_dir, exist_ok=True)
    model_name = "path_walls"
    sdf_path = os.path.join(out_dir, "model.sdf")
    config_path = os.path.join(out_dir, "model.config")

    # Combine all wall segments
    models_xml = ""
    for i in range(len(x_left) - 1):
        models_xml += create_wall_segment_sdf(x_left[i], y_left[i], x_left[i+1], y_left[i+1], i, "left",
                                              wall_width, wall_height)
    for i in range(len(x_right) - 1):
        models_xml += create_wall_segment_sdf(x_right[i], y_right[i], x_right[i+1], y_right[i+1], i, "right",
                                              wall_width, wall_height)

    # Write model.sdf
    sdf_xml = f"""<?xml version='1.0'?>
        <sdf version='1.6'>
        <model name='{model_name}'>
            <static>true</static>
            {models_xml}
        </model>
        </sdf>
        """
    with open(sdf_path, "w") as f:
        f.write(sdf_xml)

    # Write model.config
    config_xml = f"""<?xml version='1.0'?>
            <model>
            <name>{model_name}</name>
            <version>1.0</version>
            <sdf version='1.6'>model.sdf</sdf>
            <author>
                <name>AutoGen Script</name>
                <email>none@example.com</email>
            </author>
            <description>Walls ±{offset} m from path centerline</description>
            </model>
            """
    with open(config_path, "w") as f:
        f.write(config_xml)

    print(f"✅ SDF saved to: {os.path.abspath(sdf_path)}")

    # Optional plot
    plt.figure(figsize=(8, 8))
    plt.plot(x, y, 'k-', lw=2, label="Centerline")
    plt.plot(x_left, y_left, 'r--', lw=1.5, label=f"Left wall (+{offset} m)")
    plt.plot(x_right, y_right, 'b--', lw=1.5, label=f"Right wall (-{offset} m)")
    plt.axis('equal')
    plt.legend()
    plt.title("Path walls preview")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Generate SDF walls along a path")
    parser.add_argument("--path", required=True, help="Path CSV (x,y,yaw)")
    parser.add_argument("--offset", type=float, default=1.5, help="Wall offset [m]")
    parser.add_argument("--out", default="walls_model", help="Output model directory")
    args = parser.parse_args()
    build_walls_sdf(args.path, offset=args.offset, out_dir=args.out)
