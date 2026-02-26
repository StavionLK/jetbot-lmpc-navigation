# #!/usr/bin/env python3
# import math
# import rospy
# import csv
# import numpy as np
# from gazebo_msgs.srv import SpawnModel
# from geometry_msgs.msg import Pose, Point
# import tf.transformations as tft
# from scipy.interpolate import splprep, splev
# import matplotlib.pyplot as plt

# def create_road_sdf(length, i):
#     """Generate an SDF string for a single straight road section of given length."""
#     return f"""
#     <sdf version='1.6'>
#       <model name='road_section_{i}'>
#         <static>true</static>
#         <link name='road_link'>
#           <visual name='road_visual'>
#             <geometry>
#               <box><size>{length} 0.8 0.0001</size></box>
#             </geometry>
#             <material>
#               <script>
#                 <uri>model://road_material/materials/scripts</uri>
#                 <uri>model://road_material/materials/textures</uri>
#                 <name>MyRoad/Road</name>
#               </script>
#             </material>
#           </visual>
#           <collision name='road_collision'>
#             <geometry>
#               <box><size>{length} 0.8 0.0001</size></box>
#             </geometry>
#           </collision>
#         </link>
#       </model>
#     </sdf>
#     """


# def smooth_path(waypoints, resolution=0.2, closed=True):
#     """Fit a smooth periodic spline through waypoints and sample intermediate points."""
#     x, y = zip(*waypoints)

#     # If closed loop and start==end, remove duplicate endpoint
#     if closed and (x[0] == x[-1] and y[0] == y[-1]):
#         x, y = x[:-1], y[:-1]

#     # Fit a spline (periodic if closed)
#     tck, _ = splprep([x, y], s=0, per=closed)

#     # Uniformly sample along spline
#     total_length = sum(np.hypot(np.diff(x), np.diff(y)))
#     u_fine = np.linspace(0, 1, int(total_length / resolution))
#     x_smooth, y_smooth = splev(u_fine, tck)

#     # If closed, append the first point to end for full closure
#     if closed:
#         x_smooth = np.append(x_smooth, x_smooth[0])
#         y_smooth = np.append(y_smooth, y_smooth[0])

#     return list(zip(x_smooth, y_smooth))


# def main():
#     rospy.init_node('smooth_road_spawner', anonymous=True)
#     rospy.wait_for_service('/gazebo/spawn_sdf_model')
#     spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

#     # Waypoints (start and end are same for loop)
#     waypoints = [
#         (4.5, 10.4),
#         (8.0, 10.4),
#         (11.5, 10.4),
#         (11.5, 8.4),
#         (9.0, 2.9),
#         (6,1),
#         (5.0, 1),
#         (5.0, 2.9),
#         (5.0, 4.9),
#         (4.5, 10.4)
#     ]

#     # Smooth the path with periodic closure
#     smooth_waypoints = smooth_path(waypoints, resolution=0.3, closed=True)
#     rospy.loginfo(f"Generated {len(smooth_waypoints)} smooth points for closed loop.")

#     segment_length = 0.3 # Keep short for smoother curve
#     rospy.loginfo("🚧 Spawning smooth continuous curved road...")
#     sdf_sections = []
#     xs, ys, yaws = [], [], []
#     for i in range(len(smooth_waypoints) - 1):
#         x1, y1 = smooth_waypoints[i]
#         x2, y2 = smooth_waypoints[i + 1]

#         # Compute geometry
#         dx, dy = (x2 - x1), (y2 - y1)
#         length = math.sqrt(dx**2 + dy**2)
#         yaw = math.atan2(dy, dx)

#         # Center the segment
#         xm = x1 + (length / 2.0) * math.cos(yaw)
#         ym = y1 + (length / 2.0) * math.sin(yaw)

#         quat = tft.quaternion_from_euler(0, 0, yaw)
#         sdf = create_road_sdf(length, i)

#         pose = Pose()
#         pose.position = Point(xm, ym, 0)
#         pose.orientation.x = quat[0]
#         pose.orientation.y = quat[1]
#         pose.orientation.z = quat[2]
#         pose.orientation.w = quat[3]

#         print(pose)
#         xs.append(xm)
#         ys.append(ym)
#         yaws.append(yaw)

        
#         try:
#             spawn_model(f"road_section_{i}", sdf, "", pose, "world")
#             rospy.loginfo(f"✅ Spawned curved segment {i}/{len(smooth_waypoints)-1}")
#         except rospy.ServiceException as e:
#             rospy.logerr(f"❌ Failed to spawn road segment {i}: {e}")

#         rospy.sleep(0.05)

#     #     sdf_sections.append(f"""
#     #     <model name='road_section_{i}'>
#     #       <static>true</static>
#     #       <pose>{xm} {ym} 0 {quat[0]} {quat[1]} {quat[2]} {quat[3]}</pose>
#     #       <link name='road_link'>
#     #         <visual name='road_visual'>
#     #           <geometry>
#     #             <box><size>{length} 0.5 0.01</size></box>
#     #           </geometry>
#     #           <material>
#     #             <script>
#     #               <uri>model://road_material/materials/scripts</uri>
#     #               <uri>model://road_material/materials/textures</uri>
#     #               <name>MyRoad/Road</name>
#     #             </script>
#     #           </material>
#     #         </visual>
#     #         <collision name='road_collision'>
#     #           <geometry>
#     #             <box><size>{length} 0.5 0.01</size></box>
#     #           </geometry>
#     #         </collision>
#     #       </link>
#     #     </model>
#     #     """)

#     # sdf_full = f"""
#     # <sdf version='1.6'>
#     #   <model name='smooth_road'>
#     #     <static>true</static>
#     #     {''.join(sdf_sections)}
#     #   </model>
#     # </sdf>
#     # """

#     # with open("road_model.sdf", "w") as f:
#     #     f.write(sdf_full.strip())

#     path_filename = "small_path_points.csv"  # change path as needed
#     with open(path_filename, mode='w', newline='') as file:
#         writer = csv.writer(file)
#         writer.writerow(["x", "y", "yaw"])
#         for i in range(len(xs)):
#             writer.writerow([xs[i], ys[i], yaws[i]])


#     plt.figure(figsize=(10, 8))
#     plt.plot(xs, ys, 'bo-', label='Smooth Road Segment Centers')
#     plt.plot(*zip(*waypoints), 'rx--', label='Original Waypoints')

#     # Draw yaw arrows to show orientation
#     for i in range(0, len(xs), max(1, len(xs)//30)):  # reduce arrow clutter
#         plt.arrow(xs[i], ys[i],
#                   0.5 * math.cos(yaws[i]),
#                   0.5 * math.sin(yaws[i]),
#                   head_width=0.3, head_length=0.3, fc='g', ec='g')

#     plt.title("Smooth Road Segment Positions and Orientations")
#     plt.xlabel("X (meters)")
#     plt.ylabel("Y (meters)")
#     plt.legend()
#     plt.axis("equal")
#     plt.grid(True)
#     plt.show()

#     rospy.loginfo("🏁 Smooth curved road generation complete!")




# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
import math
import rospy
import csv
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point
import tf.transformations as tft
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt


# def create_road_sdf(length, i):
#     """Generate an SDF string for a single straight road section of given length."""
#     # return f"""
#     # <sdf version='1.6'>
#     #   <model name='road_section_{i}'>
#     #     <static>true</static>
#     #     <link name='road_link'>
#     #       <visual name='road_visual'>
#     #         <geometry>
#     #           <box><size>{length} 0.8 0.0001</size></box>
#     #         </geometry>
#     #         <material>
#     #           <script>
#     #             <uri>model://road_material/materials/scripts</uri>
#     #             <uri>model://road_material/materials/textures</uri>
#     #             <name>MyRoad/Road</name>
#     #           </script>
#     #         </material>
#     #       </visual>
#     #       <collision name='road_collision'>
#     #         <geometry>
#     #           <box><size>{length} 0.8 0.0001</size></box>
#     #         </geometry>
#     #       </collision>
#     #     </link>
#     #   </model>
#     # </sdf>
    # """


def create_wall_sdf(length, i, side):
    """Generate an SDF string for a wall section (left/right)."""
    return f"""
    <sdf version='1.6'>
      <model name='wall_{side}_{i}'>
        <static>true</static>
        <link name='wall_link'>
          <visual name='wall_visual'>
            <geometry>
              <box><size>{length} 0.1 1.5</size></box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Grey</name>
              </script>
            </material>
          </visual>
          <collision name='wall_collision'>
            <geometry>
              <box><size>{length} 0.1 1.5</size></box>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>
    """


def smooth_path(waypoints, resolution=0.2, closed=True):
    """Fit a smooth periodic spline through waypoints and sample intermediate points."""
    x, y = zip(*waypoints)

    if closed and (x[0] == x[-1] and y[0] == y[-1]):
        x, y = x[:-1], y[:-1]

    tck, _ = splprep([x, y], s=0, per=closed)
    total_length = sum(np.hypot(np.diff(x), np.diff(y)))
    u_fine = np.linspace(0, 1, int(total_length / resolution))
    x_smooth, y_smooth = splev(u_fine, tck)

    if closed:
        x_smooth = np.append(x_smooth, x_smooth[0])
        y_smooth = np.append(y_smooth, y_smooth[0])

    return list(zip(x_smooth, y_smooth))


def offset_curve(points, offset):
    """Generate a curve offset by a fixed perpendicular distance."""
    offset_points = []
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        yaw = math.atan2(y2 - y1, x2 - x1)
        ox = points[i][0] - offset * math.sin(yaw)
        oy = points[i][1] + offset * math.cos(yaw)
        offset_points.append((ox, oy))
    offset_points.append(offset_points[0])  # close loop
    return offset_points


def main():
    rospy.init_node('smooth_road_with_connected_walls_spawner', anonymous=True)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Define road waypoints
    # waypoints = [
    #     (4.5, 10.4),
    #     (8.0, 10.4),
    #     (11.5, 10.4),
    #     (11.5, 8.4),
    #     (9.0, 2.9),
    #     (6, 1),
    #     (5.0, 1),
    #     (5.0, 2.9),
    #     (5.0, 4.9),
    #     (4.5, 10.4)
    # ]

    waypoints = [
        (8.13, 1.00),
        (10.58, 1.77),
        (9.80, 4.04),
        (7.69, 5.39),
        (6.16, 7.30),
        (4.50, 5.56),
        (4.50, 2.83),
        (6.08, 1.00),
        (8.13, 1.00)
      ]

    # Generate smooth center path
    smooth_waypoints = smooth_path(waypoints, resolution=0.1, closed=True)
    rospy.loginfo(f"Generated {len(smooth_waypoints)} smooth points for closed loop.")

    # Generate left/right offset curves for walls
    wall_offset = 1.0
    left_curve = offset_curve(smooth_waypoints, wall_offset)
    right_curve = offset_curve(smooth_waypoints, -wall_offset)

    xs, ys, yaws = [], [], []

    rospy.loginfo("🚧 Spawning smooth road and continuous tunnel walls...")

    # --- Spawn road ---
    for i in range(len(smooth_waypoints) - 1):
        x1, y1 = smooth_waypoints[i]
        x2, y2 = smooth_waypoints[i + 1]
        dx, dy = x2 - x1, y2 - y1
        length = math.sqrt(dx**2 + dy**2)
        yaw = math.atan2(dy, dx)
        quat = tft.quaternion_from_euler(0, 0, yaw)
        xm = x1 + (length / 2.0) * math.cos(yaw)
        ym = y1 + (length / 2.0) * math.sin(yaw)

        sdf = create_road_sdf(length, i)
        pose = Pose()
        pose.position = Point(xm, ym, 0)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        spawn_model(f"road_section_{i}", sdf, "", pose, "world")

        xs.append(xm)
        ys.append(ym)
        yaws.append(yaw)
        rospy.sleep(0.02)

    # --- Spawn continuous left wall ---
    # for i in range(len(left_curve) - 1):
    #     x1, y1 = left_curve[i]
    #     x2, y2 = left_curve[i + 1]
    #     dx, dy = x2 - x1, y2 - y1
    #     length = math.sqrt(dx**2 + dy**2)
    #     yaw = math.atan2(dy, dx)
    #     xm = x1 + (length / 2.0) * math.cos(yaw)
    #     ym = y1 + (length / 2.0) * math.sin(yaw)
    #     quat = tft.quaternion_from_euler(0, 0, yaw)

    #     sdf = create_wall_sdf(length, i, "left")
    #     pose = Pose()
    #     pose.position = Point(xm, ym, 0.75)
    #     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    #     spawn_model(f"wall_left_{i}", sdf, "", pose, "world")
    #     rospy.sleep(0.01)

    # --- Spawn continuous right wall ---
    # for i in range(len(right_curve) - 1):
    #     x1, y1 = right_curve[i]
    #     x2, y2 = right_curve[i + 1]
    #     dx, dy = x2 - x1, y2 - y1
    #     length = math.sqrt(dx**2 + dy**2)
    #     yaw = math.atan2(dy, dx)
    #     xm = x1 + (length / 2.0) * math.cos(yaw)
    #     ym = y1 + (length / 2.0) * math.sin(yaw)
    #     quat = tft.quaternion_from_euler(0, 0, yaw)

    #     sdf = create_wall_sdf(length, i, "right")
    #     pose = Pose()
    #     pose.position = Point(xm, ym, 0.75)
    #     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    #     spawn_model(f"wall_right_{i}", sdf, "", pose, "world")
    #     rospy.sleep(0.01)

    # Save CSV for reference
    with open("road_with_connected_walls.csv", "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "yaw"])
        for x, y, yaw in zip(xs, ys, yaws):
            writer.writerow([x, y, yaw])

    # Plot visualization
    plt.figure(figsize=(10, 8))
    plt.plot(*zip(*smooth_waypoints), 'b-', label="Road Center")
    # plt.plot(*zip(*left_curve), 'r-', label="Left Wall")
    # plt.plot(*zip(*right_curve), 'g-', label="Right Wall")
    plt.axis("equal")
    plt.title("Smooth Road with Continuous Tunnel Walls")
    plt.legend()
    plt.grid(True)
    plt.show()

    rospy.loginfo("🏁 Continuous tunnel walls generated — no gaps!")


if __name__ == '__main__':
    main()
