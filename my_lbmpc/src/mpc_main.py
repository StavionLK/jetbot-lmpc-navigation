import pandas as pd
import numpy as np

# === 1. Load the original trajectory ===
traj = pd.read_csv("short_path.csv")
points = traj[['x', 'y', 'yaw']].to_numpy()

# === 2. Current robot pose (from your sensors or odometry) ===
current_x = 4.399793
current_y = 7.774899
current_yaw = -1.384703  # radians

# === 3. Find the closest point in the trajectory ===
# Compute position + yaw distance (yaw weighted less)
yaw_diff = np.arctan2(np.sin(points[:, 2] - current_yaw), np.cos(points[:, 2] - current_yaw))
pos_diff = np.sqrt((points[:, 0] - current_x)**2 + (points[:, 1] - current_y)**2)
dist_total = pos_diff + 0.3 * np.abs(yaw_diff)  # weight yaw difference less than position

# Find the closest trajectory index
closest_idx = np.argmin(dist_total)
print(f"Closest trajectory index: {closest_idx}")
print(f"Closest waypoint: x={points[closest_idx,0]:.3f}, y={points[closest_idx,1]:.3f}, yaw={points[closest_idx,2]:.3f}")

# === 4. Reorder trajectory so it starts from the closest point ===
reordered_points = np.vstack([points[closest_idx:], points[:closest_idx]])

# === 5. Save new trajectory to CSV ===
new_traj = pd.DataFrame(reordered_points, columns=['x', 'y', 'yaw'])
new_csv_path = "short_path_aligned.csv"
new_traj.to_csv(new_csv_path, index=False)

print(f"\n✅ New trajectory saved as: {new_csv_path}")
print("Robot will now start from the nearest waypoint in the saved path.")
