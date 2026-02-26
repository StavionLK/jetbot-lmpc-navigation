import numpy as np
import math
import csv
from PIL import Image
import yaml

RES = 0.05        # 5 cm
PATH_HALF_W = 1 # 0.5 m

traj = []
with open("path_points.csv") as f:
    r = csv.reader(f)
    next(r)  # skip header
    for row in r:
        try:
            x, y, yaw = map(float, row)
            traj.append((x, y))
        except:
            continue

xs = [p[0] for p in traj]
ys = [p[1] for p in traj]

min_x, max_x = min(xs)-2, max(xs)+2
min_y, max_y = min(ys)-2, max(ys)+2

W = int((max_x - min_x) / RES)
H = int((max_y - min_y) / RES)

# -----------------------------------------------
# BACKGROUND BLACK (0)
# -----------------------------------------------
grid = np.zeros((H, W), dtype=np.uint8)

half_cells = int(PATH_HALF_W / RES)

# -----------------------------------------------
# DRAW TRACK AS WHITE (255)
# -----------------------------------------------
for (x, y) in traj:
    gx = int((x - min_x) / RES)
    gy = int((y - min_y) / RES)

    for dx in range(-half_cells, half_cells + 1):
        for dy in range(-half_cells, half_cells + 1):
            nx = gx + dx
            ny = gy + dy

            if 0 <= nx < W and 0 <= ny < H:
                if math.sqrt(dx*dx + dy*dy) <= half_cells:
                    grid[ny][nx] = 255   # WHITE

# -----------------------------------------------
# Save PNG
# -----------------------------------------------
img = Image.fromarray(grid, mode='L')
img.save("track_map.pgm")

# -----------------------------------------------
# Save YAML
# -----------------------------------------------
map_yaml = {
    "image": "track_map.png",
    "resolution": RES,
    "origin": [float(min_x), float(min_y), 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}

with open("track_map.yaml", "w") as f:
    yaml.dump(map_yaml, f)
