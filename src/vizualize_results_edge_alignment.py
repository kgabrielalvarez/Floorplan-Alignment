import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.widgets import Slider
import pandas as pd
import numpy as np
import json

# -----------------------------------------------------------------------------
# FUNCTIONS
# -----------------------------------------------------------------------------

def R2(theta):
    """2D rotation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

# -----------------------------------------------------------------------------
# LOAD DATA
# -----------------------------------------------------------------------------

# Floorplan file path
floorplan_path = "../floorplans/masks_no_windows/floor_1.png"

# Trajectory file path
trajectory_path = "../trajectory_recorder/trajectories/trajectory_marius.csv" # "../poses_csv_files/poses_4_7_26_v1.csv"

# Load floorplan image
img = mpimg.imread(floorplan_path)
img_h, img_w = img.shape[:2]

# Load trajectory
trajectory = pd.read_csv(trajectory_path)
x = trajectory["x"].to_numpy()
y = trajectory["y"].to_numpy()
z = trajectory["z"].to_numpy()

# load orientation saved through ROS subscriber
with open('../trajectory_recorder/trajectories/orientations/orientation.json', 'r') as file:
    f = json.load(file)
    theta0 = f["yaw_deg"]
    start_position = np.array(f["translation_xyz"])

# Floorplan edges data
floorplan_edges_path = "../floorplans_csv_files/floor_1.csv"
floorplan_edges_csv = pd.read_csv(floorplan_edges_path)
floorplan_edges = floorplan_edges_csv[["x1","y1","x2","y2"]].to_numpy()

# The PNG and the DXF files have different origins so we need to align
# them relative to each other, the alignment doesn't need to be perfect 
# since the edges will be used for the final alignment, but it needs to
# get us close enough.  We figure out the translation by looking at a
# distinctive feature like a column corner.
column_corner = np.array([4051.5, 2158.5])

# -----------------------------------------------------------------------------
# GLOBAL ALIGNMENT USING TF_STATIC
# -----------------------------------------------------------------------------

theta0 = np.radians(theta0)

scale0 = 100.0 # from git repo

start_position_scaled = start_position * int(scale0)

# RAW TRAJECTORY
raw_traj = np.vstack((x,y))
raw_traj = (R2(theta0)@raw_traj)*scale0
x = raw_traj[0]
y = raw_traj[1]
x = x - x[0] + start_position_scaled[0]
y = y - y[0] + start_position_scaled[1]

# -----------------------------------------------------------------------------
# CREATE 2D FIGURE
# -----------------------------------------------------------------------------

fig2d, ax2d = plt.subplots(figsize=(10, 8))
plt.subplots_adjust(left=0.1, bottom=0.30)

# Show floorplan
ax2d.imshow(np.flipud(img), origin="lower")

# Compute initial transformed trajectory
line_2d_raw, = ax2d.plot(x, y, label='2D-Trajectory')

start_pt = ax2d.scatter(x[0], y[0], marker="o", s=60, label="Start")
end_pt = ax2d.scatter(x[-1], y[-1], marker="x", s=60, label="End")
column_corner_pt = ax2d.scatter(column_corner[0], column_corner[1], marker="x", s=60, label="Column Corner")

# axis formatting
ax2d.set_title("2D Floorplan + Trajectory Overlay")
ax2d.set_xlabel("Image X [pixels]")
ax2d.set_ylabel("Image Y [pixels]")

ax2d.set_aspect("equal")
ax2d.legend()

# -----------------------------------------------------------------------------
# CREATE 3D FIGURE
# -----------------------------------------------------------------------------

# Create Figure
fig3d = plt.figure()
ax3d = fig3d.add_subplot(projection = '3d')
ax3d.set_xlabel('x-axis [m]')
ax3d.set_ylabel('y-axis [m]')
ax3d.set_zlabel('z-axis [m]')

# Visualize poses
ax3d.scatter(x, y, z)

for edge in floorplan_edges:
    x = [edge[0], edge[2]]
    y = [edge[1], edge[3]]
    z = [0, 0]
    ax3d.plot(x, y, z, color='black')

plt.show()