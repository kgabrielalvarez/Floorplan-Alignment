import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.widgets import Slider
import pandas as pd
import numpy as np
import json
import math
from scipy.spatial.transform import Rotation

# -----------------------------------------------------------------------------
# FUNCTIONS
# -----------------------------------------------------------------------------

# Rotation Matrix
def R2(theta):
    """2D rotation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    
# Calculate z coordinate of 2D point on projected surface using EUCM Model
# Source: https://hal.science/hal-01722264v1/document
def calculateRay(u, v):
    
    # Find x_p and y_p from Eqn. 11: p = K*m
    x_p = (u - pu)/fu
    y_p = (v - pv)/fv
    
    # Radius of point
    r = math.sqrt(x_p**2 + y_p**2)
    
    # Check that z is defined (Eqn. 39, which applies because alpha >= 0.5)
    if r > 1/((alpha-gamma)*beta):
        return math.nan
    
    # Solve for z (Eqn. 37)
    z_p = ((1 - alpha**2 * beta * r**2) / 
           (alpha * math.sqrt(1 - (alpha - gamma) * beta * r**2) + gamma))
    
    return np.array([x_p, y_p, z_p])

# Make axis of 3D plot equal 1:1:1
def setAxisEqual(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)
    plot_radius = 0.5*max([x_range, y_range, z_range])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# -----------------------------------------------------------------------------
# LOAD DATA
# -----------------------------------------------------------------------------

# Floorplan file path
floorplan_path = "../floorplans/masks_no_windows/floor_1.png"

# Trajectory file path
trajectory_path = "../poses_csv_files/poses_4_21_26.csv"

# Load floorplan image
img = mpimg.imread(floorplan_path)
img_h, img_w = img.shape[:2]

# Load trajectory
trajectory = pd.read_csv(trajectory_path)
pose_timestamps = trajectory["t"].to_numpy()
pose_translations = trajectory[["x", "y", "z"]].to_numpy()
pose_quats = trajectory[["qx", "qy", "qz", "qw"]].to_numpy()

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

# Load edges
edges_path = "../lines_csv_files/lines_4_21_26.csv"
edges_csv = pd.read_csv(edges_path)
edges = edges_csv[["timestamp", "startX", "startY", "endX", "endY"]].to_numpy()

# -----------------------------------------------------------------------------
# DEFINE CAMERA CONSTANTS
# -----------------------------------------------------------------------------

# Intrinsics
alpha = 0.6899954350657926
gamma = 1 - alpha
beta = 0.8911981210457725
fu = 465.2979536302252 # [px]
fv = 465.3194431883040 # [px]
pu = 730.0455886686005 # [px]
pv = 720.14270076712060 # [px]

# Extrinsics
# T_cam_imu: camera frame <-- IMU frame (for cam0, EUCM model)
# Sources:
# 1. Extrinsics: Floorplan-Alignment/intrinsics/kalibr_imucam_chain.yaml
# 2. Units: meters (https://www.youtube.com/watch?app=desktop&v=puNXsnrYWTY)
T_cam_imu = np.array([[0.017214474772216132, -0.0008034642120502422, -0.9998514971252359, 0.020670851120764513],
                      [0.9998263174555488, -0.007128426214556394, 0.017219769539067287, 0.015539085669546057],
                      [-0.007141203091335369, -0.9999742696614562, 0.0006806125511055194, -0.01575188948566258],
                      [0.0, 0.0, 0.0, 1.0]])

# -----------------------------------------------------------------------------
# PLOT RAW 3D TRAJECTORY
# -----------------------------------------------------------------------------

# Create figure
fig3d_raw = plt.figure()
ax3d_raw = fig3d_raw.add_subplot(projection = '3d')
ax3d_raw.set_xlabel('x-axis [m]')
ax3d_raw.set_ylabel('y-axis [m]')
ax3d_raw.set_zlabel('z-axis [m]')

# Visualize raw poses
for idx in range(0, len(pose_translations), 25):
    
    pose = pose_translations[idx, :]
    quat = pose_quats[idx, :]
    R_wrld_imu = Rotation.from_quat(quat).as_matrix()
    triad_x = R_wrld_imu @ np.array([[1], [0], [0]])
    triad_y = R_wrld_imu @ np.array([[0], [1], [0]])
    triad_z = R_wrld_imu @ np.array([[0], [0], [1]])
    ax3d_raw.quiver(*pose, *triad_x, color = 'r')
    ax3d_raw.quiver(*pose, *triad_y, color = 'g')
    ax3d_raw.quiver(*pose, *triad_z, color = 'b')

# Make axis equal
setAxisEqual(ax3d_raw)

# -----------------------------------------------------------------------------
# GLOBAL ALIGNMENT USING TF_STATIC
# -----------------------------------------------------------------------------

theta0 = np.radians(theta0)

scale0 = 100.0 # from git repo

start_position_scaled = start_position * int(scale0)

# RAW TRAJECTORY
raw_traj = np.vstack((pose_translations[:, 0], pose_translations[:, 1]))
raw_traj = (R2(theta0)@raw_traj)*scale0
pose_translations[:, 0] = raw_traj[0]
pose_translations[:, 1] = raw_traj[1]
pose_translations[:, 0] = pose_translations[:, 0] - pose_translations[0, 0] + start_position_scaled[0]
pose_translations[:, 1] = pose_translations[:, 1] - pose_translations[0, 1] + start_position_scaled[1]

# -----------------------------------------------------------------------------
# CREATE 2D FIGURE
# -----------------------------------------------------------------------------

fig2d, ax2d = plt.subplots(figsize=(10, 8))
plt.subplots_adjust(left=0.1, bottom=0.30)

# Show floorplan
ax2d.imshow(np.flipud(img), origin="lower")

# Compute initial transformed trajectory
line_2d_raw, = ax2d.plot(pose_translations[:, 0], pose_translations[:, 1], label='2D-Trajectory')
start_pt = ax2d.scatter(pose_translations[0, 0], pose_translations[0, 1], marker="o", s=60, label="Start")
end_pt = ax2d.scatter(pose_translations[-1, 0], pose_translations[-1, 1], marker="x", s=60, label="End")
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

# 3D rotation
R_rotwrld_wrld = np.array([[np.cos(theta0), -np.sin(theta0), 0],
                           [np.sin(theta0),  np.cos(theta0), 0],
                           [0, 0, 1]])

# Scaling factor for tria
triad_scale = 50.0

# Visualize rotated poses
for idx in range(0, len(pose_translations), 25):
    
    pose = pose_translations[idx, :]
    quat = pose_quats[idx, :]
    R_wrld_imu = R_rotwrld_wrld @ Rotation.from_quat(quat).as_matrix()
    triad_x = R_wrld_imu @ np.array([[1], [0], [0]])
    triad_y = R_wrld_imu @ np.array([[0], [1], [0]])
    triad_z = R_wrld_imu @ np.array([[0], [0], [1]])
    ax3d.quiver(*pose, *(triad_x*triad_scale), color = 'r')
    ax3d.quiver(*pose, *(triad_y*triad_scale), color = 'g')
    ax3d.quiver(*pose, *(triad_z*triad_scale), color = 'b')

# Visualize floorplan edges
for edge in floorplan_edges:
    x = [edge[0], edge[2]]
    y = [edge[1], edge[3]]
    z = [0, 0]
    ax3d.plot(x, y, z, color='black')
    
# Make axis equal
setAxisEqual(ax3d)
    
# -----------------------------------------------------------------------------
# CALCULATE PLANES IN WHICH EDGES ARE CONTAINED
# -----------------------------------------------------------------------------

for edge in edges:
    
    # Timestamp
    timestap = edge[0]
    
    # Rays forming the plane containing the edge
    ray1 = calculateRay(edge[1], edge[2])
    ray2 = calculateRay(edge[3], edge[4])
    
    # Extrinsics
    # T_imu_wrld: IMU frame <-- world frame
    idx = np.argmin(np.abs(pose_timestamps - timestap))
    quat = pose_quats[idx, :]
    R_wrld_imu = Rotation.from_quat(quat).as_matrix()
    R_rotwrld_wrld = np.array([[np.cos(theta0), -np.sin(theta0), 0],
                                [np.sin(theta0),  np.cos(theta0), 0],
                                [0, 0, 1]])
    R_wrld_imu = R_rotwrld_wrld @ R_wrld_imu
    translation_wrld_imu = pose_translations[idx, :].reshape(3, 1)
    R_imu_wrld = R_wrld_imu.T
    translation_imu_wrld = -R_wrld_imu.T @ translation_wrld_imu
    T_imu_wrld = np.eye(4)
    T_imu_wrld[:3, :3] = R_imu_wrld
    T_imu_wrld[:3, 3] = translation_imu_wrld.flatten()
    
    # T_cam_wrld: camera frame <-- world frame
    T_cam_wrld = T_cam_imu @ T_imu_wrld
    
plt.show()