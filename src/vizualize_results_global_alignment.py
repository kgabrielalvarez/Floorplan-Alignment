import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.widgets import Slider
import pandas as pd
import numpy as np
import json



######################################## FUNCTIONS ########################################

def R2(theta):
    """2D rotation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

def estimate_theta_from_correspondences(src_xy, dst_xy):
    """
    Estimate 2D rotation R such that:
        dst_xy ≈ R @ src_xy
    src_xy, dst_xy: shape (2, N)
    """
    H = src_xy @ dst_xy.T
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Avoid reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    theta = np.arctan2(R[1, 0], R[0, 0])
    return theta, R


######################################## MAIN ########################################

# Floorplan file path
floorplan_path = "../floorplans/masks_no_windows/floor_1.png"

# Trajectory file path
trajectory_path = "../trajectory_recorder/trajectories/trajectory_marius.csv"

# Ground Truth data
gt_path = "../trajectory_recorder/ground_truth/floor_1_2025-05-05_run_1.txt"

# Load floorplan image
img = mpimg.imread(floorplan_path)
img_h, img_w = img.shape[:2]

# Load trajectory
trajectory = pd.read_csv(trajectory_path)
x = trajectory["x"].to_numpy()
y = trajectory["y"].to_numpy()
z = trajectory["z"].to_numpy()

# Load gt
gt = pd.read_csv(gt_path, sep=r"\s+", comment="#", header=None, names=["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
gt_timestamps=gt["timestamp"].to_numpy()
gt_tx = gt["tx"].to_numpy()
gt_ty = gt["ty"].to_numpy()
start_idx = np.argmax(gt_timestamps>=10005)


# -----------------------------------------------------------------------------
# STEP 1: PCA-based alignment in 3D
# -----------------------------------------------------------------------------
print(f"trajectory_start pos: x {x[0]}, y {y[0]}")
n = len(x)

# Mean
Mx = np.mean(x)
My = np.mean(y)
Mz = np.mean(z)

# Deviations
x_dev = x - Mx
y_dev = y - My
z_dev = z - Mz

# Covariance matrix
B = np.vstack((x_dev, y_dev, z_dev))
S = (1 / (n - 1)) * (B @ B.T)
print("Covariance matrix S:")
print(S)

# Eigenvalue decomposition
eigenvalues, P = np.linalg.eig(S)

# Reorder eigenvectors exactly like in your current script
vx = P[:, 0].reshape(-1, 1)
vy = P[:, 1].reshape(-1, 1)
vz = P[:, 2].reshape(-1, 1)
P_reordered = np.hstack((vy, vx, vz))

# Transform into aligned frame
G = P_reordered.T @ np.vstack((x, y, z))
x_aligned = G[0, :]
y_aligned = G[1, :]

# Center 2D trajectory around its own centroid
x_center = np.mean(x_aligned)
y_center = np.mean(y_aligned)
traj_2d = np.vstack((x_aligned - x_center, y_aligned - y_center))


# estimate rotation due to PCA
# Raw trajectory in local coordinates
raw_local = np.vstack((
    x - x[0],
    y - y[0]
))

# PCA trajectory in local coordinates
pca_local = np.vstack((
    x_aligned - x_aligned[0],
    y_aligned - y_aligned[0]
))

theta_pca_to_raw, R_pca_to_raw = estimate_theta_from_correspondences(pca_local, raw_local)


# -----------------------------------------------------------------------------
# STEP 2: Align the map with the actual size
# -----------------------------------------------------------------------------


# load orientation saved through ROS subscriber
with open('../trajectory_recorder/trajectories/orientations/orientation.json', 'r') as file:
    f = json.load(file)
    theta0 = f["yaw_deg"]
    print(f"YAW: {theta0}°")
    start_position = np.array(f["translation_xyz"])
theta0 = np.radians(theta0)


scale0 = 100.0 # from git repo

start_position_scaled = start_position * int(scale0)
print(f"Start pos gt: {start_position}")
print(f"Start position of pca flattened trajectory: {traj_2d[0,0], traj_2d[1,0]}")

# ALIGNED TRAJECTORY
traj_2d = scale0 * (R2(theta0+theta_pca_to_raw) @ traj_2d)
# remove PCA centering (centered around mean but not at start) + offset to start position of gt
traj_2d[0,:] = traj_2d[0,:] - traj_2d[0,0] + start_position_scaled[0]
traj_2d[1,:] = traj_2d[1,:] - traj_2d[1,0] + start_position_scaled[1]
traj_init = traj_2d

# RAW TRAJECTORY
raw_traj = np.vstack((x,y))
raw_traj = (R2(theta0)@raw_traj)*scale0
x = raw_traj[0]
y = raw_traj[1]
x = x - x[0] + start_position_scaled[0]
y = y - y[0] + start_position_scaled[1]


# -----------------------------------------------------------------------------
# STEP 3: Create 2D figure
# -----------------------------------------------------------------------------

fig, ax = plt.subplots(figsize=(10, 8))
plt.subplots_adjust(left=0.1, bottom=0.30)

# Show floorplan
ax.imshow(np.flipud(img), origin="lower")
#ax.imshow(img, origin="upper")



# Compute initial transformed trajectory

print(traj_init[0],traj_init[1])
x_plot = traj_init[0, :]
y_plot = traj_init[1, :]

# Plot trajectory
line_traj, = ax.plot(x_plot, y_plot, linewidth=2, label="Aligned trajectory")
line_2d_raw, = ax.plot(x, y, label='2D-Trajectory not aligned')

start_pt = ax.scatter(x_plot[0], y_plot[0], marker="o", s=60, label="Start")
start_gt = ax.scatter(start_position_scaled[0],start_position_scaled[1], marker="p",s=60, label="Start_gt")
end_pt = ax.scatter(x_plot[-1], y_plot[-1], marker="x", s=60, label="End")

# Axis formatting
ax.set_title("2D Floorplan + Trajectory Overlay")
ax.set_xlabel("Image X [pixels]")
ax.set_ylabel("Image Y [pixels]")

ax.set_aspect("equal")
ax.legend()

plt.show()