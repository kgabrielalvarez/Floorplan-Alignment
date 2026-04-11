import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.widgets import Slider
import pandas as pd
import numpy as np

######################################## FUNCTION DEFINITION ########################################

# Z-axis rotation matrix
def Rz(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])
    
################################################ MAIN ###############################################

# Floorplan file path
floorplan_path = "../floorplans/masks_no_windows/floor_1.png"

# Trajectory file path
trajectory_path = "../trajectory_recorder/trajectories/test_trajectory_V6.csv" # <-- generated from floor_1_2025-05-05_run_1
#trajectory_path = "../trajectory_recorder/trajectories/trajectory_marius.csv"

# Load the PNG image
img = mpimg.imread(floorplan_path)

# Load trajectory
trajectory = pd.read_csv(trajectory_path)
x = trajectory["x"].to_numpy()
y = trajectory["y"].to_numpy()
z = trajectory["z"].to_numpy()

# Create 3D plot
fig = plt.figure()

# Plot raw trajectory
ax = fig.add_subplot(1, 1, 1, projection='3d')
line_raw, = ax.plot(x, y, z, label='3D-Trajectory not aligned')

# Formatting
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Calculate trajectory in standard camera frame
# Source: https://www.sciencedirect.com/science/article/abs/pii/S0926580521001746

# Covariance matrix of poses in nonstandard camera frame
n = len(x)
M_x = sum(x)/n
M_y = sum(y)/n
M_z = sum(z)/n
x_dev = x - M_x
y_dev = y - M_y
z_dev = z - M_z
B = np.vstack((x_dev, y_dev, z_dev))
S = 1/(n-1)*np.matmul(B, np.transpose(B))
print(S)

# Eigenvalue decomposition
S_prime, P = np.linalg.eig(S)

# Try different arrangements of P
vx = P[:,0].reshape(-1, 1)
vy = P[:,1].reshape(-1, 1)
vz = P[:,2].reshape(-1, 1)
# P_reordered = np.hstack((vx, vy, vz))
# P_reordered = np.hstack((vx, vz, vy))
P_reordered = np.hstack((vy, vx, vz)) # !!!!
# P_reordered = np.hstack((vy, vz, vx))
# P_reordered = np.hstack((vz, vx, vy))
# P_reordered = np.hstack((vz, vy, vx))

# Poses in standard camera frame
G = np.transpose(P_reordered) @ np.vstack((x, y, z))
x_aligned = G[0, :]
y_aligned = G[1, :]
z_aligned = G[2, :]

# Plot trajectory in standard camera frame
line_aligned, = ax.plot(x_aligned, y_aligned, z_aligned, label='3D-Trajectory aligned')

# Validate that the covariance matrix is indeed diagonal
M_x_aligned = sum(x_aligned)/n
M_y_aligned = sum(y_aligned)/n
M_z_aligned = sum(z_aligned)/n
x_dev_aligned = x_aligned - M_x_aligned
y_dev_aligned = y_aligned - M_y_aligned
z_dev_aligned = z_aligned - M_z_aligned
C = np.vstack((x_dev_aligned, y_dev_aligned, z_dev_aligned))
S_prime_validation = 1/(n-1) * C @ np.transpose(C)
print(S_prime_validation)

# Plot on 2D-plane
line_2d_raw, = ax.plot(x, y, np.zeros(n), label='2D-Trajectory not aligned')
line_2d_aligned, = ax.plot(x_aligned, y_aligned, np.zeros(n), label='2D-Trajectory aligned')

# Show legend
ax.legend()

# Set top-down view
ax.view_init(elev=90, azim=-90)

# Add slider to change theta
ax_slider = plt.axes([0.2, 0.02, 0.6, 0.03])
slider = Slider(ax_slider, 'Theta', 0, 2*np.pi, valinit=0.0)

# Update function
def update(val):
    theta = slider.val

    G = Rz(theta) @ (P_reordered.T @ np.vstack((x, y, z)))

    x_aligned = G[0, :]
    y_aligned = G[1, :]
    z_aligned = G[2, :]

    # Update data instead of replotting
    line_aligned.set_data(x_aligned, y_aligned)
    line_aligned.set_3d_properties(z_aligned)

    line_2d_aligned.set_data(x_aligned, y_aligned)
    line_2d_aligned.set_3d_properties(np.zeros(n))

    fig.canvas.draw_idle()

# Connect slider
slider.on_changed(update)

plt.show()