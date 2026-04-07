import matplotlib.pyplot as plt
import cv2
from scipy.spatial.transform import Rotation
import pandas as pd
import numpy as np
import math

# ---------------------- LOAD DATA ----------------------

# Poses file path
poses_file_path = "../poses_csv_files/posesv1.csv"

# Lines file path
lines_file_path = "../lines_csv_files/linesv1.csv"

# Load poses
poses = pd.read_csv(poses_file_path)
pose_times = poses["t"].to_numpy()
pose_translations = poses[["x", "y", "z"]].to_numpy()
pose_quats = poses[["qx", "qy", "qz", "qw"]].to_numpy()

# Load keylines
keylines = pd.read_csv(lines_file_path)
keyline_times1 = keylines["t1"]
keyline_points1 = keylines[["startX1", "startY1", "endX1", "endY1"]].to_numpy()
keyline_times2 = keylines["t2"]
keyline_points2 = keylines[["startX2", "startY2", "endX2", "endY2"]].to_numpy()

# T_cam_imu: camera frame <-- IMU frame (for cam0, pinhole model)
# Sources:
# 1. Extrinsics: Floorplan-Alignment/intrinsics/kalibr_imucam_chain.yaml
# 2. Units: meters (https://www.youtube.com/watch?app=desktop&v=puNXsnrYWTY)
T_cam_imu = np.array([[0.017214474772216132, -0.0008034642120502422, -0.9998514971252359, 0.020670851120764513],
                      [0.9998263174555488, -0.007128426214556394, 0.017219769539067287, 0.015539085669546057],
                      [-0.007141203091335369, -0.9999742696614562, 0.0006806125511055194, -0.01575188948566258],
                      [0.0, 0.0, 0.0, 1.0]])

# Intrinsics (for cam0, pinhole model)
# Sources: 
# 1. "Computer Vision: Algorithms and Applications" (Szelesky), page 58
# 2. Intrinscs: Floorplan-Alignment/intrinsics/kalibr_imucam_chain.yaml
fu = 465.3015482593691      # pixels
fv = 465.32303798346413     # pixels
pu = 730.0455886686005      # pixels
pv = 720.1427007671206      # pixels
K = np.array([[fu, 0, pu],
              [0, fv, pv],
              [0, 0, 1]])

# Pixels/m
width_pixels = 1472 # [pixels]
height_pixels = 1440 # [pixels]
diagonal_pixels = math.sqrt(width_pixels**2 + height_pixels**2) # [pixels]
diagonl_m = 1*0.0254 # [m]
pixels_per_m = diagonal_pixels/diagonl_m # [pixels/m]

# ---------------------- PROJECTION MATRIX ----------------------

def CalculateProjectionMatrix(time):
    
    # Index corresponding to t
    idx = np.argmin(np.abs(pose_times - time))
    
    # T_imu_wrld: IMU frame <-- world frame
    quat = pose_quats[idx, :]
    R_wrld_imu = Rotation.from_quat(quat).as_matrix()
    translation_wrld_imu = pose_translations[idx, :].reshape(3, 1)
    R_imu_wrld = R_wrld_imu.T
    translation_imu_wrld = -R_wrld_imu.T @ translation_wrld_imu
    T_imu_wrld = np.eye(4)
    T_imu_wrld[:3, :3] = R_imu_wrld
    T_imu_wrld[:3, 3] = translation_imu_wrld.flatten()
    
    # T_cam_wrld: camera frame <-- world frame
    T_cam_wrld = T_cam_imu @ T_imu_wrld
    
    # Convert translation in T_cam_wrld from [m] to [pixels]
    T_cam_wrld[:3, 3] = T_cam_wrld[:3, 3] * pixels_per_m
    
    # Full transformation matrix (4x4)
    # Source: "Computer Vision: Algorithms and Applications" (Szelesky), page 60
    projMatr = K @ T_cam_wrld[:3, :]
    
    # Return projMatr
    return projMatr.astype(np.float64)

# ---------------------- VISUALIZE POSES ----------------------

# Create Figure
fig = plt.figure()
ax = fig.add_subplot(projection = '3d')
ax.set_xlabel('x-axis [m]')
ax.set_ylabel('y-axis [m]')
ax.set_zlabel('z-axis [m]')

# Visualize poses
ax.scatter(pose_translations[:,0], pose_translations[:,1], pose_translations[:,2])

# ---------------------- TRIANGULATE KEYLINES ----------------------

# Number of keylines to consider
N = 200
N_start = 7120

# Indices
indices = np.linspace(N_start, N_start + N-1, N, dtype=np.int32)

# Sources:
# 1. Triangulate Points function: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html?utm_source=chatgpt.com#gad3fc9a0c82b08df034234979960b778c
for idx in indices:
    
    # Time at which each keyline was captured
    t1 = keyline_times1[idx]
    t2 = keyline_times2[idx]
    
    # Frame 1 keyline start and end points
    startX1 = keyline_points1[idx, 0]
    startY1 = keyline_points1[idx, 1]
    endX1 = keyline_points1[idx, 2]
    endY1 = keyline_points1[idx, 3]
    projPoints1 = np.array([[startX1, startY1],
                            [endX1, endY1]], 
                            dtype=np.float64)
    
    # Frame 2 keyline start and end points
    startX2 = keyline_points2[idx, 0]
    startY2 = keyline_points2[idx, 1]
    endX2 = keyline_points2[idx, 2]
    endY2 = keyline_points2[idx, 3]
    projPoints2 = np.array([[startX2, startY2],
                            [endX2, endY2]],
                            dtype=np.float64)
    
    # Projection matrix for frame 1
    projMatr1 = CalculateProjectionMatrix(t1)
    
    # Projection matrix for frame 2
    projMatr2 = CalculateProjectionMatrix(t2)
    
    # Variable to store solution
    line_3d = np.zeros((4,2))
    
    # Triangulate
    cv2.triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2, line_3d)
    
    # Convert from homogeneous to euclidean
    p1 = line_3d[:3, 0] / line_3d[3, 0] # [pixels]
    p2 = line_3d[:3, 1] / line_3d[3, 1] # [pixels]
    # Convert fromp pixels to m
    p1 = p1*pixels_per_m**(-1) # [m]
    p2 = p2*pixels_per_m**(-1) # [m]
    
    # Visualize 3D lines
    x = [p1[0], p2[0]]
    y = [p1[1], p2[1]]
    z = [p1[2], p2[2]]
    ax.plot(x, y, z)

# ---------------------- SHOW PLOT ----------------------

plt.show()