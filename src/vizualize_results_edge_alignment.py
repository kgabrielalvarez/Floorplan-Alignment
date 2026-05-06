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
    
    # Scaling factor for rays
    ray_scale = 2000.0
    
    return np.array([[x_p*ray_scale], [y_p*ray_scale], [z_p*ray_scale], [1.0]])

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
    
# Update ray plot
def updateRays(val):
    idx = int(slider.val)
    t, r1, r2, be = ray_pairs[idx]

    ray1_viz.set_data([t[0], r1[0]], [t[1], r1[1]])
    ray1_viz.set_3d_properties([t[2], r1[2]])

    ray2_viz.set_data([t[0], r2[0]], [t[1], r2[1]])
    ray2_viz.set_3d_properties([t[2], r2[2]])
    
    best_edge_viz.set_data([be[0], be[2]], [be[1], be[3]])
    best_edge_viz.set_3d_properties([0, 0])

    fig3d.canvas.draw_idle()
    
# Check that segment is within angle range
def angleRangeCheck(edge, ray1_vector, ray2_vector, plane_point):
    
    # Angles of rays
    theta_ray1 = math.atan2(ray1_vector[1], ray1_vector[0])
    theta_ray2 = math.atan2(ray2_vector[1], ray2_vector[0])
    
    # Angles of vertices
    theta_vertex1 = math.atan2(edge[1] - plane_point[1], edge[0] - plane_point[0])
    theta_vertex2 = math.atan2(edge[3] - plane_point[1], edge[2] - plane_point[0])
    
    # Edge case where we are above and below pi (version 1)
    if (theta_ray1 > math.pi/2 and theta_ray2 < -math.pi/2):
        if (theta_vertex1 > theta_ray1 or theta_vertex1 < theta_ray2):
            return 1
        if (theta_vertex2 > theta_ray1 or theta_vertex2 < theta_ray2):
            return 1
    
    # Edge case where we are above and below pi (version 2)
    elif (theta_ray2 > math.pi/2 and theta_ray1 < -math.pi/2):
        if (theta_vertex1 > theta_ray2 or theta_vertex1 < theta_ray1):
            return 1
        if (theta_vertex2 > theta_ray2 or theta_vertex2 < theta_ray1):
            return 1
    
    # Standard cases
    else:
        if ((theta_vertex1 >= theta_ray1 and theta_vertex1 < theta_ray2) or
            (theta_vertex1 < theta_ray1 and theta_vertex1 >= theta_ray2)):
            return 1
        if ((theta_vertex2 >= theta_ray1 and theta_vertex2 < theta_ray2) or
            (theta_vertex2 < theta_ray1 and theta_vertex2 >= theta_ray2)):
            return 1
    
    return 0

# -----------------------------------------------------------------------------
# LOAD DATA
# -----------------------------------------------------------------------------

# Floorplan file path
floorplan_path = "../floorplans/masks_no_windows/floor_1.png"

# Trajectory file path
trajectory_path = "../poses_csv_files/poses_4_22_26.csv"

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
edges_path = "../lines_csv_files/lines_4_22_26.csv"
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

T_imu_cam = np.eye(4)
T_imu_cam[:3, :3] = T_cam_imu[:3, :3].T
T_imu_cam[:3, 3] = -T_cam_imu[:3, :3].T @ T_cam_imu[:3, 3]

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
for idx in range(0, len(pose_translations), 15):
    
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

ray_pairs = []

for edge in edges:
    
    # Timestamp
    timestap = edge[0]
    
    # Rays forming the plane containing the edge
    ray1_cam = calculateRay(edge[1], edge[2])
    ray2_cam = calculateRay(edge[3], edge[4])
    
    # Extrinsics
    # T_imu_wrld: IMU frame <-- world frame
    idx = np.argmin(np.abs(pose_timestamps - timestap))
    quat = pose_quats[idx, :]
    R_wrld_imu = R_rotwrld_wrld @ Rotation.from_quat(quat).as_matrix()
    translation_wrld_imu = pose_translations[idx, :].reshape(3, 1)
    T_wrld_imu = np.eye(4)
    T_wrld_imu[:3, :3] = R_wrld_imu
    T_wrld_imu[:3, 3] = translation_wrld_imu.flatten()
    
    # T_wrld_cam: world frame <-- camera frame
    T_wrld_cam = T_wrld_imu @ T_imu_cam
    translation_wrld_cam = T_wrld_cam[:3, 3]
    
    # Tranform the ray
    ray1_wrld = T_wrld_cam @ ray1_cam
    ray2_wrld = T_wrld_cam @ ray2_cam
    # Keep only the first 3x elements
    ray1_wrld = ray1_wrld[:3, 0]
    ray2_wrld = ray2_wrld[:3, 0]
    
    # Calculate plane formed by rays
    ray1_vector = ray1_wrld - translation_wrld_cam
    ray2_vector = ray2_wrld - translation_wrld_cam
    plane_normal = np.cross(ray1_vector, ray2_vector)
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    plane_point = translation_wrld_cam
    
    # Find the closest floorplan edge
    best_alignment_score = math.inf
    best_distance_score = math.inf
    best_edge = np.array([0, 0, 0, 0])
    for edge in floorplan_edges:
        
        floorplan_edge_vector = np.array([edge[2] - edge[0], 
                                          edge[3] - edge[1],
                                          0])
        floorplan_edge_vector = floorplan_edge_vector / np.linalg.norm(floorplan_edge_vector)
        
        # Check that segment is within angle range
        if angleRangeCheck(edge, ray1_vector, ray2_vector, translation_wrld_cam):
        
            # Check alignment
            alignment_score = abs(np.dot(plane_normal, floorplan_edge_vector))
            if alignment_score <= best_alignment_score:
                
                # Edge is aligned so check distance
                dist_1 = abs(np.dot(plane_normal, np.array([edge[0], edge[1], 0]) - plane_point))
                dist_2 = abs(np.dot(plane_normal, np.array([edge[2], edge[3], 0]) - plane_point))
                distance_score = dist_1 + dist_2
                if distance_score < best_distance_score:
                    
                    # Update best choice of segment
                    best_edge = edge
                    best_alignment_score = alignment_score
                    best_distance_score = distance_score
        
    # Save rays
    ray_pairs.append((translation_wrld_cam, ray1_wrld, ray2_wrld, best_edge))
    
# Plot first pair of rays
t, r1, r2, be = ray_pairs[0]
ray1_viz, = ax3d.plot([t[0], r1[0]],
                      [t[1], r1[1]],
                      [t[2], r1[2]], color='k')
ray2_viz, = ax3d.plot([t[0], r2[0]],
                      [t[1], r2[1]],
                      [t[2], r2[2]], color='k')
best_edge_viz, = ax3d.plot([be[0], be[2]],
                           [be[1], be[3]],
                           [0, 0], color = 'r')

# Add slider to step between rays
from matplotlib.widgets import Slider
ax_slider = plt.axes([0.2, 0.02, 0.6, 0.03])
slider = Slider(ax_slider, 'Ray Index', 0, len(ray_pairs)-1, valinit=0, valstep=1)

slider.on_changed(updateRays)

# -----------------------------------------------------------------------------
# CALCULATE ROTATION CORRECTION
# -----------------------------------------------------------------------------

print('Number of edges considered = ', len(ray_pairs))
possible_theta_corrections = np.linspace(-20, 20, 81) * np.pi/180
best_thetas = np.zeros(len(ray_pairs))
for idx in range(len(ray_pairs)):
    
    # Raw data
    pose_origin, ray_1, ray_2, edge = ray_pairs[idx]
    
    # Calculate vectors
    ray_vector_1 = ray_1 - pose_origin
    ray_vector_2 = ray_2 - pose_origin
    edge_vector = np.array([edge[2]-edge[0],
                            edge[3]-edge[1],
                            0])
    
    # Consider all possible rotations
    best_score = np.inf
    for possible_theta in possible_theta_corrections:
        possible_rot = np.array([[np.cos(possible_theta), -np.sin(possible_theta), 0],
                                 [np.sin(possible_theta), np.cos(possible_theta), 0],
                                 [0, 0, 1]])
        rotated_ray_vector_1 = possible_rot @ ray_vector_1
        rotated_ray_vector_2 = possible_rot @ ray_vector_2
        plane_normal = np.cross(rotated_ray_vector_1, rotated_ray_vector_2)
        alignment_score = abs(np.dot(edge_vector, plane_normal))
        if alignment_score < best_score:
            best_score = alignment_score
            best_thetas[idx] = possible_theta
            
# Average theta
theta_correction = np.average(best_thetas)
print('Theta correction = ', theta_correction*180/np.pi, ' deg')

# Visualize rotation corrections
rotation_correction_plot = plt.figure()
ax_rotation_correction_plot = rotation_correction_plot.add_subplot()
ax_rotation_correction_plot.set_xlabel('ray pair index')
ax_rotation_correction_plot.set_ylabel('angle [deg]')
ray_indices = np.linspace(0, len(ray_pairs), len(ray_pairs))
ax_rotation_correction_plot.scatter(ray_indices, best_thetas*180/np.pi)

# -----------------------------------------------------------------------------
# CALCULATE TRANSLATION CORRECTION
# -----------------------------------------------------------------------------

ray_vectors_1 = []
ray_vectors_2 = []
rotated_ray_vectors_1 = []
rotated_ray_vectors_2 = []
edge_vectors = []
edge_vertices = []
ray_origin_vertices = []
ray_origin_vertices_rotated = []
rotation_matrix = np.array([[np.cos(theta_correction), -np.sin(theta_correction), 0],
                            [np.sin(theta_correction), np.cos(theta_correction), 0],
                            [0, 0, 1]])
for idx in range(len(ray_pairs)):
    
    # Load data
    pose_origin, ray_1, ray_2, edge = ray_pairs[idx]
    
    # Define vectors
    pose_origin_rotated = rotation_matrix @ pose_origin.T
    ray_vector_1 = ray_1 - pose_origin
    ray_vector_2 = ray_2 - pose_origin
    rotated_ray_vector_1 = rotation_matrix @ ray_vector_1
    rotated_ray_vector_2 = rotation_matrix @ ray_vector_2
    edge_vector = np.array([edge[2]-edge[0],
                            edge[3]-edge[1],
                            0])
    
    # Check that we have a valid edge
    if np.all(edge_vector == 0):
        continue
    
    edge_vertex = np.array([edge[0], edge[1], 0])
    
    # Save vectors
    ray_vectors_1.append(ray_vector_1)
    ray_vectors_2.append(ray_vector_2)
    rotated_ray_vectors_1.append(rotated_ray_vector_1)
    rotated_ray_vectors_2.append(rotated_ray_vector_2)
    edge_vectors.append(edge_vector)
    edge_vertices.append(edge_vertex)
    ray_origin_vertices.append(pose_origin)
    ray_origin_vertices_rotated.append(pose_origin_rotated)
    
possible_x_translations = np.linspace(-250, 250, 3)
possible_y_translations = np.linspace(-250, 250, 3)
possible_z_translations = np.linspace(-500.0, 500.0, 11)
best_total_dists = np.inf
for delta_x in possible_x_translations:
    for delta_y in possible_y_translations:
        for delta_z in possible_z_translations:
            
            total_dists = 0
            for idx_v in range(len(rotated_ray_vectors_1)):
                
                # Define ray and edge vectors
                rotated_ray_vector_1 = rotated_ray_vectors_1[idx_v]
                rotated_ray_vector_2 = rotated_ray_vectors_2[idx_v]
                edge_vector = edge_vectors[idx_v]
                
                # Plane vectors
                plane_vector_1 = np.cross(rotated_ray_vector_1, edge_vector)
                plane_vector_2 = np.cross(rotated_ray_vector_2, edge_vector)
            
                # Points on the planes
                vertex_edge = edge_vertices[idx_v]
                ray_origin_vertex = ray_origin_vertices_rotated[idx_v] + np.array([delta_x, delta_y, delta_z])
                
                # Distances between the planes
                # Based on the calculation of the distance between a line and a line described here:
                # https://math.stackexchange.com/questions/210848/finding-the-shortest-distance-between-two-lines
                D_1_edge = -np.dot(plane_vector_1, vertex_edge)
                D_1_ray = -np.dot(plane_vector_1, ray_origin_vertex)
                D_2_edge = -np.dot(plane_vector_2, vertex_edge)
                D_2_ray = -np.dot(plane_vector_2, ray_origin_vertex)
                dist_1 = abs(D_1_edge - D_1_ray)/np.linalg.norm(plane_vector_1)
                dist_2 = abs(D_2_edge - D_2_ray)/np.linalg.norm(plane_vector_2)
                total_dists = total_dists + dist_1 + dist_2
                
            if total_dists < best_total_dists:
                best_total_dists = total_dists
                best_delta_x = delta_x
                best_delta_y = delta_y
                best_delta_z = delta_z
                
print('delta x = ', best_delta_x, ', delta y = ', best_delta_y, ', delta z = ', best_delta_z)
translation_vector = np.array([best_delta_x, best_delta_y, best_delta_z])

# -----------------------------------------------------------------------------
# PLOT ORIGINAL AND CORRECTED PATHS WITH RAYS
# -----------------------------------------------------------------------------

fig_rays_raw = plt.figure()
ax_rays_raw = fig_rays_raw.add_subplot(projection = '3d')
ax_rays_raw.set_xlabel('x-axis')
ax_rays_raw.set_ylabel('y-axis')
ax_rays_raw.set_zlabel('z-axis')
ax_rays_raw.set_title('Before rotation and translation')

fig_rays_corrected = plt.figure()
ax_rays_corrected = fig_rays_corrected.add_subplot(projection = '3d')
ax_rays_corrected.set_xlabel('x-axis')
ax_rays_corrected.set_ylabel('y-axis')
ax_rays_corrected.set_zlabel('z-axis')
ax_rays_corrected.set_title('After rotation and translation')

# Floorplan edges
for edge in floorplan_edges:
    x = [edge[0], edge[2]]
    y = [edge[1], edge[3]]
    z = [0, 0]
    ax_rays_raw.plot(x, y, z, color = 'black')
    ax_rays_corrected.plot(x, y, z, color = 'black')
    
# Path uncorrected
ax_rays_raw.plot(pose_translations[:, 0],
                 pose_translations[:, 1],
                 pose_translations[:, 2],
                 color = 'red')
pose_translations_rotated = rotation_matrix @ pose_translations.T + translation_vector[:, None]
ax_rays_corrected.plot(pose_translations_rotated[0, :],
                       pose_translations_rotated[1, :],
                       pose_translations_rotated[2, :],
                       color = 'red')

# Set axis equal
setAxisEqual(ax_rays_raw)
setAxisEqual(ax_rays_corrected)

# Rays
ray_origin_vertices_rotated = np.array(ray_origin_vertices_rotated) + translation_vector
for idx in range(0, len(ray_vectors_1), 400):
    
    # Uncorrected
    ray_origin_vertex = ray_origin_vertices[idx]
    ray_vector_1 = ray_vectors_1[idx]
    ray_vector_2 = ray_vectors_2[idx]
    x_1 = [ray_origin_vertex[0], ray_origin_vertex[0] + ray_vector_1[0]]
    y_1 = [ray_origin_vertex[1], ray_origin_vertex[1] + ray_vector_1[1]]
    z_1 = [ray_origin_vertex[2], ray_origin_vertex[2] + ray_vector_1[2]]
    x_2 = [ray_origin_vertex[0], ray_origin_vertex[0] + ray_vector_2[0]]
    y_2 = [ray_origin_vertex[1], ray_origin_vertex[1] + ray_vector_2[1]]
    z_2 = [ray_origin_vertex[2], ray_origin_vertex[2] + ray_vector_2[2]]
    ax_rays_raw.plot(x_1, y_1, z_1, color = 'green')
    ax_rays_raw.plot(x_2, y_2, z_2, color = 'green')
    
    # Corrected
    ray_origin_vertex = ray_origin_vertices_rotated[idx]
    ray_vector_1 = rotated_ray_vectors_1[idx]
    ray_vector_2 = rotated_ray_vectors_2[idx]
    x_1 = [ray_origin_vertex[0], ray_origin_vertex[0] + ray_vector_1[0]]
    y_1 = [ray_origin_vertex[1], ray_origin_vertex[1] + ray_vector_1[1]]
    z_1 = [ray_origin_vertex[2], ray_origin_vertex[2] + ray_vector_1[2]]
    x_2 = [ray_origin_vertex[0], ray_origin_vertex[0] + ray_vector_2[0]]
    y_2 = [ray_origin_vertex[1], ray_origin_vertex[1] + ray_vector_2[1]]
    z_2 = [ray_origin_vertex[2], ray_origin_vertex[2] + ray_vector_2[2]]
    ax_rays_corrected.plot(x_1, y_1, z_1, color = 'green')
    ax_rays_corrected.plot(x_2, y_2, z_2, color = 'green')

# -----------------------------------------------------------------------------
# SHOW ALL PLOTS
# -----------------------------------------------------------------------------
    
plt.show()