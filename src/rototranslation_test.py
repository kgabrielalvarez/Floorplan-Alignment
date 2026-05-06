import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(precision=3)

# Define floorplan edges
floorplan_vertices = np.array([[0, 0, 0], 
                               [10, 0, 0], 
                               [15, 5, 0],
                               [10, 8, 0],
                               [0, 8, 0],
                               [0, 0, 0]])

ray_vertices = np.array([[0.2, 0.2, 0.2], 
                         [10.4, 0, 0], 
                         [14.6, 5.2, 0],
                         [9.0, 8, 0],
                         [0, 8.5, -0.3],
                         [0.2, 0.2, 0.2]])

# Rotation
theta = 30*np.pi/180
rotation = np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])
origin = np.array([5.0, 4.0, 3.0])
delta = np.array([1.5, 0.8, -1.0])
origin_translated = origin + delta

# Create 3D-plot
fig_og = plt.figure()
ax_og = fig_og.add_subplot(projection = '3d')
ax_og.set_xlabel('x-axis')
ax_og.set_ylabel('y-axis')
ax_og.set_zlabel('z-axis')
ax_og.set_title('unprocessed rays')

# Plot floorplan edges
ax_og.plot(floorplan_vertices[:,0], floorplan_vertices[:,1], floorplan_vertices[:,2], color = 'black')

# Rays
ray_vectors = []
for vertex in ray_vertices:
    ray_vector = vertex - origin
    ray_vector = rotation @ ray_vector.T
    ray_vectors.append(ray_vector)
    ray_vector_scaled = ray_vector * 3
    
    # Visualize rays
    x = [origin_translated[0], origin_translated[0]+ray_vector_scaled[0]]
    y = [origin_translated[1], origin_translated[1]+ray_vector_scaled[1]]
    z = [origin_translated[2], origin_translated[2]+ray_vector_scaled[2]]
    
    ax_og.plot(x, y, z, color = 'red')
    
# Find rotation that minimizes error
edge_vectors = []
possible_theta_corrections = np.linspace(-45, 45, 91) * np.pi/180
best_thetas = np.zeros(len(floorplan_vertices) - 1)
for idx in range(len(floorplan_vertices) - 1):
    start_vertex = floorplan_vertices[idx]
    end_vertex = floorplan_vertices[idx+1]
    edge_vector = end_vertex - start_vertex
    edge_vectors.append(edge_vector)
    
    best_score = np.inf
    for possible_theta in possible_theta_corrections:
        possible_rot = np.array([[np.cos(possible_theta), -np.sin(possible_theta), 0],
                                 [np.sin(possible_theta), np.cos(possible_theta), 0],
                                 [0, 0, 1]])
        ray_vector_1 = possible_rot @ ray_vectors[idx]
        ray_vector_2 = possible_rot @ ray_vectors[idx+1]
        plane_normal = np.cross(ray_vector_1, ray_vector_2)
        alignment_score = np.dot(edge_vector, plane_normal)
        
        if abs(alignment_score) < best_score:
            best_score = abs(alignment_score)
            best_thetas[idx] = possible_theta
            
theta_correction = np.average(best_thetas)
print('Best theta = ', theta_correction*180/np.pi)

# Create second 3D plot for rotated rays
fig_rot = plt.figure()
ax_rot = fig_rot.add_subplot(projection = '3d')
ax_rot.set_xlabel('x-axis')
ax_rot.set_ylabel('y-axis')
ax_rot.set_zlabel('z-axis')
ax_rot.set_title('rotated rays')

# Plot floorplan edges
ax_rot.plot(floorplan_vertices[:,0], floorplan_vertices[:,1], floorplan_vertices[:,2], color = 'black')

# Visualize rotated rays
rotated_ray_vectors = []
theta = theta_correction*np.pi/180
rotation_correction = np.array([[np.cos(theta_correction), -np.sin(theta_correction), 0],
                                [np.sin(theta_correction), np.cos(theta_correction), 0],
                                [0, 0, 1]])
for ray_vector in ray_vectors:
    rotated_ray_vector = rotation_correction @ ray_vector
    rotated_ray_vectors.append(rotated_ray_vector)
    x = [origin_translated[0], origin_translated[0] + rotated_ray_vector[0]]
    y = [origin_translated[1], origin_translated[1] + rotated_ray_vector[1]]
    z = [origin_translated[2], origin_translated[2] + rotated_ray_vector[2]]
    
    ax_rot.plot(x, y, z, color = 'blue')
    
# Find best translation
# Based on the calculation of the distance between a line and a line described here:
# https://math.stackexchange.com/questions/210848/finding-the-shortest-distance-between-two-lines
possible_x_translations = np.linspace(-3.0, 3.0, 61)
possible_y_translations = np.linspace(-3.0, 3.0, 61)
possible_z_translations = np.linspace(-3.0, 3.0, 61)
best_total_dists = np.inf
for delta_x in possible_x_translations:
    for delta_y in possible_y_translations:
        for delta_z in possible_z_translations:
            
            corrected_origin = origin_translated + np.array([delta_x, delta_y, delta_z])
            total_dists = 0
            for idx_v in range(len(rotated_ray_vectors)):
                rotated_ray_vector = rotated_ray_vectors[idx_v]
                
                if idx_v == 0:
                    edge_vector = edge_vectors[1]
                    
                elif idx_v == len(rotated_ray_vectors)-1:
                    edge_vector = edge_vectors[-1]
                    
                else:
                    edge_vector_1 = edge_vectors[idx_v-1]
                    edge_vector_2 = edge_vectors[idx_v]
                    plane_vector_1 = np.cross(rotated_ray_vector, edge_vector_1)
                    plane_vector_2 = np.cross(rotated_ray_vector, edge_vector_2)
                    vertex_1_edge = floorplan_vertices[idx_v-1]
                    vertex_2_edge = floorplan_vertices[idx_v]
                    D_1_edge = -np.dot(plane_vector_1, vertex_1_edge)
                    D_1_ray = -np.dot(plane_vector_1, corrected_origin)
                    D_2_edge = -np.dot(plane_vector_2, vertex_2_edge)
                    D_2_ray = -np.dot(plane_vector_2, corrected_origin)
                    dist_1 = abs(D_1_edge - D_1_ray)/np.linalg.norm(plane_vector_1)
                    dist_2 = abs(D_2_edge - D_2_ray)/np.linalg.norm(plane_vector_2)
                    total_dists = total_dists + dist_1 + dist_2
                     
            if total_dists < best_total_dists:
                best_total_dists = total_dists
                best_delta_x = delta_x
                best_delta_y = delta_y
                best_delta_z = delta_z

print('delta x = ', best_delta_x, ', delta y = ', best_delta_y, ', delta z = ', best_delta_z)

# Final corrected origin
corrected_origin = origin_translated + np.array([best_delta_x, best_delta_y, best_delta_z])

# Create third 3D plot for rotated+translated rays
fig_trans = plt.figure()
ax_trans = fig_trans.add_subplot(projection = '3d')
ax_trans.set_xlabel('x-axis')
ax_trans.set_ylabel('y-axis')
ax_trans.set_zlabel('z-axis')
ax_trans.set_title('rotated rays')

ax_trans.plot(floorplan_vertices[:,0], floorplan_vertices[:,1], floorplan_vertices[:,2], color = 'black')

# Visualize translated rays
for rotated_ray_vector in rotated_ray_vectors:
    
    x = [corrected_origin[0], corrected_origin[0] + rotated_ray_vector[0]]
    y = [corrected_origin[1], corrected_origin[1] + rotated_ray_vector[1]]
    z = [corrected_origin[2], corrected_origin[2] + rotated_ray_vector[2]]
    
    ax_trans.plot(x, y, z, color = 'green')
    
    
# Display plot
plt.show()
    
    