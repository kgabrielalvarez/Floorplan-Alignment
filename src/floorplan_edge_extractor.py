# Import libraries
import ezdxf
import math
import matplotlib.pyplot as plt
import csv
import numpy as np

# Minimum length that segment must have to be kept
min_length = 40

# Load DXF file of floorplan
raw_floorplan = ezdxf.readfile('../floorplans/dxf_files/floor_1.dxf')
msp = raw_floorplan.modelspace()

# Scaling factor
scale = 100

# The PNG and the DXF files have different origins so we need to align
# them relative to each other, the alignment doesn't need to be perfect 
# since the edges will be used for the final alignment, but it needs to
# get us close enough.  We figure out the translation by looking at a
# distinctive feature like a column corner.
column_corner = np.array([-1037.5, 1832.75])
column_corner_png = np.array([4051.5, 2158.5])
offset = column_corner_png - column_corner

# Save the lines in the dxf to the segmentes variable
segments = []
for entity in msp:
    
    if entity.dxftype() == "LWPOLYLINE":
        points = entity.get_points()
        
        for idx in range(len(points) - 1):
            x1, y1 = points[idx][:2]
            x2, y2 = points[idx + 1][:2]
            p1 = (x1 * scale + offset[0], y1 * scale + offset[1])
            p2 = (x2 * scale + offset[0], y2 * scale + offset[1])
            
            # Check length
            if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) > min_length:
                segments.append((p1, p2))

# Save segments to CSV
with open("../floorplans_csv_files/floor_1.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    # Header
    writer.writerow(["x1", "y1", "x2", "y2"])
    # Data
    for seg in segments:
        p1, p2 = seg
        writer.writerow([p1[0], p1[1], p2[0], p2[1]])
            
# Visualize segments
for seg in segments:
    x = [seg[0][0], seg[1][0]]
    y = [seg[0][1], seg[1][1]]
    plt.plot(x, y)
    
# Visualize column corner
column_corner_pt = plt.scatter(column_corner[0], column_corner[1], marker="x", s=60, label="Column Corner")
    
plt.gca().set_aspect('equal', adjustable='box')
plt.show()