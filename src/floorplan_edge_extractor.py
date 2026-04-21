# Import libraries
import ezdxf
import math
import matplotlib.pyplot as plt
import csv

# Minimum length that segment must have to be kept
min_length = 40

# Load DXF file of floorplan
raw_floorplan = ezdxf.readfile('../floorplans/dxf_files/floor_1.dxf')
msp = raw_floorplan.modelspace()

# Scaling factor
scale = 100

# Save the lines in the dxf to the segmentes variable
segments = []
for entity in msp:
    
    if entity.dxftype() == "LWPOLYLINE":
        points = entity.get_points()
        
        for idx in range(len(points) - 1):
            x1, y1 = points[idx][:2]
            x2, y2 = points[idx + 1][:2]
            p1 = (x1 * scale, y1 * scale)
            p2 = (x2 * scale, y2 * scale)
            
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
    
plt.gca().set_aspect('equal', adjustable='box')
plt.show()