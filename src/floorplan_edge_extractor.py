# Import libraries
import ezdxf
import math
import matplotlib.pyplot as plt

# Minimum length that segment must have to be kept
min_length = 40

# Load DXF file of floorplan
raw_floorplan = ezdxf.readfile('../floorplans/dxf_files/floor_1.dxf')
msp = raw_floorplan.modelspace()

# Save the lines in the dxf to the segmentes variable
segments = []
for entity in msp:
    if entity.dxftype() == "POLYLINE":
        points = list(entity.points())
        
        for idx in range(len(points) - 1):
            p1 = points[idx]
            p2 = points[idx + 1]
            
            # Check length
            if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) > min_length:
                segments.append((p1, p2))
            
# Visualize segments
for seg in segments:
    x = [seg[0][0], seg[1][0]]
    y = [seg[0][1], seg[1][1]]
    plt.plot(x, y)
    
plt.gca().set_aspect('equal', adjustable='box')
plt.show()