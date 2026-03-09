import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pandas as pd

# Floorplan file path
floorplan_path = "../floorplans/masks_no_windows/floor_1.png"

# Trajectory file path
trajectory_path = "../trajectory_recorder/trajectories/test_trajectory.csv" # <-- generated from floor_1_2025-05-05_run_1

# Load the PNG image
img = mpimg.imread(floorplan_path)

# Load trajectory
trajectory = pd.read_csv(trajectory_path)
scale = 100 # CONFIRM THAT THIS IS CORRECT!!!
trajectory['x'] = trajectory['x']*scale
trajectory['y'] = trajectory['y']*scale

# Display the image
plt.imshow(img)
plt.plot(trajectory['x'], trajectory['y'], color='red', linewidth=2, label='Trajectory')
plt.axis('off')  # turn off axis labels
plt.show()

