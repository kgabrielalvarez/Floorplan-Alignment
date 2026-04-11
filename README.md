# Overview


- `src/visualize_results_global_alignment.py`: loads the floorplan PNG and overlays the recorded trajectory in 2D.

- `trajectory_recorder/src/save_tf.py`: listens to the ROS static transform `map -> global`, saves it once published, and exits.

The idea was:
- get the alignment transform from ROS, by running a  subscriber which reads information from HILI's TF publisher -> start position + Openvins estimated orientation
- then use that transform later to align the raw trajectory with the floorplan.

# How to run the TF subscriber

Run the subscribers: (Note: make sure it runs before OpenVINS is brought up) - Orientation subscriber integrated in the file that runs the whole thing simultaneously (./run_slam.sh)

Then run:

```bash
ros2 run trajectory_recorder save_tf.py

