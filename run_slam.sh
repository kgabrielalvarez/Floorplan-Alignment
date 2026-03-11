#!/bin/bash

SESSION_NAME="hilti_challenge"
SETUP_CMD="cd ~/hilti-trimble-challenge_ws && source install/setup.bash"

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux could not be found. Please install it using: sudo apt install tmux"
    exit 1
fi

echo "Starting tmux session: $SESSION_NAME"

# 1. Start a new detached tmux session
tmux new-session -d -s $SESSION_NAME

# 2. Split the window into a 2x2 grid
# Split horizontally (left/right)
tmux split-window -h -t $SESSION_NAME:0.0 
# Split the left side vertically (top/bottom)
tmux split-window -v -t $SESSION_NAME:0.0 
# Split the right side vertically (top/bottom)
tmux split-window -v -t $SESSION_NAME:0.2 

# 3. Send commands to each pane
# (C-m simulates pressing the 'Enter' key to execute the command)

# Pane 0 (Top Left): OpenVINS
tmux send-keys -t $SESSION_NAME:0.0 "$SETUP_CMD && ros2 launch challenge_tools_ros run_openvins.launch.py" C-m

# Pane 1 (Bottom Left): Map Server
tmux send-keys -t $SESSION_NAME:0.1 "$SETUP_CMD && ros2 launch challenge_tools_ros map_server.launch.py mask:=masks_with_windows run_name:=floor_1_2025-05-05_run_1" C-m

# Pane 2 (Top Right): Groundtruth Server
tmux send-keys -t $SESSION_NAME:0.2 "$SETUP_CMD && ros2 launch challenge_tools_ros groundtruth_server.launch.py run_name:=floor_1_2025-05-05_run_1" C-m

# Pane 3 (Bottom Right): ROSbag Play
tmux send-keys -t $SESSION_NAME:0.3 "$SETUP_CMD && ros2 bag play src/hilti-trimble-slam-challenge-2026/challenge_tools_ros/download_data/challenge_data/data/floor_1/2025-05-05/run_1/rosbag -p -r 0.7" C-m

# 4. Attach to the session so you can view the grid
tmux attach-session -t $SESSION_NAME