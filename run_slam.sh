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
tmux split-window -h -t $SESSION_NAME:0.0 
tmux split-window -v -t $SESSION_NAME:0.0 
tmux split-window -v -t $SESSION_NAME:0.2 

# 3. Send commands to each pane with built-in sleep delays

# Pane 0 (Starts immediately)
tmux send-keys -t $SESSION_NAME:0.0 "echo 'Starting OpenVINS immediately...' && $SETUP_CMD && ros2 launch challenge_tools_ros run_openvins.launch.py" C-m

# Pane 1 (Waits 5 seconds)
tmux send-keys -t $SESSION_NAME:0.1 "echo 'Waiting 5 seconds...' && sleep 5 && echo 'Starting Map Server...' && $SETUP_CMD && ros2 launch challenge_tools_ros map_server.launch.py mask:=masks_with_windows run_name:=floor_1_2025-05-05_run_1" C-m

# Pane 2 (Waits 10 seconds)
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Waiting 10 seconds...' && sleep 10 && echo 'Starting Groundtruth Server...' && $SETUP_CMD && ros2 launch challenge_tools_ros groundtruth_server.launch.py run_name:=floor_1_2025-05-05_run_1" C-m

# pane 4 (Waits 20 seconds)
tmux send-keys -t $SESSION_NAME:0.3 "echo 'Waiting 20 seconds...' && sleep 20 && echo 'Playing ROSbag...' && $SETUP_CMD && ros2 bag play src/hilti-trimble-slam-challenge-2026/challenge_tools_ros/download_data/challenge_data/data/floor_1/2025-05-05/run_1/rosbag -p -r 0.5" C-m

# 4. Attach to the session IMMEDIATELY
tmux attach-session -t $SESSION_NAME