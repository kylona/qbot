#!/bin/zsh

# --- Trap for cleanup ---
# Function to kill all child processes
cleanup_processes() {
    echo "Caught Ctrl+C. Terminating child processes..."
    # Kill ros2 run processes
    if [[ -n "$IMAGE_DECOMPRESSOR_PID" ]]; then
        kill "$IMAGE_DECOMPRESSOR_PID" 2>/dev/null
    fi
    if [[ -n "$CAMERA_IMU_SYNC_PID" ]]; then
        kill "$CAMERA_IMU_SYNC_PID" 2>/dev/null
    fi
    # Kill zenoh-bridge-ros2dds
    if [[ -n "$ZENOH_BRIDGE_PID" ]]; then
        kill "$ZENOH_BRIDGE_PID" 2>/dev/null
    fi
    # Wait for a moment for processes to terminate
    sleep 1

    exit 1 # Exit the script with an error code to indicate interruption
}

# Trap SIGINT (Ctrl+C) and SIGTERM (kill command)
trap cleanup_processes SIGINT SIGTERM

# --- Source ROS2 and local setup ---
source /opt/ros/jazzy/setup.zsh
source ./install/setup.zsh

echo "Starting ROS2 nodes and Zenoh bridge..."

# --- Start ROS2 nodes in the background and store their PIDs ---
ros2 run qbot image_decompressor &
IMAGE_DECOMPRESSOR_PID=$!
echo "image_decompressor started with PID: $IMAGE_DECOMPRESSOR_PID"

#ros2 run qbot camera_imu_sync &
#CAMERA_IMU_SYNC_PID=$!
#echo "camera_imu_sync started with PID: $CAMERA_IMU_SYNC_PID"

# --- Start Zenoh bridge in the background and store its PID ---
RUST_LOG=info zenoh-bridge-ros2dds -c config.json5 &
ZENOH_BRIDGE_PID=$!
echo "zenoh-bridge-ros2dds started with PID: $ZENOH_BRIDGE_PID"

echo "All processes started. Press Ctrl+C to terminate them."

# --- Wait for all background jobs to complete ---
wait

echo "All child processes have finished or were terminated."
