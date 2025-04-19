#!/bin/zsh
source /opt/ros/jazzy/setup.zsh
source ./install/setup.zsh
ros2 run qbot image_decompressor &
RUST_LOG=info zenoh-bridge-ros2dds -c config.json5 &
wait
