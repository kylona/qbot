# MPU9250 FIFO

## About
  A driver for the MPU9250 9-axis Inertial Measurement Unit (IMU) with support to utilize the built in FIFO to achieve higher frequency and more consistent sampling.

## Why
  I built this as a pet project as a way to learn Rust. I intend to use it to publish accurate orientation data as a ROS2 node. I also want to experiment with publishing velocity and position estimates by integrating acceleration data. This will be prone to drift and to achieve any amount of accuracy it will require high frequency samples.  