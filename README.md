# ROS2 VESC Driver for Differential Drive Setup

A basic python driver for VESC-based differential drive system that runs on ROS2-foxy

## Introduction

I made this tiny script for my robotic platform. The existing VESC drivers at the moment either don't support ROS2 or dont implement CAN chaining (pass command through to a CAN device). In my HW config I have 2x VESC devices connected via CAN. One of those acts as a master and connects to the host via serial (USB-serial).

## Install (native mode)

Install dependencies of this package into your python env;

Clone the repo into your workspace, then build the package in your workspace root and so on :-)

```bash
cd ros2_ws
colcon build --packages-select ros2_vesc_drv
. install/bash.setup
```

To run the node do something like:

```bash
ros2 run ros2_vesc_drv vesc_diff_drv
```

## Usage

The node expects Float64 (signed) demands to come through the following topics:

- `/vesv_L/duty` for the left drive
- `/vesc_R/duty` for the right drive

to test the thing you may start up the node and in a separate terminal feed it with some Float64 values like so:

```bash
ros2 topic pub --rate 10 /vesc_R/duty std_msgs/msg/Float32 data:\ 0.05\
```
