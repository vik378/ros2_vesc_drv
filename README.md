# ROS2 VESC Driver for Differential Drive Setup

A basic python driver for VESC-based differential drive system that runs on ROS2-foxy

## Introduction

I made this tiny script for my robotic platform. The existing VESC drivers at the moment either don't support ROS2 or don't implement CAN chaining (pass command through to a CAN device). In my HW config I have 2x VESC devices connected via CAN. One of those acts as a master and connects to the host via serial (USB-serial).

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

The node expects Float32 (signed) demands to come through the following topics:

- `/vesv_L/duty` for the left drive
- `/vesc_R/duty` for the right drive

to test the thing you may start up the node and in a separate terminal feed it with some Float32 values like so:

```bash
ros2 topic pub --rate 10 /vesc_R/duty std_msgs/msg/Float32 data:\ 0.05\
```

There is also a launch file that allows for teleoperation using joystick / cmd_vel topic. It implements the graph below:

![vesc_with_joy ROS2 nodes graph](docs/vesc_with_joy.png)

The launch fila has some parameters tuned for my platform (like speed limits) so you may need to tune those things a bit.

The thing may be started with the following command:

```bash
ros2 launch ros2_vesc_drv vesc_with_joy.launch.py
```

## Running in a container

To build the container with this package you may run `docker build -t ros2_vesc_drv .` in the root of this package.

To run the package with /cmd_vel interpreter (diamond steering mixer) use the below:

```bash
docker run -it --net=host --pid=host --device=/dev/ttyACM0 ros2_vesc_drv:latest ros2 launch ros2_vesc_drv vesc_cmd_vel.launch.py
```

You may feed it with some /cmd_vel with Message Publisher (rqt) or joystick from your host PC, there is a launch file for that too :-)

```bash
# below thing will turn joystick inputs into a /cmd_vel topic / messages
ros2 launch ros2_vesc_drv joy_teleop.launch.py
```

## Parameters of "vesc_diff_srv" Node

The node supports dynamic parameter re-definition. The following parameters are supported:

| Parameter   | Description                                             | Type   | Default Value  |
| ----------- | ------------------------------------------------------- | ------ | -------------- |
| serial_port | path to the serial port that VESC master is attached to | String | "/dev/ttyACM0" |
