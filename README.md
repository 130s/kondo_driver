# Kondo ICS motor (KRS40xxHV, etc.) driver

This ROS package contains hardware interface to Kondo Kagaku's ICS
motor system. ICS(Interactive Communication System) is a kind of
serial bus to control RC servo motors. Basically it can control
measure its position. It has additional features such as setting gain,
getting current.

## Kickstart

0. Connect servo motor (ID0 and ID1) by USB ICS adapter.
1. Launch kondo_driver.launch
```
$ roslaunch kondo_driver kondo_driver.launch
```
2. Publish /joint_0_controller/command (std_msgs/Float64) to control joint angle. Unit is in radian.

## How it works

1. The driver scan the ICS bus. All motors need to be asigned unique ID.
2. The driver read the parameters from /kondo_driver.
3. Each setting of motor should be placed /kondo_driver/<motor_name>/
4. The driver set parameters to motor.
5. This driver provides standard ros_control interface.
6. Use controller_manager/spawner to load joint_state_controller and joint_position_controller.

