# Kondo ICS motor (KRS40xxHV, etc.) driver

This ROS package contains hardware interface to Kondo Kagaku's ICS
motor system. ICS(Interactive Communication System) is a kind of
serial bus to control RC servo motors. Basically it can control
measure its position. It has additional features such as setting gain,
getting current.

This package contains ics read/write code (ics.c, ics.h) from [libkondo4 project](https://bitbucket.org/vo/libkondo4/wiki/Home).

## Supported Hardware

### USB ICS Adapters
* [ICS USB Adapter HS](http://kondo-robot.com/product/02116)
* [Dual USB Adapter HS](http://www.kopropo.co.jp/sys/archives/4315)

### ICS servo motors
* [KRS-4031HV ICS](http://kondo-robot.com/product/krs-4031hv-ics)
* [KRS-4031HV ICS](http://kondo-robot.com/product/krs-4031hv-ics)

# Quick start

## Setting servo ID.
Connect each servo and the adapter one to one.
For setting servo ID, you can use ics_set_id.
```
$ rosrun ics_set_id <product_id> <id>
```

<product id> is 6 for 'ICS USB adapter HS', 8 for 'dual ICS adapter HS'.

After setting each servo motor, you can connect your motors as you want.

## Change config/driver_sample.yaml

Describe servo setting in yaml file. 'driver_sample.yaml' shows the example in which two motors (ID0,ID1) connected.
* Change product_id to your adapter.
* Change joint_0_driver and joint_1_driver field as your servo system.

Each field is;

* joint: Name of joint 
* id: Servo's ICS id. It is set by utility.
* min_anlge: Minimum angle of servo [deg.]
* max_anlge: Maximum angle of servo [deg.]
* speed: speed paramter. See ICS manual.
* stretch: stretch parameter. See ICS manual.

## Change config/controller_sample.yaml

The kondo_driver is fit for using ros_controllers. Check config/controller_sample.yaml for ros_control configuration.
This example uses position_controllers/JointPositionController to control joint position and joint_state_controller/JointStateController to publish /joint_states.

## Launch kondo_driver.launch to bring up driver.
The following sample bring up controllers. 
```
$ roslaunch kondo_driver kondo_driver.launch
```
## Command the servo
Check if you can command the sevo position.
Publish /joint_0_controller/command (std_msgs/Float64) to control joint angle. Unit is in radian.
```
$ rostopic pub -1 /joint_0_controller/command std_msgs/Float64 "data: 0.0"
```

