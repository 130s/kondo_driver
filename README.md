# Kondo ICS motor (KRS40xxHV, etc.) driver

This ROS package contains hardware interface to Kondo Kagaku's ICS
motor system. ICS(Interactive Communication System) is a kind of
serial bus to control RC servo motors. Basically it can control
measure its position. It has additional features such as setting gain,
getting current.

## Supported Hardware
### USB ICS Adapters
* [ICS USB Adapter HS](http://kondo-robot.com/product/02116)

* [Dual USB Adapter HS](http://www.kopropo.co.jp/sys/archives/4315)
## Kickstart

### Connect servo motor (ID0 and ID1) by USB ICS adapter.

### Change config/driver_sample.yaml
Change product_id to your adapter.

Change joint_0_driver and joint_1_driver field as your servo system.
Each field is;

* joint: Name of joint 
* id: Servo's ICS id. It is set by utility.
* min_anlge: Minimum angle of servo [deg.]
* max_anlge: Maximum angle of servo [deg.]
* speed: speed paramter. See ICS manual.
* stretch: stretch parameter. See ICS manual.
* punch: punch parameter. See ICS manual.
* dead band: dead band parameter. See ICS manual.

### Launch kondo_driver.launch
```
$ roslaunch kondo_driver kondo_driver.launch
```
### Command the servo

Publish /joint_0_controller/command (std_msgs/Float64) to control joint angle. Unit is in radian.
```
$ rostopic pub -1 /joint_0_controller/command std_msgs/Float64 "data: 0.0"
```

## How it works

1. The driver scan the ICS bus. All motors need to be asigned unique ID.
2. The driver read the parameters from /kondo_driver.
3. Each setting of motor should be placed /kondo_driver/<motor_name>/
4. The driver set parameters to motor.
5. This driver provides standard ros_control interface.
6. Use controller_manager/spawner to load joint_state_controller and joint_position_controller.

