# Kondo ICS motor (KRS40xxHV, etc.) driver

## Kickstart

1. Launch kondo_ics_driver.launch

```
$ roslaunch kondo_ics_driver kondo_ics_driver.launch
```

2. Launch kondo_ics_driver.launch

## How it works

1. The driver scan the ICS bus. All motors need to be asigned unique ID.
2. The driver read the parameters from /kondo_ics_driver. 
3. The driver map the motor ID(n) to joint by setting the joint name into motor_n/joint.
