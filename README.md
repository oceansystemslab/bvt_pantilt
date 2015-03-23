Remote Ocean Systems Pan&Tilt Driver
====================================

The pan and tilt driver need to be started to use the unit. Once the driver is started the unit will change its position to the default initial position: front-facing aligned with vehicle body and with zero tilting.

## Basic info
The driver is able to operate the unit within given safety limits which are enforced in software and partially in hardware when the stop bolt is placed in the stop collar.

The unit is therefore limited to:
  - Pan Angle (azimuth): -135 to +135 degrees
  - Tilt Angle (elevation): 0 to 30 degrees

The Pan Angle is aligned with the robot frame with the zero pointing forward and increasing anti-clockwise. The Tilt Angle is zero when the unit is aligned with the robot frame and increasing moving away from the robot (ie. looking down).

Future integration with JointState messages is planned later on.

## Driver
The driver is usually started by top-level launch files. If the unit needs to be tested standalone a launch file is provided with this package:

```
roslaunch bvt_pantilt bvt_pantilt.launch
```

## Simulator

Alternatively a simulator can be started. This is provided with this package and provides the same interface of the main driver. Its configuration can be adjusted by editing the config file `launch/pantilt_sim.yaml` or by using the `rosparam` utility.

```
roslaunch bvt_pantilt bvt_pantilt_sim.launch
```

## Using the driver 
The unit can be controlled publishing a custom ros message directly to the driver:

```
rostopic pub -1 /pantilt/orientation_request bvt_pantilt/PanTiltOrientation "{azimuth: 0, elevation: 0}"
```

Listening to pan and tilt position can be done subscribing to the `orientation` topic:

```
rostopic echo -c /pantilt/orientation
```

This topic is published at the maximum speed allowed by the RS485 protocol used by the unit itself. It is quite slow and the rate should not exceed 2-4Hz.
