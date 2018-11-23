# Rosserial Mbed Libraries #
=====================================

This repository consists of the contents of the `ros_lib` directory created by
running the script `rosrun rosserial_mbed make_libraries.py`.

It exists just to be uploaded to the PlatformIO Library Registry.

ROS package: ros-kinetic-rosserial-mbed (0.7.6-0xenial-20170416-194902-0800)

## Changes for PlatformIO ##
=====================================

To make it compile under PlatformIO, 2 changes has been made:

1. In file **MbedHardware.h**, changed 
```
#include "BufferedSerial.h"
```
to
```
#include "BufferedSerial/BufferedSerial.h"
```

2. In file **BufferedSerial/BufferedSerial.h**, changed 
```
#include "Buffer.h"
```
to
```
#include "Buffer/Buffer.h"
```
