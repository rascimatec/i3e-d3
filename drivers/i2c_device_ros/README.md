# I2C Device ROS

## Overview
C++ library to read/write from/to I2C devices. This package abstracts bit and byte I2C R/W functions into a convenient class. This library was adapted from [i2cdevlib](https://github.com/jrowberg/i2cdevlib) to be used in the Raspberry Pi boards used in ROS projects such as [doogie_mouse robot](https://github.com/Brazilian-Institute-of-Robotics/doogie) that was the motivation to made this porting.

### License

The source code is released under a [MIT license](LICENSE).

**Author:** Mateus Menezes
**Maintainer:** Mateus Menezes, mateusmenezes95@gmail.com

The i2c_device_ros package has been tested under Raspberry Pi Zero W and Raspbian Jessie.

## Installation

### Building from soruce

First, put the package into your workspace
```sh
$ cd YOUR_WORKSPACE/src
$ git clone https://github.com/mateusmenezes95/i2c_device_ros.git
```

#### Building

After instalation of the package into your catkin workspace, you can install the library using two options. The first one is using `catkin_make` command 
```sh
$ cd YOUR_WORKSPACE
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="i2c_device_ros"
$ source devel/setup.bash
```

The second option is using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
```sh
$ cd YOUR_WORKSPACE
$ catkin build i2c_device_ros
$ source devel/setup.bash
```

## Usage

To use this library you need to set your package to recognize this library. First you need configure the `package.xml`file of the package that will use the `i2c_device_ros` library adding the line below:
```xml
<depend>i2c_device_ros</depend>
```

Then in the `CMakeLists.txt` add the macros below

```cmake
set(CATKIN_DEPS 
  <depend_package_1>
  <depend_package_2>
  ...
  i2c_device_ros
)

...

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPS}
)
```

With this configuration the header of this library will be available in the `${catkin_INCLUDE_DIRS}` variable and the dynamic library (`libi2c_device_ross.so`) will be available in `${catkin_LIBRARIES}` and then you can use in the macros that makes the targets of interest.

For more details how to use this library, see the [mpu6050_driver](https://github.com/mateusmenezes95/mpu6050_driver).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/mateusmenezes95/i2c_device_ros/issues).
