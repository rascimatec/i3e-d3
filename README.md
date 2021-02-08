# I3E-D3
A project of a openSource robotic plataform with a manipulator, developed by the IEEE RAS CIMATEC Student Chapter

## Install the packages
- First, create a ROS workspace:
```bash
$ mkdir -p ~/i3e-d3_ws/src
```
- Clone this repository inside your /src folder:
```bash
$ cd i3e-d3_ws/src
$ git clone https://github.com/rascimatec/i3e-d3
```
- Install all dependencies and build your workspace:
```bash
$ cd ~/i3e-d3_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
$ catkin_make
>>>>>>> Stashed changes
```

## Package contents
- **3irobotics-delta2a-sdk:** LIDAR SDK package, cloned from [https://github.com/biomchen/3irobotics-delta2a-sdk](https://github.com/biomchen/3irobotics-delta2a-sdk)
- **i3e-d3_control:** Control and teleoperation of the robot
- **i3e-d3_description:** I3E-D3 description, URDF and mesh files
- **i3e-d3_gazebo:** Gazebo worlds for simulation
- **i3e-d3_launchs:** A useful package for launch files
- **i3e-d3_navigation:** SLAM and move_base, used for navigation
- **i3e-d3_viz:** Package for visualization

## Simulation
When open a new terminal:
```bash
$ cd ~/i3e-d3_ws
$ source devel/setup.bash
```
The i3e-d3_launchs package provides some useful launch files for simulation. 
By default, it will start the Playpen world, but you can change if you want.

Also, if you want to make your own launch file, you can be based on the 'template.launch'.

### Autonomous Exploration
The robot will explore the world on its own until the mapping is complete.
```bash
$ roslaunch i3e-d3_launchs exploration.launch
```

### Autonomous Navigation
The robot will navigate through a known map.
```bash
$ roslaunch i3e-d3_launchs navigation.launch
```

### Mapping with Navigaton
The robot will map the environment using navigation.
```bash
$ roslaunch i3e-d3_launchs mapping_nav.launch
```

### Mapping with Teleoperation
The robot will map the environment using teleoperation.
```bash
$ roslaunch i3e-d3_launchs mapping_teleop.launch
```

### Teleoperation
You just teleoperate the robot, using a generical joystick.
```bash
$ roslaunch i3e-d3_launchs teleoperation.launch
```

#### * This repository has not yet been finished. Not all features will be working correctly
