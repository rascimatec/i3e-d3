# I3E-D3
A project of a openSource robotic plataform with a manipulator, developed by the IEEE RAS CIMATEC Student Branch

## Install the packages
- First, create a ROS workspace:
```bash
mkdir -p ~/i3e-d3_ws/src
```
- Clone this repository inside your /src folder:
```bash
cd i3e-d3_ws/src
git clone https://github.com/rascimatec/i3e-d3
```
- Install all dependencies and build your workspace:
```bash
cd ~/i3e-d3_ws
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
catkin_make
```

## Testing
- When open a new terminal:
```bash
cd ~/i3e-d3_ws
source devel/setup.bash
```
- In the first terminal:
```bash
roslaunch i3e-d3_gazebo i3e-d3_jackal_world.launch
```
- In the second terminal:
```bash
roslaunch i3e-d3_launchs exploration.launch
```

#### * This repository has not yet been finished. Not all features will be working correctly
