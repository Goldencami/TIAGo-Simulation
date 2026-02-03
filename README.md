# TIAGo-Simulation
Simulation of a Tiago Robot moving in front of a table and moving its arm using ROS2 Humble

## Dependencies
```bash
sudo apt-get update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control -y
sudo apt install ros-humble-xacro
sudo apt install python3-pip
sudo apt-get install git python3-vcstool python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
sudo apt install ros-humble-ros-ign-gazebo
```

## Add world
This project uses PAL Robotic's tiago_gazebo, their script launches a world stored inside `pal_gazebo_worlds`. To be able to launch our simulation, you must copy the custome world from `launch_sim/worlds/sim_world1.world` and paste it inside `pal_gazebo_worlds/worlds`.

## Source the workspace
```bash
cd ~/TIAGo-Simulation/ros2_ws
source install/setup.bash
```

## Build the workspace
```bash
cd ~/TIAGo-Simulation/ros2_ws
colcon build
source install/setup.bash
```

# Usage
## Start Tiago, simulated in Gazebo Harmonic
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=sim_world1
```

### Launch exercise
Pick one of the two exercises to launch.

```bash
ros2 launch launch_sim start_exercise1.launch.py
```

### Launch exercise
```bash
ros2 launch launch_sim start_exercise2.launch.py
```

## Resources
- [TIAGo ROS 2 Simulation](https://github.com/pal-robotics/tiago_simulation/tree/humble-devel)
- [Setting up a robot simulation](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html)
- [Motion planning](https://www.youtube.com/watch?v=G0T6IzXM4xQ&list=PLaxxZSuubhFfIoeeT1M74CN2vFaWP4vFu&index=27)