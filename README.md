# TIAGo-Simulation
Simulation of a Tiago Robot moving in front of a table and moving its arm using ROS2 Humble

## Dependencies
```bash
sudo apt-get update
sudo apt-get install git python3-vcstool python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

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

## Documentation
- [TIAGo ROS 2 Simulation](https://github.com/pal-robotics/tiago_simulation/tree/humble-devel)
- [Setting up a robot simulation](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html)