# TIAGo-Simulation
Simulation of a Tiago Robot moving in front of a table and moving its arm using ROS2 Humble

## Clone submodules
```bash
git submodule update --init --recursive
```

## Dependencies
```bash
sudo apt-get update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control -y
sudo apt install python3-pip
sudo apt-get install git python3-vcstool python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

## Add world
This project uses PAL Robotic's tiago_gazebo, their script launches a world stored inside `pal_gazebo_worlds`. 

To be able to launch our simulation, you must copy the custome worlds from `launch_sim/worlds` and paste it inside `pal_gazebo_worlds/worlds`.

## Source the workspace
```bash
cd ~/TIAGo-Simulation/ros2_ws
source install/setup.bash
```

## Build the workspace
```bash
cd ~/TIAGo-Simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Simulation
### Start TIAGo Simulation
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=small_sim_world_obj
```

### Launch exercise
```bash
ros2 launch launch_sim start_exercise.launch.py
```

## Logic
The following diagram shows how obstacle avoiding and navigation towards the goal were implemented
![Image](https://github.com/user-attachments/assets/1c4adccd-84c4-4ec4-bf8e-8dc32425948b)

## State Machine
![Image](https://github.com/user-attachments/assets/ad0dd9aa-bb3f-4ac9-9918-0f1d1039d291)

## Resources
- [TIAGo ROS 2 Simulation](https://github.com/pal-robotics/tiago_simulation/tree/humble-devel)
- [Setting up a robot simulation](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html)
- [Motion planning](https://www.youtube.com/watch?v=G0T6IzXM4xQ&list=PLaxxZSuubhFfIoeeT1M74CN2vFaWP4vFu&index=27)
- [MoveIt](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)
