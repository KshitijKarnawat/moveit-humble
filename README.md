# Project 4

## MoveIt! Setup Assistant and Pick and Place

## Requirements

- ROS Humble
- MoveIt!
- Rviz

## Installation

```bash
# Install ROS Humble and MoveIt! if you haven't already

# Make sure you have MoveIt! tutorials installed

# Clone the repository / or donwload the zip file
git clone git@github.com:KshitijKarnawat/moveit-humble.git

# Build the package
cd <path_to_workspace>

colcon build

# Source the workspace
source install/setup.bash
```

## Usage

```bash
# Run the launch file
ros2 launch moveit2_tutorial demo.launch.py

# In a new terminal, run the following command to start the pick and place demo:

ros2 run package_119188651 pick_place_tutorial
```

## Video Links

- [MoveIt Pick and Place](https://youtu.be/f4jPHQOG504)
- [MoveIt Setup Assistant](https://youtu.be/T7-E3bh5LMU)
