# My ROS2 Package

## Description
This package implements the Bug1 algorithm for navigating a robot in a simulated environment.

## Installation

### Prerequisites
- ROS2 Foxy Fitzroy or later.
- `stage_ros2` package. This package is required to run the simulation.
- https://github.com/tuw-robotics/stage_ros2

### Installation Steps

1. Clone this repository:
    ```sh
    git clone https://github.com/Gardiy/Laser-Stage.git
    cd nombre_del_repositorio
    ```

2. Build the package:
    ```sh
    colcon build
    ```

3. Source the setup file:
    ```sh
    source install/setup.bash
    ```

4. Make sure to have the `stage_ros2` package installed. It is available in the following repository: [Stage_ros2 GitHub Repository](https://github.com/ros-simulation/stage_ros2).

5. Run the stage simulator with the following command:
    ```sh
    ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true
    ```

## Usage

After setting up the environment, you can run the main node of this package using:
```sh
ros2 launch laser_stage robot_launch.py

