
# Webots ROS2 Gen3

[![ROS2 Humble](https://github.com/skpawar1305/webots_ros2_gen3/actions/workflows/test_ros2_humble.yml/badge.svg?branch=2f85)](https://github.com/skpawar1305/webots_ros2_gen3/actions/workflows/test_ros2_humble.yml)

This is a ROS 2 package to simulate the Kinova Gen3 robot in [Webots](https://cyberbotics.com/). The package supports MoveIt2 integration, gripper, and example launch files for simulation and motion planning.

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble ([installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
- Webots 2025a ([download](https://github.com/cyberbotics/webots/releases/tag/R2025a))

## Install

1. Install ROS2 Development tools and initialise and update rosdep:
    ```
    sudo apt install -y ros-dev-tools
    ```
    ```
    source /opt/ros/humble/setup.bash
    sudo rosdep init
    rosdep update
    ```

2. Create a new ROS2 workspace:
    ```
    export COLCON_WS=~/ros2_ws
    mkdir -p $COLCON_WS/src
    ```

3. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```
    cd $COLCON_WS
    git clone https://github.com/skpawar1305/webots_ros2_gen3 src/webots_ros2_gen3
    vcs import --recursive src --skip-existing --input src/webots_ros2_gen3/webots_ros2_gen3.repos
    rosdep install --ignore-src --from-paths src -y -r
    chmod +x src/webots_ros2/webots_ros2_driver/webots_ros2_driver/ros2_supervisor.py
    ```

4. Build packages and source the workspace
    ```
    colcon build --symlink-install
    source install/setup.bash
    ```

## Start

To launch the robot in Webots:
```bash
ros2 launch webots_gen3 robot_launch.py
```

To launch MoveIt2 integration:
```bash
ros2 launch webots_gen3 moveit_launch.py
```

## Additional Links

- [Webots](https://cyberbotics.com/)
- [Kinova Gen3](https://www.kinovarobotics.com/en/products/gen3-robot)
- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)

```
