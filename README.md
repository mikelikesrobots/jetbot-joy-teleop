# Jetbot Joy Teleop

This repository contains code for running the Waveshare Jetbot AI with either a gamepad or with keyboard.

It builds a basic ROS2 node that converts movement commands into I2C commands for motors.

There are two ROS2 workspaces:
1. `jetbot_complete_ws`: contains a package with a launch file, plus VSCode configuration for building and running a custom ROS2-based Docker container for running the software properly.
2. `jetbot_incomplete_ws`: contains the same package without the launch file and configuration. The user may learn to build this configuration themself.

## Building the code

This code is intended to be run on board the Jetbot directly, but can be run anywhere. The user should use VSCode to easily run the code; otherwise, the required Dockerfile is contained [here](./jetbot_complete_ws/.devcontainer/Dockerfile).

In VSCode, install the Remote - SSH and Remote - Docker extensions.
Open the `jetbot_complete_ws` in VSCode on the Jetbot and wait for a container to be started.

Run the following command:

```bash
colcon build --symlink-install
```

Once the build is complete, use either of the two options:

```bash
# For keyboard movement, use two terminals:
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard  # Terminal 1
ros2 run py_jetbot_control jetbot  # Terminal 2

# For gamepad movement, connect a PS4 controller
source install/setup.bash
ros2 launch py_jetbot_control joy.yml
```

The robot should now respond to commands! The gamepad requires that the "A" button be held down for any commands to be transmitted.
