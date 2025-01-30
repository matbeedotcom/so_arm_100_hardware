# SO-ARM-100 Hardware Interface

## Overview
The `so_arm_100_hardware` package provides a ROS 2 Control hardware interface plugin for the SO100-arm low-cost 5DoF robotic manipulator. This plugin implements a ROS 2 Control SystemInterface that enables communication with the robotic arm through ROS topics.

## Features

- ROS 2 Control hardware interface implementation
- Position control interface for all joints
- Communication through ROS topics for commands and feedback
- Thread-safe feedback handling
- Lifecycle-managed node implementation

## Package Details

- **Name**: so_arm_100_hardware
- **Version**: 0.0.0
- **Description**: ROS2 Control Hardware Interface for SO100-arm low-cost 5DoF robotic manipulator
- **Maintainer**: Bruk Gebregziabher (<bruk@signalbotics.com>)
- **License**: Apache-2.0

## Dependencies

- `rclcpp`
- `hardware_interface`
- `pluginlib`
- `rclcpp_lifecycle`
- `sensor_msgs`

## Communication Interface

The hardware interface communicates using the following ROS topics:

- **Command Topic**: `command` (sensor_msgs/msg/JointState)
  - Publishes joint position commands to the robot
  
- **Feedback Topic**: `feedback` (sensor_msgs/msg/JointState)
  - Subscribes to joint state feedback from the robot

## Hardware Interface Details

The `SOARM100Interface` class implements:

- State and command interfaces for position control
- Lifecycle management (init, activate, deactivate)
- Read and write methods for communication
- Thread-safe feedback handling using mutex
- Asynchronous ROS communication using a dedicated executor thread

## Installation

1. **Clone the package into your ROS 2 workspace**:

   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>/so_arm_100_hardware.git
   ```

2. **Install dependencies**:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:

   ```bash
   colcon build --packages-select so_arm_100_hardware
   ```

4. **Source the workspace**:

   ```bash
   source install/setup.bash
   ```

## Usage

### Configuration

Create a ROS 2 Control configuration file (YAML) that includes the hardware interface:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

so_arm_100:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
```

### Launch File Example

Create a launch file to load and start the hardware interface:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': '<robot_description_xml>'},
                'config/controllers.yaml'
            ],
            output='screen',
        ),
    ])
```

### Running the Hardware Interface

1. Start the hardware interface:

   ```bash
   ros2 launch <your_package> so_arm_100.launch.py
   ```

2. Load and configure controllers:

   ```bash
   ros2 control load_controller joint_state_broadcaster --state active
   ros2 control load_controller joint_trajectory_controller --state active
   ```

## Development and Testing

The hardware interface includes commented-out serial communication code that can be implemented for direct hardware control. Currently, it operates using ROS topics for command and feedback.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This package is licensed under the Apache License 2.0. See the LICENSE file for details.
