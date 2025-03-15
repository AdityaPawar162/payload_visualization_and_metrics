# Payload visualization and metrics
This ros2 package demonstrates the use of the Pinocchio rigid body dynamics library to perform various operations on a robot model. It subscribes to the robot description topic and uses Pinocchio to create a model, perform forward kinematics, calculate Jacobians, and check for collisions between links.

## Overview
This package provides a ROS2 node that:
1. Subscribes to the `robot_description` topic to get the URDF model of the robot
2. Creates a Pinocchio model from the URDF
3. Perform forward kinematics to get the end effector position and orientation
4. Computes the Jacobian matrix at a specific frame
5. Checks for collisions between specified links

The implementation focuses on the TIAGo robot model but can be adapted for other robots.

## Dependencies
* ROS 2(Humble)
* Pinocchio
* Eigen3

## Installation

### Install ROS2
Follow the official ROS 2 installation for your platform:
[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Install Pinocchio

```sh
sudo apt install ros-$ROS_DISTRO-pinocchio
```

## Building the package
1. Create a ROS 2 workspace(if you don't have one alaready):
```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:

```sh
git clone https://github.com/AdityaPawar162/payload_visualization_and_metrics.git
```

3. Build the package:

```sh
cd ~/ros2_ws
colcon build --packages-select payload_visualization_and_metrics
```

4. Source the workspace:

```sh
source ~/ros2_ws/install/setup.bash
```

## Usage

1. Start a ROS 2 node that publishes the robot's URDF on the `robot_description` topic. For example, with TIAGo:

```sh
ros2 launch tiago_description show.launch.py
```

2. Run the payload_visualization_and_metrics node:

```sh
ros2 run payload_visualization_and_metrics pinocchio_task
```

## Implementation Details

### Node Structure
The main node is implemented in the `PinocchioTiagoNode` class, which inherits from `rclcpp::Node`. Here's a breakdown of its functionality

### Initialization
* Subscribes to the `robot_description` topic
* Sets up the necessary Pinocchio data structures

### Robot Description Processing
When a URDF is recieved via the `robot_description` topic:
1. Saves the URDF to a temporary file
2. Creates a Pinocchio model from the URDF
3. Initializes the collision model
4. Performs the following operations:
   - Forward kinematics
   - Jacobian computation
   - Collision checking

### Key Methods

* `createPinocchioModel()`: Builds the Pinocchio model from the URDF
* `performForwardKinematics()`: Computes the forward kinematics and retrieves the end effector position and orientation
* `getJacobian()`: Computes the Jacobian matrix at the specified frame
* `checkCollisions()`: Checks for collisions between specified links

### Approach to Solution
The implementation follows these principles:

1. **Modular Design**: The code is structured into separate methods for each operation, making it easy to understand and extend.
2. **Error Handling**: Each method includes checks to ensure the model is properly created before performing operations.
3. **Comprehensive Logging**: The node provides detailed logging to help users understand what's happening during execution.
4. **Flexibility**: Although designed for TIAGo, the code can be easily adapted for other robots by changing the frame names and link specifications.
5. **Integration with ROS 2**: The node seamlessly integrates with the ROS 2 ecosystem, subscribing to the standard robot_description topic.
