# Control_ur5_ros_noetic

This repository provides a comprehensive collection of tasks demonstrating control of the UR5 robotic arm using ROS Noetic and Gazebo. It showcases essential robotic manipulation techniques and automation within a simulated environment, highlighting motion control, path planning, and integration with robotic frameworks.

## Key Features

- **UR5 Robotic Arm Control**: Implements inverse kinematics for precise joint positioning and manipulation tasks.
- **ROS Noetic and Gazebo Integration**: Utilizes ROS topics, services, and action servers for real-time control in a simulated dynamic environment.
- **Trajectory and Motion Planning**: Implements custom trajectory generation using polynomial interpolation and path-following algorithms.

## Directory Overview

- `ur5_task.py`: Script demonstrating basic joint movement commands and kinematic positioning.
- `ur5_task10.py`: Advanced script showcasing trajectory following and feedback-based corrections.
- Additional files explore inverse kinematics calculations, path interpolation, and control tuning.

## Installation and Dependencies

- **ROS Noetic**: Installed on Ubuntu 20.04.
- **Gazebo Simulation**: Used to simulate and visualize UR5 robot behavior.
- **Python 3.x**: Required for running the control scripts.

Install ROS dependencies using:
```bash
sudo apt-get install ros-noetic-ur5-moveit-config ros-noetic-gazebo-ros
```

To install ROS Noetic: [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Usage Instructions

1. Set up the simulation environment:
   ```bash
   roslaunch ur5_control ur5.launch
   ```
2. Run task-specific Python scripts, such as:
   ```bash
   python3 ur5_task.py
   ```
3. Customize trajectory parameters by modifying the script variables.

### Example Script Workflow

- `ur5_task.py`: 
  - Initializes ROS nodes and UR5 control interfaces.
  - Computes joint-space movements for target coordinates.
  - Executes motion commands via ROS publishers.
- `ur5_task10.py`:
  - Generates smooth trajectories using cubic spline interpolation.
  - Implements feedback correction using sensor inputs for enhanced precision.

## Technical Details

- **Kinematic Control**: Uses forward and inverse kinematics equations for target positioning.
- **PID Controller (Optional)**: Planned integration for enhanced position control.
- **ROS Communication**: Leverages `ros_control` and `robot_state_publisher` for efficient message handling.

## Planned Enhancements

- **MoveIt! Integration**: Enabling collision detection and dynamic planning.
- **Real-Time Obstacle Avoidance**: Dynamic path generation with proximity sensors.
- **PID Control for Joint Trajectories**: Optimizing stability and responsiveness.

## Contribution Guidelines

We welcome contributions that add functionality, improve performance, or introduce new features. Please fork the repository, make your changes, and submit a pull request with a clear description.

## Licensing

This project is distributed under the MIT License.

## Support and Contact

For inquiries, suggestions, or feedback, please contact the repository maintainer via [GitHub](https://github.com/HemantP02/Control_ur5_ros_noetic) or open an issue.
