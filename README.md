# ABB IRB 2400 Industrial Robot Manipulator Simulation Using Ros2,Gazebo,Rviz,moveit

This project involves a simulation of the ABB IRB 2400 Industrial Robot Manipulator using ROS 2, Gazebo, and MoveIt for advanced motion planning. The project consists of four primary packages that facilitate the development, control, and simulation of the robot.

## Project Structure

The project includes the following packages:

1. **irb2400_description**: Contains the robot's URDF description and launch files and others for visualizing the robot in RViz and Gazebo.
2. **irb2400_controller**:  Contains the configuration and launch files for the controller manager.
3. **irb2400_moveit**:	    Integrates MoveIt for motion planning and move robot.


### Prerequisites

- ROS 2 (Humble)
- Gazebo (11.10.2)
- MoveIt (2.5.5 - Alpha)

### Installation

1. Clone the repository to your local machine:
    ```bash

    git clone https://github.com/MaEl1405/abb_irb2400_ros2.git
    cd abb_irb2400_ros2
    ```
2. Build the workspace:
    ```bash
    colcon build
    source install/setup.bash
    ```

### Launching the Simulation

To run the simulation, use the following commands:

1. Launch the robot in Gazebo:
    ```bash
    ros2 launch irb2400_description gazebo.launch.py
    ```

2. Launch the robot controller:
    ```bash
    ros2 launch irb2400_controller controller.launch.py
    ```

4. Launch MoveIt:
    ```bash
    ros2 launch irb2400_moveit moveit.launch.py
    ```

5. Run the motion planning node:
    ```bash
    ros2 run irb2400_moveit move_arm
    ```

### Videos
<p align="center">
  <video src="Docs/images/irb2400.mp4" width="500px"></video>
</p>

---

## Contributing
If you would like to contribute to this project, please open an issue or submit a pull request. I welcome improvements, bug fixes, and new features.
