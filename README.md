# Reference Generators and Repository Overview

This is the accompanying code of the paper

> D. Risi, V. Petrone, A. Langella, L. Petrone, E. Ferrentino, P. Chiacchio, "Simplifying ROS2 controllers with a modular architecture for robot-agnostic reference generation". Under peer-review.

## Dependencies

This code requires the installation of Ubuntu 22.04 and [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html).

### System dependencies

To install the packages needed by this repo as system-wide dependencies, run the following command from the repository's root folder:

```bash
rosdep install --from-paths . -i
```

## Demonstration

### JRG + PD with gravity compensation + simulated FER.

First, build the workspace and source the setup file.
From the workspace root folder, run:

```bash
colcon build --packages-up-to fer_gravity_compensation_pd_controller_demo
source install/setup.bash
```

Launch the simulation using the provided launch file:

```bash
ros2 launch fer_gravity_compensation_pd_controller_demo gravity_compensation_pd_controller_gazebo_demo.launch.py
```

Then, in a new terminal, source the setup file again and run the following command to start sending joint space references to the controller:

```bash
source install/setup.bash
ros2 launch follow_joint_trajectory_action_client follow_joint_trajectory_action_client.launch.py input_trajectory_filename:=fer_exciting_acg_trajectory_42_real_0_no_vel action_name:=joint_space_reference_generator fraction_feedback_messages_to_save:=1
```


&copy; *2025 Automatic Control Group (DIEM, University of Salerno)*
