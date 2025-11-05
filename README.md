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

### JRG + PID + simulated FER

First, build the workspace and source the setup file.
From the workspace root folder, run:

```bash
colcon build --packages-up-to acg_resources_fer_moveit_config
colcon build --packages-up-to follow_joint_trajectory_action_client
source install/setup.bash
```

Launch the simulation using the provided launch file:

```bash
ros2 launch acg_resources_fer_moveit_config gazebo_ros2_control_demo.launch.py hand:=false controller:=pid_controller enable_effort_interfaces:=true controllers_file:=fer_reference_generator.yaml
```

Then, load and activate the `joint_space_command_controller` by running the following commands:

```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

Finally, send the excitation trajectory with the following command:

```bash
source install/setup.bash
ros2 launch follow_joint_trajectory_action_client follow_joint_trajectory_action_client.launch.py input_trajectory_filename:=fer_exciting_acg_trajectory_42_real action_name:=joint_space_reference_generator fraction_feedback_messages_to_save:=1
```

### JRG + PID + simulated UR10

First, build the workspace and source the setup file.
From the workspace root folder, run:

```bash
colcon build --packages-up-to acg_resources_ur10_moveit_config
colcon build --packages-up-to follow_joint_trajectory_action_client
source install/setup.bash
```

Launch the simulation using the provided launch file:

```bash
ros2 launch acg_resources_ur10_moveit_config gazebo_ros2_control_demo.launch.py controllers_file:=ur10_test_reference_controllers.yaml rviz_config_file:=config/joint_space_reference_generator_view_ee_pose.rviz initial_joint_controller:=pid_controller
```

Then, load and activate the `joint_space_command_controller` by running the following commands:

```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

Finally, send the excitation trajectory with the following command:

```bash
source install/setup.bash
ros2 launch follow_joint_trajectory_action_client follow_joint_trajectory_action_client.launch.py input_trajectory_filename:=ur10_excitation_trajectory action_name:=joint_space_reference_generator fraction_feedback_messages_to_save:=1
```

### TRG + CPC + simulated UR10

First, build the workspace and source the setup file.
From the workspace root folder, run:

```bash
colcon build --packages-up-to ur10_cartesian_controller_demo
colcon build --packages-up-to follow_task_trajectory_action_client
source install/setup.bash
```

Launch the simulation using the provided launch file, which will start the UR10 with the CPC and TRG controllers:

```bash
ros2 launch ur10_cartesian_controller_demo cartesian_controller_gazebo_demo.launch.py
```

Then, send the trajectory to the `task_space_reference_generator` controller using the command:

```bash
source install/setup.bash
ros2 launch follow_task_trajectory_action_client follow_task_trajectory_action_client.launch.py input_trajectory_filename:=ur10_squared_trajectory action_name:=task_space_reference_generator fraction_feedback_messages_to_save:=1
```

### TRG + AC + CPC + simulated UR10

First, build the workspace and source the setup file.
From the workspace root folder, run:

```bash
colcon build --packages-up-to ur10_admittance_controller_demo
colcon build --packages-up-to follow_task_trajectory_action_client
source install/setup.bash
```

Launch the simulation using the provided launch file, which will start the UR10 with the CPC, AC, and TRG controllers:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_gazebo_demo.launch.py
```

Then, send the trajectory to the `task_space_reference_generator` controller using the command:

```bash
source install/setup.bash
ros2 launch follow_task_trajectory_action_client follow_task_trajectory_action_client.launch.py input_trajectory_filename:=wall_sliding_trajectory action_name:=task_space_reference_generator fraction_feedback_messages_to_save:=1
```

&copy; *2025 Automatic Control Group (DIEM, University of Salerno)*
