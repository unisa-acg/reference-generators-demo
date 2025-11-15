# Reference Generators and Repository Overview

This is the accompanying code of the paper

> D. Risi, V. Petrone, A. Langella, L. Petrone, E. Ferrentino, P. Chiacchio, "Simplifying ROS2 controllers with a modular architecture for robot-agnostic reference generation". Under peer-review.

## Getting Started

Let's start by creating and populating the colcon workspace with the `unisa_acg_ros2` package suite and its dependencies. 
In this case we will use `wstool` for convenience, which can be installed via aptitude:

```bash
sudo apt install python3-wstool
```

After having cloned the repository `reference-generators-demo` under `src` folder, checkout to the branch named `rap`:

```bash
git checkout rap
```

To populate the workspace with the package suite and its dependencies, under your workspace's `src` folder, run:

```bash
wstool init . reference-generators-demo/reference_generators_demo.rosinstall
```

**Note**: if a workspace config file `.rosinstall` has been already added to the workspace, update such file with the new dependencies and install them by running:

```bash
wstool merge -t . reference-generators-demo/reference_generators_demo.rosinstall
wstool update -t .
```

## Dependencies

This code requires the installation of Ubuntu 22.04 and [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html).

### System dependencies

To install the packages needed by this repo as system-wide dependencies, run the following command from the repository's root folder:

```bash
rosdep install --from-paths . -i
```

## Demonstration

The following sections show how to run simulations in Gazebo using the proposed Joint Space Reference Generator (JRG) and Task Space Reference Generator (TRG) with four controllers—PD with gravity compensation, PID, CPC, and AC—on the Franka Emika Robot (FER) and UR10.

Before starting, compile from the workspace root folder:
```bash
colcon build
source install/setup.bash
```

Note: In each new terminal, run `source install/setup.bash` before executing any ROS2 commands.

### JRG + PD with gravity compensation + simulated FER

Launch the simulation, which starts FER with PD controller with gravity compensation, and the JRG:
```bash
ros2 launch fer_gravity_compensation_pd_controller_demo gravity_compensation_pd_controller_gazebo_demo.launch.py
```

In a new terminal, send the joint space trajectory:
```bash
ros2 launch follow_joint_trajectory_action_client follow_joint_trajectory_action_client.launch.py input_trajectory_filename:=fer_exciting_acg_trajectory_42_real_0_no_vel action_name:=joint_space_reference_generator fraction_feedback_messages_to_save:=1
```

### JRG + PID + simulated FER

Launch the simulation, which starts FER with PID controller:
```bash
ros2 launch acg_resources_fer_moveit_config gazebo_ros2_control_demo.launch.py hand:=false controller:=pid_controller enable_effort_interfaces:=true controllers_file:=fer_reference_generator.yaml
```

Then, load and activate the `joint_space_command_controller`, for example using the GUI:
```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

In a new terminal, send the joint space trajectory:
```bash
ros2 launch follow_joint_trajectory_action_client follow_joint_trajectory_action_client.launch.py input_trajectory_filename:=fer_exciting_acg_trajectory_42_real action_name:=joint_space_reference_generator fraction_feedback_messages_to_save:=1
```

### JRG + PID + simulated UR10

Launch the simulation, which starts UR10 with PID controller:
```bash
ros2 launch acg_resources_ur10_moveit_config gazebo_ros2_control_demo.launch.py controllers_file:=ur10_test_reference_controllers.yaml rviz_config_file:=config/joint_space_reference_generator_view_ee_pose.rviz initial_joint_controller:=pid_controller
```

Then, load and activate `joint_space_command_controller`, for example using the GUI:
```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

In a new terminal, send the joint space trajectory:
```bash
ros2 launch follow_joint_trajectory_action_client follow_joint_trajectory_action_client.launch.py input_trajectory_filename:=ur10_validation_trajectory action_name:=joint_space_reference_generator fraction_feedback_messages_to_save:=1
```

### TRG + CPC + simulated UR10

Launch the simulation, which starts UR10 with CPC and TRG:
```bash
ros2 launch ur10_cartesian_controller_demo cartesian_controller_gazebo_demo.launch.py
```

In a new terminal, send task-space trajectory:
```bash
ros2 launch follow_task_trajectory_action_client follow_task_trajectory_action_client.launch.py input_trajectory_filename:=ur10_squared_trajectory action_name:=task_space_reference_generator fraction_feedback_messages_to_save:=1
```

### TRG + AC + CPC + PID + simulated UR10

Launch the simulation, which starts UR10 with CPC, AC, and TRG:
```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_gazebo_demo.launch.py enable_effort_interfaces:=true
```
In a new terminal, send task-space trajectory:
```bash
ros2 launch follow_task_trajectory_action_client follow_task_trajectory_action_client.launch.py input_trajectory_filename:=wall_sliding_trajectory action_name:=task_space_reference_generator fraction_feedback_messages_to_save:=1
```

&copy; *2025 Automatic Control Group (DIEM, University of Salerno)*
