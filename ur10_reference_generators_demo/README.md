# UR10 Reference Generators Demo

This package provides demonstrations for the UR10 robot using both task space and joint space reference generators. It includes configurations for simulating the UR10 robot in Gazebo with ROS 2 control.
More information about the reference generator can be found in the [README](../reference_generator/README.md).

## Simulations with Task Space Reference Generator

To simulate the UR10 robot in Gazebo using the task-space reference generator, the `cartesian_pose_controller` is used. This controller allows you to control the end-effector pose in Cartesian space.
The generated references or trajectories will be forwarded to the `cartesian_pose_controller`.

To launch the demo, run:

```bash
ros2 launch ur10_reference_generators_demo task_space_reference_generator_gazebo_demo.launch.py
```

Then, send the trajectory to the `task_space_reference_generator` using the command:

```bash
ros2 action send_goal /task_space_reference_generator/follow_task_space_trajectory xxx_control_msgs/action/FollowTaskSpaceTrajectory "$(cat $(ros2 pkg prefix --share ur10_reference_generators_demo)/config/squared_trajectory.yaml)" --feedback
```

The alternative is to load the `task_space_reference_generator` manually.
For example, launch the following command:

```bash
ros2 launch ur10_reference_generators_demo gazebo_ros2_control_demo.launch.py runtime_config_package:=ur10_reference_generators_demo controllers_file:=ur10_damped_cpc_simulation.yaml rviz_config_file:=config/task_space_reference_generator_view_ee_pose.rviz initial_joint_controller:=cartesian_pose_controller
```

Then, load and activate the `task_space_reference_generator`.
Use the following command in a new terminal to run the controller manager and select the desired controllers:

```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

The `task_space_reference_generator` can receive references or trajectories.
For example, run the following command:

```bash
ros2 action send_goal /task_space_reference_generator/follow_task_space_trajectory xxx_control_msgs/action/FollowTaskSpaceTrajectory "$(cat $(ros2 pkg prefix --share ur10_reference_generators_demo)/config/triangular_trajectory.yaml)" --feedback
```

## Simulations with Joint Space Reference Generator

To simulate the UR10 robot in Gazebo using the joint space reference generator, the `pid_controller` is used.
The generated references or trajectories will be forwarded to the `pid_controller`.

To launch the demo, run:

```bash
ros2 launch ur10_reference_generators_demo joint_space_reference_generator_gazebo.launch.py
```

Then, send the trajectory to the `joint_space_reference_generator` using the command:

```bash
ros2 action send_goal /joint_space_reference_generator/follow_joint_trajectory xxx_control_msgs/action/FollowJointTrajectory "$(cat $(ros2 pkg prefix --share ur10_reference_generators_demo)/config/test_joint_trajectory.yaml)" --feedback
```

The alternative is to load the `joint_space_reference_generator` manually.
For example, launch the following command:

```bash
ros2 launch ur10_reference_generators_demo gazebo_ros2_control_demo.launch.py runtime_config_package:=ur10_reference_generators_demo controllers_file:=ur10_pid_simulation.yaml rviz_config_file:=config/joint_space_reference_generator_view_ee_pose.rviz initial_joint_controller:=pid_controller initial_positions_package:=ur10_reference_generators_demo
```

Then, load and activate the `joint_space_reference_generator`.
Use the following command in a new terminal to run the controller manager and select the desired controllers:

```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

The `joint_space_reference_generator` can receive references or trajectories. For example, run the following command:

```bash
ros2 action send_goal /joint_space_reference_generator/follow_joint_trajectory xxx_control_msgs/action/FollowJointTrajectory "$(cat $(ros2 pkg prefix --share ur10_reference_generators_demo)/config/test_joint_trajectory.yaml)" --feedback
```
