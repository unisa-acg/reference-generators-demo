# UR10 Cartesian Controller Demo

This package provides a demo of the cartesian pose controller for the UR10 robot, which can be used in both simulation and on the real robot.

## Simulation with cartesian pose controller

To simulate the cartesian pose controller, properly configure the [`ur10_cpc_simulation.yaml`](./config/ur10_cpc_simulation.yaml) file.
The controller is designed to work via controller chaining, so the only way to send references to the cartesian pose controller is through the reference interfaces exposed by the `cartesian_pose_controller` controller.

For this reason, is convenient to work with the `task_space_reference_generator` controller, and send references or trajectories to the `task_space_reference_generator` controller, as described in the [`Simulation with task space reference generator`](../acg_resources_ur10_moveit_config/README.md#simulation-with-task-space-reference-generator) section, which will be forwarded to the `cartesian_pose_controller` controller.

To launch the demo, run:

```bash
ros2 launch acg_resources_ur10_moveit_config gazebo_ros2_control_demo.launch.py runtime_config_package:=ur10_cartesian_controller_demo controllers_file:=ur10_damped_cpc_simulation.yaml rviz_config_file:=config/task_space_reference_generator_view_ee_pose.rviz initial_joint_controller:=cartesian_pose_controller
```

Then, load and activate the `task_space_reference_generator` controller.
Use the following command in a new terminal to run the controller manager and select the desired controllers:

```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

The `task_space_reference_generator` can receive references or trajectories. For example, from the workspace folder, run the following command:

```bash
ros2 action send_goal /task_space_reference_generator/follow_task_space_trajectory acg_control_msgs/action/FollowTaskSpaceTrajectory "$(cat ./src/reference_generators_demo/ur10_cartesian_controller_demo/config/triangular_trajectory.yaml)" --feedback
```

The alternative is to use a custom launch file that loads the `task_space_reference_generator` controller and starts it automatically.
For example, launch the following command:

```bash
ros2 launch ur10_cartesian_controller_demo cartesian_controller_gazebo_demo.launch.py
```

Then, send the trajectory to the `task_space_reference_generator` controller using the command:

```bash
ros2 action send_goal /task_space_reference_generator/follow_task_space_trajectory acg_control_msgs/action/FollowTaskSpaceTrajectory "$(cat ./src/reference_generators_demo/ur10_cartesian_controller_demo/config/squared_trajectory.yaml)" --feedback
```

## Working with the real robot

In order to work using the real robot, execute the following command:

```bash
ros2 launch acg_resources_ur10_moveit_config ur10_control.launch.py runtime_config_package:=<package> controllers_file:=<controllers_file.yaml> robot_ip:=192.168.1.6
```

Note that if you want to work using the real robot and custom controllers, the `runtime_config_package` and `controllers_file` arguments must be specified.

Please, be aware that the file containing the controllers' configuration must contain at least the following controllers defined into the controllers configuration file:

* `io_and_status_controller`;
* `speed_scaling_state_broadcaster`;
* `force_torque_sensor_broadcaster`;
* `ur_configuration_controller`.

For an example of a controllers configuration file, please refer to [this controllers configuration file provided by an official Universal Robots repository](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/config/ur_controllers.yaml).

### Squared trajectory on the real robot

First, move the robot to the following initial position configuration:

| Joint               | Deg       | Rad     |
| :-----------------: | :-------: | :-----: |
| shoulder_pan_joint  |   84.7833 |  1.4797 |
| shoulder_lift_joint |  -47.8084 | -0.8344 |
| elbow_joint         |  107.9702 |  1.8844 |
| wrist_1_joint       |  119.8382 |  2.0916 |
| wrist_2_joint       |  -84.7833 | -1.4797 |
| wrist_3_joint       | -179.9947 | -3.1415 |

Then, launch the following command to start the controllers:

```bash
ros2 launch acg_resources_ur10_moveit_config ur10_control.launch.py runtime_config_package:=ur10_cartesian_controller_demo controllers_file:=ur10_cpc_real.yaml robot_ip:=192.168.1.6
```

Load and activate the `task_space_reference_generator` controller, by running the following command in a new terminal:

```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

Finally, send the trajectory to the `task_space_reference_generator` controller using the command:

```bash
ros2 launch follow_task_trajectory_action_client follow_task_trajectory_action_client.launch.py input_trajectory_filename:=ur10_squared_trajectory_real action_name:=task_space_reference_generator fraction_feedback_messages_to_save:=1
```
