# fer Gravity Compensation PD Controller Demo

This package provides a demo of the PD controller with gravity compensation for the fer robot, which can be used in simulation.

## Simulation with gravity compensation PD controller

To simulate the PD controller with gravity compensation, properly configure the [`fer_gc_pd_simulation.yaml`](./config/fer_gc_pd_simulation.yaml) file.
The controller is designed to work via controller chaining, so the only way to send references to the PD controller with gravity compensation is through the reference interfaces exposed by the controller.

For this reason, it is convenient to work with the `joint_space_reference_generator`, and send references or trajectories to it via topic or action interface.
The references will be interpolated by the `joint_space_reference_generator` and forwarded to the `gravity_compensation_pd_controller`.

To launch the demo, run:

```bash
ros2 launch fer_gravity_compensation_pd_controller_demo gravity_compensation_pd_controller_gazebo_demo.launch.py
```

Then, send the trajectory to the `joint_space_reference_generator` using the command:

```bash
ros2 launch follow_joint_trajectory_action_client follow_joint_trajectory_action_client.launch.py input_trajectory_filename:=fer_exciting_acg_trajectory_42_real_0_no_vel action_name:=joint_space_reference_generator fraction_feedback_messages_to_save:=1
```

## Test with real robot

### Move to initial position (Optional)

Send the robot to the desired initial joint positions:

```bash
ros2 launch acg_resources_fer_moveit_config moveit_fer.launch.py robot_ip:=192.168.1.1
```

```bash
ros2 topic pub --once /fer_arm_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{"joint_names":["fer_joint1","fer_joint2","fer_joint3","fer_joint4","fer_joint5","fer_joint6","fer_joint7"], "points":[{"positions":[-2.25558,-0.471367,0.777765,-1.712816,-0.357352,1.553972,0.483476], "time_from_start":{"sec":3,"nanosec":0}}]}'
```

### Run the PD controller with gravity compensation

Launch the controller:

```bash
ros2 launch fer_gravity_compensation_pd_controller_demo gravity_compensation_pd_controller_demo.launch.py
```

Then, send the trajectory to the `joint_space_reference_generator` using the command:

```bash
ros2 launch follow_joint_trajectory_action_client follow_joint_trajectory_action_client.launch.py input_trajectory_filename:=fer_exciting_acg_trajectory_42_real_0_no_vel action_name:=joint_space_reference_generator fraction_feedback_messages_to_save:=1
```
