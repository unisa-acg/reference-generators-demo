# ur10_admittance_controller_demo

This package contains the configuration files and launch files to run the admittance controller with the UR10 robot in Gazebo.

## Simulation demos

In this package, two demos are provided to show the admittance controller in action.
The first demo consists of a vertical wall sliding task, while the second one consists of an inclined ramp sliding task.
In both demos, the admittance controller is configured to work in chain with the `cartesian_pose_controller` controller, responsible for computing the IK of the compliant pose, and the `task_space_reference_generator` controller, responsible for providing task space references.

### Vertical wall sliding

In order to start the demo with the simulated UR10 and a vertical wall, run:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_gazebo_demo.launch.py
```

To command the robot to follow a trajectory along the wall, run the following command:

```bash
ros2 action send_goal /task_space_reference_generator/follow_task_space_trajectory acg_control_msgs/action/FollowTaskSpaceTrajectory "$(cat $(ros2 pkg prefix --share ur10_admittance_controller_demo)/config/wall_sliding_trajectory.yaml)" --feedback
```

To execute the same demo with the dynamic simulation, run the following command instead:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_gazebo_demo.launch.py enable_effort_interfaces:=true
```

### Inclined ramp sliding

In order to start the demo with the simulated UR10 and an inclined ramp, run:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_gazebo_demo.launch.py gazebo_world_file:=world/world_with_ft_sensor_and_ramp.sdf spawn_z:=1.0 initial_positions_file:="initial_positions_ramp.yaml" controllers_file:="ur10_simulation_admittance_controller_ramp.yaml"
```

To command the robot to follow a trajectory along the ramp, run the following command:

```bash
ros2 action send_goal /task_space_reference_generator/follow_task_space_trajectory acg_control_msgs/action/FollowTaskSpaceTrajectory "$(cat $(ros2 pkg prefix --share ur10_admittance_controller_demo)/config/ramp_sliding_trajectory.yaml)" --feedback
```

## Working with the real robot

In this package, two demos are provided to show the admittance controller in action with the real robot.
The first demo simply allows the user to physically interact with the robot experiencing the mass–spring–damper dynamics.
The second demo consists of a vertical blackboard sliding task.
In both demos, the admittance controller is configured to work in chain with the `cartesian_pose_controller` controller, responsible for computing the IK of the compliant pose, and the `task_space_reference_generator` controller, responsible for providing task space references.

### Physical human-robot interaction

In order to run the admittance controller on the real UR10 robot, start the communication with the robot by running:

```bash
ros2 launch ur10_admittance_controller_demo ur10_control.launch.py
```

After you click "play" on the teach pendant, open another shell and run the following command to load the controllers:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_demo.launch.py
```

After that, you can interact with the robot.

**Note:** this demo loads the parameter defined in [`ur10_admittance_controller_phri.yaml`](./config/ur10_admittance_controller_phri.yaml), tuned for a 6-DOF physical human-robot interaction (pHRI).

### Vertical blackboard sliding

Before starting the demo with the real UR10 sliding on a blackboard, brings the robot to the following configuration: [0.372, -1.439, 1.53, 3.056, -1.956, 1.577] radians ([21.32, −82.43, 87.64, 175.11, −112.06, 90.39] degrees).
After that, start the communication with the robot by running:

```bash
ros2 launch ur10_admittance_controller_demo ur10_control.launch.py controllers_file:=ur10_admittance_controller_blackboard.yaml
```

After you click "play" on the teach pendant, open another shell and run the following command to load the controllers:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_demo.launch.py
```

To command the robot to follow a trajectory along the blackboard, run the following command:

```bash
ros2 action send_goal /task_space_reference_generator/follow_task_space_trajectory acg_control_msgs/action/FollowTaskSpaceTrajectory "$(cat $(ros2 pkg prefix --share ur10_admittance_controller_demo)/config/blackboard_sliding_trajectory.yaml)" --feedback
```

**Note:** this demo loads the parameter defined in [`ur10_admittance_controller_blackboard.yaml`](./config/ur10_admittance_controller_blackboard.yaml), tuned for a blackboard writing task.
