# follow_task_trajectory_action_client

This package implements an Action Client over the [FollowTaskSpaceTrajectory](../acg_control_msgs/action/FollowTaskSpaceTrajectory.action) action.
It can be used with any Action Server that expects the same message.

The trajectory to be sent as goal to the Action Server must be saved in a bag file.
During execution, feedback is recorded and stored in three separate trajectories representing the actual, desired, and error task-space robot state.
These three trajectories are written sequentially in the output bag file, in the following order:

* ACTUAL trajectory
* DESIRED trajectory
* ERROR trajectory

This order is important to keep in mind, as the mapping between the recorded trajectories and their meaning is not encoded in the bag file itself.

**Note:** Currently, if the input bag file contains multiple trajectories in sequence, only the **first trajectory** will be read and sent as goal. Subsequent trajectories in the same bag file will be ignored. Make sure to create separate bag files for each trajectory if multiple executions are required.

The preferred usage of this node is within a launch file, through which it is possible to define the parameters required by the node:

* **`action_topic`**: the topic used by the task space trajectory action server;
* **`input_trajectory_filename`**: the name of the input trajectory;
* **`fraction_feedback_messages_to_save`**: the portion of feedback messages to be saved: if its value is `N`, `1/N` of total number of messages will be output to bag file.
    * Defaults to `25`
* **`output_path`**: the (local) path where the output bag file will be saved
    * Defaults to the current working directory

Such parameters are defined via command line as arguments of the launch file.

## How to Test

Before running the demos, make sure this package is built and sourced in your workspace:

```bash
colcon build --packages-up-to follow_task_trajectory_action_client
source install/setup.bash
```

You can choose between three demos:

1. [Cartesian controller on the simulated robot](#cartesian-controller-on-the-simulated-robot)
1. [Cartesian controller on the real robot](#cartesian-controller-on-the-real-robot)
1. [Admittance controller on the real robot](#admittance-controller-on-the-real-robot)

### Cartesian controller on the simulated robot

This demo build upon [`ur10_cartesian_controller_demo`](../ur10_cartesian_controller_demo/README.md), so build this package too:

```bash
colcon build --packages-up-to ur10_cartesian_controller_demo
source install/setup.bash
```

To test the action client and server setup, follow these steps:

1. **Launch the Action Server (Controller):**

   ```bash
   ros2 launch ur10_cartesian_controller_demo cartesian_controller_gazebo_demo.launch.py
   ```

1. **Launch the Action Client to send the trajectory from bag file:**

   ```bash
   ros2 launch follow_task_trajectory_action_client follow_task_trajectory_action_client.launch.py input_trajectory_filename:=ur10_squared_trajectory action_name:=task_space_reference_generator fraction_feedback_messages_to_save:=25
   ```

   **Note:** The controller specified by the `action_name` parameter (e.g., `task_space_reference_generator`) must be active in the controller manager before running the action client.
   If it is not active, activate it using the controller manager, e.g. with

   ```bash
   ros2 run rqt_controller_manager rqt_controller_manager
   ```

### Cartesian controller on the real robot

This demo build upon [`ur10_cartesian_controller_demo`](../ur10_cartesian_controller_demo/README.md), so build this package too:

```bash
colcon build --packages-up-to ur10_cartesian_controller_demo
source install/setup.bash
```

Please follow [these intrusctions](../ur10_cartesian_controller_demo/README.md#squared-trajectory-on-the-real-robot) to proceed with the demo.

### Admittance controller on the real robot

This demo build upon [`ur10_admittance_controller_demo`](../ur10_admittance_controller_demo/README.md), so build this package too:

```bash
colcon build --packages-up-to ur10_admittance_controller_demo
source install/setup.bash
```

To test the action client and server setup, follow these steps:

1. **Initial configuration:**

   Before starting the demo with the real UR10 sliding on a blackboard, brings the robot to the following configuration: [0.372, -1.439, 1.53, 3.056, -1.956, 1.577] radians ([21.32, −82.43, 87.64, 175.11, −112.06, 90.39] degrees).

1. **Run the robot driver:**

   Start the communication with the robot by running:

   ```bash
   ros2 launch ur10_admittance_controller_demo ur10_control.launch.py controllers_file:=ur10_admittance_controller_blackboard.yaml
   ```

1. **Launch the Action Server (Controller):**

   After you click "play" on the teach pendant, open another shell and run the following command to load the controller:

   ```bash
   ros2 launch ur10_admittance_controller_demo admittance_controller_demo.launch.py
   ```

1. **Launch the Action Client to send the trajectory from bag file:**

   ```bash
   ros2 launch follow_task_trajectory_action_client follow_task_trajectory_action_client.launch.py input_trajectory_filename:=ur10_blackboard_sliding_trajectory action_name:=task_space_reference_generator fraction_feedback_messages_to_save:=25
   ```

## Trajectory origin

The following trajectories were generated from the following YAML files:

* [`ur10_squared_trajectory`](./trajectories/ur10_squared_trajectory/ur10_squared_trajectory_0.db3): generated using the [`squared_trajectory.yaml`](../ur10_cartesian_controller_demo/config/squared_trajectory.yaml) file in the [`ur10_cartesian_controller_demo`](../ur10_cartesian_controller_demo/README.md#simulation-with-cartesian-pose-controller) package.
* [`ur10_squared_trajectory_real`](./trajectories/ur10_squared_trajectory_real/ur10_squared_trajectory_real_0.db3): generated using the [`squared_trajectory_real.yaml`](../ur10_cartesian_controller_demo/config/squared_trajectory_real.yaml) file in the [`ur10_cartesian_controller_demo`](../ur10_cartesian_controller_demo/README.md#squared-trajectory-on-the-real-robot) package.
* [`ur10_blackboard_sliding_trajectory`](./trajectories/ur10_blackboard_sliding_trajectory/ur10_blackboard_sliding_trajectory_0.db3): generated using the [`blackboard_sliding_trajectory.yaml`](../ur10_admittance_controller_demo/config/blackboard_sliding_trajectory.yaml) file in the [`ur10_admittance_controller_demo`](../ur10_admittance_controller_demo/README.md#vertical-blackboard-sliding) package.
