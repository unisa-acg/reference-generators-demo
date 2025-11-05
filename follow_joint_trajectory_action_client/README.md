# follow_joint_trajectory_action_client

This package implements an Action Client over the [FollowJointTrajectory](../acg_control_msgs/action/FollowJointTrajectory.action) action.
It can be used with any Action Server that expects the same message.

The trajectory to be sent as goal to the Action Server must be saved in a bag file, written in a [`JointTrajectory`](../acg_control_msgs/msg/JointTrajectory.msg) message.
During execution, feedback is recorded and stored in three separate trajectories representing the actual, desired, and error joint values. These three trajectories are written sequentially in the output bag file, in the following order:

1. ACTUAL trajectory
1. DESIRED trajectory
1. ERROR trajectory

This order is important to keep in mind, as the mapping between the recorded trajectories and their meaning is not encoded in the bag file itself.

**Note:** Currently, if the input bag file contains multiple trajectories in sequence, only the **first trajectory** will be read and sent as goal. Subsequent trajectories in the same bag file will be ignored. Make sure to create separate bag files for each trajectory if multiple executions are required.

The preferred usage of this node is within a launch file, through which it is possible to define the parameters required by the node:

* **`action_topic`**: the topic used by the joint space trajectory action server;
* **`input_trajectory_filename`**: the name of the input trajectory;
* **`fraction_feedback_messages_to_save`**: the portion of feedback messages to be saved: if its value is `N`, `1/N` of total number of messages will be output to bag file.
    * Defaults to `25`
* **`output_path`**: the (local) path where the output bag file will be saved
    * Defaults to the current working directory

Such parameters are defined via command line as arguments of the launch file.

## How to Test

Before running the demo, make sure this package is built and sourced in your workspace:

```bash
colcon build --packages-up-to follow_joint_trajectory_action_client
source install/setup.bash
```

This demo build upon [`acg_resources_ur10_moveit_config`](../acg_resources_ur10_moveit_config/README.md), so build this one too:

```bash
colcon build --packages-up-to acg_resources_ur10_moveit_config
source install/setup.bash
```

To test the action client and server setup, follow these steps:

1. **Launch the Action Server (Controller):**

   For example, to launch the UR10 MoveIt Gazebo demo with a mock controller:

   ```bash
   ros2 launch acg_resources_ur10_moveit_config gazebo_ros2_control_demo.launch.py controllers_file:=ur10_test_reference_controllers.yaml rviz_config_file:=config/joint_space_reference_generator_view_ee_pose.rviz initial_joint_controller:=pid_controller
   ```

2. **Load/Activate the Controller (if not already active):**

   Use the controller manager GUI to activate the required controller:

   ```bash
   ros2 run rqt_controller_manager rqt_controller_manager
   ```

   Select and activate the controller specified by the `action_name` parameter (see note below).

3. **Launch the Action Client to send the trajectory from bag file:**

   ```bash
   ros2 launch follow_joint_trajectory_action_client follow_joint_trajectory_action_client.launch.py input_trajectory_filename:=ur10_excitation_trajectory action_name:=joint_space_reference_generator fraction_feedback_messages_to_save:=25
   ```

   **Note:** The controller specified by the `action_name` parameter (e.g., `joint_space_reference_generator`) must be active in the controller manager before running the action client.
   If it is not active, activate it using the controller manager as shown above.

### Other trajectories

The [`fer_exciting_acg_trajectory_42_real`](./trajectories/fer_exciting_acg_trajectory_42_real/fer_exciting_acg_trajectory_42_real_0.db3) trajectory has been designed for the [FER robot](../acg_resources_fer_moveit_config/README.md), but a demo with a controller accepting it is currently under development.
Please consider it for future uses.
