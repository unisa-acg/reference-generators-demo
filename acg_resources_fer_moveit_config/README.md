# acg_resources_fer_moveit_config

This package contains the FER robot configuration used by the Automatic Control Group. It has been created through the MoveIt! Setup Assistant and then manually modified. Since the `.setup_assistant` file does not get modified upon manual modifications, additional functions shall be configured by manually editing the available launch and config files.
The `.setup_assistant` file is retained to allow the `MoveItConfigBuilder` to work properly, e.g. to load a custom SRDF file instead of the default `<config_dir_path>/<robot_name>.srdf` file.

Some of the modifications applied to this package are listed here below. The list is not exhaustive and it cannot be, as this package is likely to evolve with time and there is no reason to keep track of all the modifications with respect to the Setup Assistant's baseline in this Readme:

* `initial_velocities.yaml` and `initial_efforts.yaml` have been added to contain initial velocities and efforts for the `ros2_control` plugin. At the moment, `initial_velocities.yaml` is read by `fer.ros2_control.xacro`, while `initial_efforts.yaml` is only a placeholder in case it is going to be needed in the future.
* Default planners configurations that are not provided with this package are read from the official `moveit_configs_utils` package by the `move_group.launch.py` (also called by `demo.launch.py`). In this package `ompl_planning.yaml` has been manually configured starting from the default one in order for the user to be able to enforce planning in the joint space (if they wish to) when Cartesian constraints are set. This configuration now overrides the default one.
* The launch file `moveit_rviz.launch.py` has been modified to allow the user specifying relative paths when loading RViz configurations (in place of absolute ones).
* Two configurations are provided for RViz, namely `moveit.rviz` (default) and `planning_demo.rviz`. The former loads the MotionPlanning plugin, while the latter loads several plugins for external applications to send results to RViz, like planned joint space and end-effector trajectories, obstacles and replay of executions.

## Usage with Motion Planning plugin

In order to execute a demo with RViz's **MotionPlanning** plugin, run

```bash
ros2 launch acg_resources_fer_moveit_config demo.launch.py
```

## Usage with external planning application

The only difference with the previous modality is the RViz's configuration, which loads **MarkerArray**, **PlanningScene** and **Trajectory** plugins for external applications to effectively visualize the results of planning and execution:

```bash
ros2 launch acg_resources_fer_moveit_config demo.launch.py rviz_config:=planning_demo.rviz
```

## Trajectory execution

Together with planning, this package supports trajectory execution through `ros2_control`. Several configurations are available, described here below.

### Execution with mock hardware and MoveIt! reference generator

The `demo.launch.py` demo provides a basic execution demo using mock hardware, loading a `ros2_control_node` (identified by `ros2 node list` as `/controller_manager`) and spawning a joint trajectory controller. In this case, a `moveit_simple_controller_manager` is also instantiated, which, despite its name, provides action clients connecting to the controllers loaded by the (real) controller manager. For more information, please refer to [this tutorial](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html).

### Execution with mock hardware and external reference generator

If the user intends to provide reference generators outside of MoveIt!, they can rely on the following demo:

```bash
ros2 launch acg_resources_fer_moveit_config ros2_control_demo.launch.py
```

The demo loads a forward position controller, therefore position references can be sent by an external reference generator, such as `ros2 topic pub`, e.g.

```bash
ros2 topic pub /fer_forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, -0.7, 0.0, 0.3, 0.0, 0.0]"
```

When using mock hardware, remember that commands are just copied in the state, but no simulation really happens. Therefore, if switching to a velocity controller, the user shall not expect the mock robot to move. However, its state, as read by `/joint_states`, shall update according to the commands.

### Execution with Gazebo

This demo is the same as the demo above, but `gz_ros2_control::GazeboSimROS2ControlPlugin` plugin is used instead of launching a dedicated `ros2_control_node`. This means that the control node (and, therefore, the controller manager) is started by Gazebo at the time the plugin is loaded. Also, in terms of hardware, the `gz_ros2_control/GazeboSimSystem` is used instead of mock hardware.

To launch the demo, run

```bash
ros2 launch acg_resources_fer_moveit_config gazebo_ros2_control_demo.launch.py
```

This demo launches the forward position controller by default, but it can be changed at launch time by properly setting the `controller` parameter.

Position references can be sent to the Gazebo robot by an external reference generator, such as `ros2 topic pub`, e.g.

```bash
ros2 topic pub /fer_forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, -0.7, 0.0, 0.3, 0.0, 0.0]"
```

Some remarks:

* The robot falls under the effect of gravity and stops falling when the motion controller is loaded; differently from old versions of Gazebo, the controllers are only loaded if the simulator is not paused; this prevents loading the controllers before the action of gravity.
* If switching to a velocity controller, remember that the `data` field in the `Float64MultiArray` shall be a vector of 7 elements (instead of 8): the gripper fingers cannot be velocity-commanded.
* When moving the robot in Gazebo, remember that only one of the fingers is controlled, while the other should mirror; however, at the time of writing, no mimicking plugin has been configured.

### Execution with Gazebo and MoveIt! planning system

This package also offers a complete planning and simulation setup with the following demo:

```bash
ros2 launch acg_resources_fer_moveit_config moveit_gazebo_ros2_control_demo.launch.py
```

Here, the user can plan trajectories through the MoveIt! planning pipeline by using the `MotionPlanning` plugin in RViz. Execution is performed on the Gazebo robot through the `moveit_simple_controller_manager`, which uses the loaded joint trajectory controller.

Some remarks:

* In this demo, the fall is recovered by spawning a reference generator at launch time, which sends references to a temporarily loaded forward position controller, then, when the robot is up, such controller is replaced with a joint trajectory controller.
* Recovering from the fall is a necessary step for the simulator to work as expected when receiving trajectory references: sometimes, after falling, joint 4 seems "locked" and this prevents trajectories to be correctly executed; this looks like a bug, even though, at the time of writing, we could not find any confirmation of this online.

## Launch File Parameters

### Common Parameters

All launch files included in this package support the following two command-line arguments:

* **`tf_prefix`**: Specifies the prefix to be applied to all TF frames and joint names. The default value is `fer_`, which identifies the *FER* robot instance used in our setup.

    Other supported prefixes correspond to models developed by **Franka Emika**, including:

    * `fer_`
    * `fp3_`
    * `fr3_`
    * `fr3_duo_`
    * `fr3v2_`

    The selected prefix must match the naming used in the configuration files for controllers and motion planning.

* **`hand`**: A boolean flag that enables or disables the presence of the robot’s hand (gripper). The default value is `true`. If set to `false`, the controller configuration files must be appropriately adjusted to remove references to the hand joints and actions.

## Known issues

If the `ros2_control.xacro` file contains both effort interfaces and other interfaces (e.g., position, velocity), the simulation with the latter interfaces does not work.
This issue is explained in [this issue](https://github.com/ros-controls/gz_ros2_control/issues/343).
A workaround for this is to create a separate Xacro file for the effort interfaces (see [this file](./config/fer.ros2_control_effort.xacro)) and load it only when necessary.

The following startup errors and warnings are expected because they concern parts of the system that are not configured:

```text
[move_group-3] [WARN] [1690472142.374495961] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-3] [ERROR] [1690472142.374504145] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[move_group-3] [ERROR] [1690472142.389909353] [moveit.ros_planning.planning_pipeline]: Exception while loading planner 'chomp_interface/CHOMPPlanner': According to the loaded plugin descriptions the class chomp_interface/CHOMPPlanner with base class type planning_interface::PlannerManager does not exist. Declared types are  ompl_interface/OMPLPlanner pilz_industrial_motion_planner/CommandPlannerAvailable plugins: ompl_interface/OMPLPlanner, pilz_industrial_motion_planner/CommandPlanner
[move_group-3] [ERROR] [1690472142.390715494] [moveit.ros_planning_interface.moveit_cpp]: Failed to initialize planning pipeline 'chomp'.
[rviz2-4] [ERROR] [1690472145.719367862] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
```

The following startup warning is not expected, but the same behavior arises in official MoveIt!2 tutorials:

```text
[rviz2-4] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
```

The following shutdown warnings are not expected, but the same behavior arises in official MoveIt!2 tutorials:

```text
[move_group-3] Warning: class_loader.ClassLoader: SEVERE WARNING!!! Attempting to unload library while objects created by this loader exist in the heap! You should delete your objects before attempting to unload the library or destroying the ClassLoader. The library will NOT be unloaded.
[rviz2-4] [WARN] [1690472155.865821471] [interactive_marker_display_94774200196176]: Server not available while running, resetting
[rviz2-4]
[rviz2-4] >>> [rcutils|error_handling.c:108] rcutils_set_error_state()
[rviz2-4] This error state is being overwritten:
[rviz2-4]
[rviz2-4]   'rcl node's context is invalid, at ./src/rcl/node.c:428'
[rviz2-4]
[rviz2-4] with this new error message:
[rviz2-4]
[rviz2-4]   'publisher's context is invalid, at ./src/rcl/publisher.c:389'
[rviz2-4]
[rviz2-4] rcutils_reset_error() should be called after error handling to avoid this.
[rviz2-4] <<<
```
