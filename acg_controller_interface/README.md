# acg_controller_interface

This package contains custom semantic components, a customized semantic component command interface, and their implementations for the Automatic Control Group.
The semantic component command interface was manually added in this package from `ros2_control` `Jazzy` ([link](https://github.com/ros-controls/ros2_control/blob/master/controller_interface/include/semantic_components/semantic_component_command_interface.hpp), commit sha: 2dc3725) because ROS2 `Humble` did not provide it.
In addition, it has been modified and expanded to allow users to handle controller references with the same semantic component command interface.

The following semantic components are available in this package:

- `semantic_component::PositionEncoder`: It provides the current joint positions of the robot.
- `semantic_component::VelocityEncoder`: It provides the current joint velocities of the robot.
- `semantic_component::Accelerometer`: It provides the current joint accelerations of the robot.
- `semantic_component::EffortSensor`: It provides the current joint efforts of the robot.
- `semantic_component::TwistSensor`: It provides the twist of the robot.

The following semantic component commands are available in this package:

- `semantic_components::JointPositionCommand`: It allows users to set the desired joint positions of the robot and read the joint positions written in the command.
- `semantic_components::JointVelocityCommand`: It allows users to set the desired joint velocities of the robot and read the joint velocities written in the command.
- `semantic_components::JointAccelerationCommand`: It allows users to set the desired joint accelerations of the robot and read the joint accelerations written in the command.
- `semantic_components::JointEffortCommand`: It allows users to set the desired joint efforts of the robot and read the joint efforts written in the command.
- `semantic_components::PoseCommand`: It allows users to set the desired pose of the robot and read the pose written in the command.
- `semantic_components::TwistCommand`: It allows users to set the desired twist of the robot and read the twist written in the command.
- `semantic_components::AccelCommand`: It allows users to set the desired task-space acceleration of the robot and read the acceleration written in the command.
- `semantic_components::WrenchCommand`: It allows users to set the desired wrench of the robot and read the wrench written in the command.
- `semantic_components::WrenchDerivativeCommand`: It allows users to set the desired wrench derivative of the robot and read the wrench derivative written in the command.

The development of this package follows the same implementation style as the `ros2_control` semantic components and semantic component commands.
This involves creating header-only libraries for the components and their implementations. A GitHub issue discussing the reasons behind this choice can be found [here](https://github.com/ros-controls/ros2_control/issues/2072).

## How to test

If you want to perform the tests contained in this package after the build, you must launch the following command:

```bash
colcon test --packages-select acg_controller_interface
```

If the terminal does not show the test details, you should run the following command:

```bash
colcon test-result --all --verbose
```

### Optional analysis

If you wish to see the INFO messages printed to the console during the test, run the following:

```bash
colcon test --packages-select acg_controller_interface  --event-handlers console_cohesion+
```

The expected output looks like this:

```text
100% tests passed, 0 tests failed out of 1
```
