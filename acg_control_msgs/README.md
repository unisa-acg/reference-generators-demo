# acg_control_msgs

This package defines new messages and actions following the structure and style of [control_msgs](https://github.com/ros-controls/control_msgs/tree/humble) and [trajectory_msgs](https://github.com/ros2/common_interfaces/tree/humble/trajectory_msgs).
It includes the definition of several messages:

* `AdmittanceControllerState`: a message representing the admittance controller's state;
* `JointTrajectory`: a message representing a joint trajectory;
* `JointTrajectoryPoint`: a message representing a joint trajectory point;
* `JointWrenchPoint`: a message representing a joint point with a wrench;
* `TaskSpaceTrajectory`: a message representing a task-space tajectory;
* `TaskSpaceTrajectoryPoint`: a message representing a task-space tajectory point;
* `TaskPoint`: a message representing a task-space point;

Additionally, the package includes the following actions:

* `FollowJointTrajectory`: an action enabling a manipulator to follow a joint-space trajectory;
* `FollowTaskSpaceTrajectory`: an action enabling a manipulator to follow a task-space trajectory;

Consult the `msg` and `action` directories for more information on the messages and actions defined in this package.
