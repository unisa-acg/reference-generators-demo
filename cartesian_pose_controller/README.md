# Cartesian Pose Controller

This package provides a Cartesian pose controller that converts desired end-effector poses and twists in task space to joint position commands for a robotic manipulator.
It is a chainable controller that works with the [`task space reference generator`](../reference_generator/README.md) to obtain references.

## Control Law

The control law is illustrated in the figure below:

![Control law](./doc/media/Cartesian_Pose_Controller_Control_Law.png)

Note that the task space references should be expressed in the `kinematics.base` frame (see [Controller Configuration](#controller-configuration)), which defines the coordinate frame used as the reference for all pose and twist commands sent to the controller. The `kinematics.base` parameter must match the root link described in the robot description, as the kinematics interface (e.g. the function `convert_cartesian_deltas_to_joint_deltas`) assumes this frame as the reference frame for its calculations.
If the desired twist is not provided (and the controller is configured correctly), the controller computes the desired twist from the desired pose.

To guarantee controller stability, users must ensure that the gain matrix `K` is positive definite.
The matrix `K` is constructed as a diagonal matrix from the user-provided gain vector `k_matrix_gains` (see [Controller Configuration](#controller-configuration)). These gains determine the controller's convergence speed.

## Controller Configuration

Control parameters for the cartesian pose controller are defined in the file [cartesian_pose_controller_parameters.yaml](src/cartesian_pose_controller_parameters.yaml).
Please refer to [`cartesian_pose_controller_parameters.md`](./doc/cartesian_pose_controller_parameters.md) for the documentation.

## Behavior in Edge Cases

The controller is designed to handle the following edge cases:

- **Computed positions/velocities exceeding joint limits**. If the computed positions/velocities exceed the joint limits, the controller will saturate the output to remain within the range specified in the robot description.

- **Reference too far from the current position**. If the commanded position/velocity is too far from the current robot position/velocity, the robot will stop and the controller will stop computing the control law and will go into an "error" state. To recover from this state, the user needs to stop the controller and restart it. To configure the tolerances refer to the [`cartesian_pose_controller_parameters.md`](./doc/cartesian_pose_controller_parameters.md).

- **Invalid task space reference**. If the task space reference contains any `NaN` values, the controller will fall back to the last valid commanded joint positions/velocities. Initially, the first valid commanded joint positions/velocities are set to the robot state.

## Demo

To test this controller in simulation, please refer to the documentation of [`ur10_cartesian_controller_demo`](../ur10_cartesian_controller_demo/README.md).
