# UR10 Gravity Compensation PD Controller Demo

This package provides a demo of the PD controller with gravity compensation for the UR10 robot, which can be used in simulation.

## Simulation with gravity compensation PD controller

To simulate the PD controller with gravity compensation, properly configure the [`ur10_gc_pd_simulation.yaml`](./config/ur10_gc_pd_simulation.yaml) file.
The controller is designed to work via controller chaining, so the only way to send references to the PD controller with gravity compensation is through the reference interfaces exposed by the controller.

For this reason, it is convenient to work with the `joint_trajectory_controller`, and send references or trajectories to it via topic or action interface.
The references will be interpolated by the `joint_trajectory_controller` and forwarded to the `gravity_compensation_pd_controller`.

To launch the demo, run:

```bash
ros2 launch ur10_gravity_compensation_pd_controller_demo gravity_compensation_pd_controller_gazebo_demo.launch.py
```

Then, send the trajectory to the `joint_trajectory_controller` using the command:

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "$(cat $(ros2 pkg prefix --share ur10_gravity_compensation_pd_controller_demo)/config/validation_trajectory.yaml)" --feedback
```
