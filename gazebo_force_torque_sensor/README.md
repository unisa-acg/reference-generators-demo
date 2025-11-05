# gazebo_force_torque_sensor

This package provides a concrete implementation of a `gz_ros2_control::GazeboSimSystemInterface`, enabling communication with a simulated Force/Torque (F/T) sensor in the Ignition Gazebo environment.
The purpose of the plugin `gz_ros2_control::GazeboFTSensor`, developed in this package, is to simulate the presence of an F/T sensor within the ROS2 architecture, delivering F/T measurements in accordance with the `ros2_control` framework.

- The [`include`](include/) and [`src`](src/) folders contain header and source files developing the [`GazeboFTSensor`](./include/gazebo_force_torque_sensor/gazebo_ft_sensor.hpp) plugin.
- The [`config`](./config/) folder contain :
    - [`gazebo_fts.ros2_control.xacro`](./config/gazebo_fts.ros2_control.xacro): defines the configuration for the `GazeboFTSensor`;
    - [`gazebo_force_torque_measurements_plot.xml`](./config/gazebo_force_torque_measurements_plot.xml): defines the configuration file for the Plotjuggler layout.

The force/torque sensor plugin offers the ability to compensate for both sensor and gravity bias.
For the bias to be calculated accurately, the robot must remain stationary throughout the entire bias computation process.
Once the bias is calculated, it remains valid only if the robot continues operating while maintaining the same orientation it had during the computation.
If the robot operates in a different orientation, the bias must be recalculated restarting the plugin.

**Note: we recommend setting the `number_of_samples_for_computing_bias` parameter to at least 10 so that stable sensor’s measurements will be taken into account.**

## Launch demos

For demo launches, please refer to the [`Simulation with force/torque sensor`](../acg_resources_ur10_moveit_config/README.md#simulation-with-forcetorque-sensing) section in the README of `acg_resources_ur10_moveit_config` package.
