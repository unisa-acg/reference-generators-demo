# force_torque_sensors

This is a container for force/torque sensor description packages that can be part of an end-effector.

## How to extend the force/torque sensors library

To extend the force/torque sensors library with a new sensor, the description package which will contain the new sensor should be identified.
This could be done by creating a new package in this folder, or by adding a new sensor to an existing package.

Each force/torque sensor is represented by a `.urdf.xacro` file in the `urdf` folder of the package.
The xacro file must define a property `ft_sensor_last_link` in the outermost xacro namespace, which should be set to the name of the last link of the sensor.
This property may not be defined before calling the `build_force_torque_sensor` macro, but must have a definite value after the macro is called, as it will be used to attach more components on the tip of the sensor.
The `.urdf.xacro` file must define a xacro macro named `build_force_torque_sensor` that generates the URDF description of the sensor from the following parameters:

* `mounting_link`: the name of the link to which the sensor is mounted;
* `sensed_joint_name`: the name of the joint that senses the force/torque sensor.
  This joint should be only defined if the `enable_ft_sensing` parameter is set to `true`.
* `sensing_frame_name`: the name of the frame in which the force/torque sensor senses the forces and torques.
  This frame should be only defined if the `enable_ft_sensing` parameter is set to `true`.
* `config`: a dictionary containing the configuration parameters of the sensor provided by the user.
  The provided configuration parameters should be used to generate the URDF description of the sensor.
  A configuration parameter can be either mandatory or optional (i.e. it has a default value).
  To read the value of a configuration parameter, the [`define_property_from_dict`](../../acg_common_libraries/xacro/define_property_from_dict.xacro) macro should be used.
  For more information on how to use this macro, please refer to the [`acg_common_libraries` package README](../../acg_common_libraries/README.md#using-the-define_property_from_dict-macro) or any of the existing mountings in this library.
* `enable_ft_sensing`: a boolean that indicates whether the force/torque sensor should sense forces and torques.
  If `true`, the `sensed_joint_name` and `sensing_frame_name` parameters should be defined.
  If `false`, the sensor will not sense forces and torques, but it will still be mounted to the robot.
  This parameter should be captured by the parent xacro namespace with the `^` operator and have a default value of `false`.
  For more information on the `^` operator, refer to the official [xacro wiki](https://github.com/ros/xacro/wiki#default-parameter-values).
  For an example of how to use this parameter, refer to any of the existing sensors in this library.
* `sim_ignition`: a boolean that indicates whether the sensor should be simulated in Ignition Gazebo.
  If `true`, gazebo-specific tags should be added to the URDF description of the sensor.
  This parameter should be captured by the parent xacro namespace with the `^` operator and have a default value of `false`.
  For more information on the `^` operator, refer to the official [xacro wiki](https://github.com/ros/xacro/wiki#default-parameter-values).
  For an example of how to use this parameter, refer to any of the existing sensors in this library.

Furthermore, after calling the macro:

* the local `name` property should be set to the name of the sensor link;
* the local `ft_sensor_last_link` property should be set to the name of the last link of the sensor.

**Naming convention:** The link of the force/torque sensor should be named `ft_sensor`.
