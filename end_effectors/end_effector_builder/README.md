# end_effector_builder

This package provides a system to build end-effector descriptions (URDFs) and the associated semantic descriptions (SRDFs) from a configuration file written in `yaml` format.
This `yaml` file describes the components of the end-effector, such as the force/torque sensor, the tool and their respective mountings.
It serves as the main input to the macros provided by this package, specifying the structure and parameters of the end-effector to be generated.

**Purpose of the `yaml` configuration file:**

* The `yaml` file defines the components and parameters of the end-effector in a structured way.
* The format of the `yaml` file defined in this package is a standard that can be used across different end-effectors.
  The standard may be extended by external packages to include additional metadata or configuration parameters.
* It is required by the macros in this package to generate the corresponding URDF and SRDF descriptions.
* As for the format defined in this package, each top-level key in the `yaml` file represents a component (e.g., `ft_sensor_mounting`, `force_torque_sensor`, `tool_mounting`, `tool`), and each component can have its own set of parameters.
  However, keys added by external packages may not follow this structure and can include additional metadata or configuration parameters.
* The file must be written according to the expected structure, as described in the [How to write end-effector configuration files](#how-to-write-end-effector-configuration-files) section at the end of this document.

The core features of this package are provided by the [`build_end_effector`](urdf/build_end_effector.xacro) and [`build_end_effector_srdf`](urdf/build_end_effector_srdf.xacro) macros, which generate the URDF and SRDF descriptions of the end-effector, respectively, using the information from the `yaml` file.

## Package contents

* The [`urdf`](urdf/) folder contains:
    * [`build_end_effector_srdf.xacro`](urdf/build_end_effector_srdf.xacro): the macro that generates the SRDF description of the end-effector;
    * [`build_end_effector.xacro`](urdf/build_end_effector.xacro): the macro that generates the URDF description of the end-effector;
    * [`generate_disable_collision_tags.xacro`](urdf/generate_disable_collision_tags.xacro): a helper macro that generates the tags to disable the collision between the end-effector and the robot;

## Using the `build_end_effector` macro

The [`build_end_effector`](urdf/build_end_effector.xacro) macro generates the URDF description of the end-effector from the following parameters:

* `ee_mounting_link`: the name of the link to which the end-effector is mounted.
  It should be the name of the robot flange link;
* `ee_config_file`: the path to the `yaml` file that describes the components of the end-effector.
  Please refer to the [How to write end-effector configuration files](#how-to-write-end-effector-configuration-files) section for more information on how to write this file;
* `enable_ft_sensing`: a boolean flag that controls the inclusion of a force/torque sensor in the end-effector. The default value is `false`. When set to `true`:
    * If a force/torque sensor is described in the `yaml` file:
        * If `sim_ignition` is `false`, the `ros2_control` description of the selected force/torque sensor will be included in the end-effector URDF.
        * If `sim_ignition` is `true`, the `gz_ros2_control/GazeboFTSensor` plugin (provided by the [gazebo_force_torque_sensor package](../../gazebo_force_torque_sensor/README.md)) will be loaded instead.
    * If no force/torque sensor is described in the `yaml` file, an error will be raised.
* `sim_ignition`: a boolean flag that, if true, enables the simulation of the force/torque sensor in Ignition Gazebo and loads the `gz_ros2_control/GazeboFTSensor` plugin.
  The default value is `false`.

## Using the `build_end_effector_srdf` macro

The [`build_end_effector_srdf`](urdf/build_end_effector_srdf.xacro) macro generates the SRDF description of the end-effector from the following parameters:

* `ee_mounting_link`: the name of the link to which the end-effector is mounted.
  It should be the name of the robot flange link;
* `ee_config_file`: the path to the `yaml` file that describes the components of the end-effector;
* `moveit_manipulator_group_name`: the name of a MoveIt! planning group that only includes the robot arm links.
  This group must be already defined before calling the `build_end_effector_srdf` macro.
* `moveit_end_effector_group_name`: the name of a MoveIt! planning group that will only include the end-effector links.
  This group will be created by the `build_end_effector_srdf` macro.
  The default value is `end_effector`;
* `moveit_end_effector_name`: the name of the end-effector.
  This name will be used to define the end-effector in the MoveIt! configuration.
  The default value is `end_effector`;
* `moveit_end_effector_parent_link`: the name of the link that is the parent of the end-effector.
  This link should be the parent link of `ee_mounting_link` in the robot URDF;
* `ignore_ee_collision_links`: a list of manipulator links whose collision with the end-effector should be ignored.
  This list is used when generating the collision matrix for the end-effector.
  The default value is an empty list.

  **Note:** Selecting this list can be challenging for users, as it depends on the specific geometry of the end-effector and the joint position limits of the robot.
  For example, collisions with the adjacent link are typically safe to omit, but there may be other links in the kinematic chain for which collisions are either impossible or very unlikely, and these are not always obvious to identify in advance.
  If you want to determine this list accurately, it is recommended to use a tool similar to the MoveIt Setup Assistant, running it on the URDF generated by the `build_end_effector` macro.
  This allows for interactive and visual selection of allowed collisions based on the actual robot and end-effector configuration.

## Using the `generate_disable_collision_tags` macro

The [`generate_disable_collision_tags`](urdf/generate_disable_collision_tags.xacro) macro is a helper macro that generates the necessary `<disable_collisions>` tags to disable collisions between a given link and a list of other links, for a specified reason.

It takes the following parameters:

* `first_link`: the name of the first link (typically the end-effector link).
* `second_link_list`: a list of link names (the other links to disable collisions with).
* `reason`: (optional) a string specifying the reason for disabling the collision. Defaults to `Never`.

For each link in `second_link_list`, the macro generates a `<disable_collisions>` tag between `first_link` and that link, with the specified reason.

All collisions in a single call are disabled for the same reason.
If you want to disable collisions for different reasons (e.g., links 1,2,3 for reason A and links 4,5,6 for reason B), you must call the macro multiple times, each time with the appropriate subset of links and reason.

This macro is typically used in the [`build_end_effector_srdf` macro](urdf/build_end_effector_srdf.xacro) to ensure that the end-effector does not collide with the robot links during simulation.

## How to write end-effector configuration files

The end-effector configuration file is a `yaml` file that describes the components of the end-effector.
An example end-effector configuration file is the following:

```yaml
ft_sensor_mounting:
  description_package: "acg_resources_generic_mounting_description"
  description_file: "urdf/generic_mounting.urdf.xacro"
  description_params:
    height: 0.01
    radius: 0.05
    mass: 0.05

force_torque_sensor:
  description_package: "acg_resources_generic_force_torque_sensor_description"
  description_file: "urdf/generic_force_torque_sensor.urdf.xacro"
  description_params:
    height: 0.04
    radius: 0.05
    mass: 0.440
  ros2_control_package: "end_effector_builder"
  ros2_control_file: "urdf/gazebo_force_torque_sensor.ros2_control.xacro"
  ros2_control_params:
    number_of_samples_for_computing_bias: 10
  sensed_joint_name: "ft_sensor_measure_joint"
  sensing_frame_name: "ft_sensing_frame"
  gazebo_params:
    topic: "force_torque"
    update_rate: 100

tool_mounting:
  description_package: "acg_resources_generic_mounting_description"
  description_file: "urdf/generic_mounting.urdf.xacro"
  description_params:
    height: 0.01
    radius: 0.025
    mass: 0.0

tool:
  description_package: "acg_resources_long_handle_description"
  description_file: "urdf/long_handle.urdf.xacro"
```

Each top-level key in the `yaml` file represents a component of the end-effector and is optional.

* The `ft_sensor_mounting` key describes the mounting of the f/t sensor to the robot flange.
  It must contain the following mandatory parameters:
    * `description_package`: the package that contains the URDF description of the mounting;
    * `description_file`: the path to the URDF description of the mounting.
      In this file, the `build_mounting` macro must be defined as explained in the [mountings documentation](../mountings/README.md#how-to-extend-the-mountings-library).
    * `description_params`: a dictionary of parameters that are passed to the URDF description of the mounting.
      This parameter is mandatory only if the mounting description file has some mandatory parameters.
* The `force_torque_sensor` key describes the force/torque sensor mounted on the end-effector.
  It contains the following parameters:
    * `description_package`: the package that contains the URDF description of the force/torque sensor.
      This parameter is mandatory;
    * `description_file`: the path to the URDF description of the force/torque sensor.
      In this file, the `build_force_torque_sensor` macro must be defined as explained in the [force/torque sensors documentation](../force_torque_sensors/README.md#how-to-extend-the-forcetorque-sensors-library).
      This parameter is mandatory;
    * `description_params`: a dictionary of parameters that are passed to the URDF description of the force/torque sensor.
      This parameter is mandatory only if the force/torque sensor description file has some mandatory parameters;
    * `ros2_control_package`: the package that contains the `ros2_control` description of the force/torque sensor.
      This parameter is mandatory if the `build_end_effector` macro is called with `enable_ft_sensing` set to `true` and the `sim_ignition` parameter is set to `false`;
    * `ros2_control_file`: the path to the `ros2_control` description of the force/torque sensor.
      In this file, the `build_ros2_control_description` macro must be defined.
      This parameter is mandatory if the `build_end_effector` macro is called with `enable_ft_sensing` set to `true` and the `sim_ignition` parameter is set to `false`.
    * `ros2_control_params`: a dictionary of parameters that are passed to the `ros2_control_file`.
      This parameter is mandatory only if the `ros2_control_file` has some mandatory parameters;
    * `sensed_joint_name`: the name of the joint which specifies the F/T measurements' reference frame.
      This parameter is only used if `enable_ft_sensing` is set to `true`.
      By default, it is set to `ft_sensor_measure_joint`;
    * `sensing_frame_name`: the name of the frame in which the force/torque sensor measurements are provided.
      This parameter is only used if `enable_ft_sensing` is set to `true`.
      By default, it is set to `ft_sensing_frame`;
    * `gazebo_params`: a dictionary of parameters that are passed to the Gazebo force/torque sensor plugin.
      This parameter is optional and is used only if the `build_end_effector` macro is called with the `enable_ft_sensing` and `sim_ignition` parameters set to `true`.
      It can contain the following optional parameters:
        * `topic`: the topic to which the force/torque sensor plugin publishes its data.
          The default value is `force_torque`;
        * `update_rate`: the rate at which the force/torque sensor plugin publishes its data.
          The default value is `100` Hz.
* The `tool_mounting` key describes the mechanical mounting of the tool to the end-effector.
  It must contain the following mandatory parameters:
    * `description_package`: the package that contains the URDF description of the tool mounting;
    * `description_file`: the path to the URDF description of the tool mounting.
      In this file, the `build_mounting` macro must be defined as explained in the [mountings documentation](../mountings/README.md#how-to-extend-the-mountings-library).
    * `description_params`: a dictionary of parameters that are passed to the URDF description of the tool mounting.
      This parameter is mandatory only if the tool mounting description file has some mandatory parameters.
* The `tool` key describes the tool mounted on the end-effector.
  It must contain the following mandatory parameters:
    * `description_package`: the package that contains the URDF description of the tool;
    * `description_file`: the path to the URDF description of the tool.
      In this file, the `build_tool` macro must be defined as explained in the [tools documentation](../tools/README.md#how-to-extend-the-tools-library).

The `yaml` file can also contain additional top-level keys that are not directly related to the end-effector building process.
These keys can be used to store additional information about the end-effector, such as metadata or configuration parameters that are not directly used by the macros in this package.
Please refer to the specific package documentation for more information on how to use these additional keys.
For an example on how to extend the end-effector configuration file with additional keys, see the [acg_resources_ur10_moveit_config package documentation](../../acg_resources_ur10_moveit_config/README.md#end-effector-configuration).
