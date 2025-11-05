# mountings

This is a container for mechanical mountings that can be used to attach a force/torque sensor or a tool to a mechanical interface.

## How to extend the mountings library

This package contains a generic mounting description file, [`generic_mounting.urdf.xacro`](urdf/generic_mounting.urdf.xacro), that can be used as a template to create new mountings.

To extend the mounting library with a new mounting, you should create a new description package in this folder, or add a new mounting to an existing package.

Each mounting is represented by a `.urdf.xacro` file in the `urdf` folder of the package.
The `.urdf.xacro` file must define a xacro macro named `build_mounting` that generates the URDF description of the mounting from the following parameters:

* `mounting_link`: the name of the link to which the mounting is attached;
* `name`: the name of the mounting link.
  This name should be unique within the robot description.
  The default value of this parameter is `mounting`.
* `config`: a dictionary containing the configuration parameters of the mounting provided by the user.
  The provided configuration parameters should be used to generate the URDF description of the mounting.
  A configuration parameter can be either mandatory or optional (i.e. it has a default value).
  To read the value of a configuration parameter, the [`define_property_from_dict`](../../acg_common_libraries/xacro/define_property_from_dict.xacro) macro defined in the [`acg_common_libraries` package](../../acg_common_libraries/xacro/define_property_from_dict.xacro) can be used.
  For more information on how to use this macro, please refer to the [`acg_common_libraries` package README](../../acg_common_libraries/README.md#using-the-define_property_from_dict-macro) or any of the existing mountings in this library.
* `sim_ignition`: a boolean that indicates whether the mounting should be simulated in Ignition Gazebo.
  If `true`, gazebo-specific tags should be added to the URDF description of the mounting.
  This parameter should be captured by the parent xacro namespace with the `^` operator and have a default value of `false`.
  For more information on the `^` operator, refer to the official [xacro wiki](https://github.com/ros/xacro/wiki#default-parameter-values).
  For an example of how to use this parameter, refer to any of the existing mountings in this library.

**Naming convention:** The tip link of the mounting (i.e., the link to which the next component, such as a force/torque sensor or tool, will be attached) should be named `mounting` for compatibility with the rest of the end-effector builder system.
