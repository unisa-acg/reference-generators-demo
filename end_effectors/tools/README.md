# tools

This is a container for tool description packages that can be part of an end-effector.

## How to extend the tools library

To extend the tools library with a new tool, the description package which will contain the new tool should be identified.
This could be done by creating a new package in this folder, or by adding a new tool to an existing package.

Each tool is represented by a `.urdf.xacro` file in the `urdf` folder of the package.
The xacro file must define a property `name` in the outermost xacro namespace, which should be set to the name of the tool.
The user must also define a xacro macro named `build_tool` that generates the URDF description of the tool from the following parameters:

* `mounting_link`: the name of the link to which the tool is mounted;
* `mounting_to_tool_joint`: the name of the joint that connects the tool to the parent link;
* `config`: a dictionary containing the configuration parameters of the tool provided by the user.
  The provided configuration parameters should be used to generate the URDF description of the tool.
  A configuration parameter can be either mandatory or optional (i.e. it has a default value).
  To read the value of a configuration parameter, the [`define_property_from_dict`](../../acg_common_libraries/xacro/define_property_from_dict.xacro) macro should be used.
  For more information on how to use this macro, please refer to the [`acg_common_libraries` package README](../../acg_common_libraries/README.md#using-the-define_property_from_dict-macro) or any of the existing mountings in this library.
* `sim_ignition`: a boolean that indicates whether the tool should be simulated in Ignition Gazebo.
  If `true`, gazebo-specific tags should be added to the URDF description of the tool.
  This parameter should be captured by the parent xacro namespace with the `^` operator and have a default value of `false`.
  For more information on the `^` operator, refer to the official [xacro wiki](https://github.com/ros/xacro/wiki#default-parameter-values).
  For an example of how to use this parameter, refer to any of the existing tools in this library.
