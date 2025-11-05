# acg_resources_short_handle_description

This package contains description files a short end-effector handle, which is used to make a manipulator interact with objects in the environment.

## Visualization demos

This package provides a launch file to visualize the handle in RViz. To do so, run the following command:

```bash
ros2 launch acg_resources_short_handle_description display.launch.py
```

## Xacro Macro Parameters

The main macro in this package, [`build_tool`](urdf/short_handle.urdf.xacro), accepts several parameters.
For an explanation of these parameters, please refer to the [tools container README](../README.md#how-to-extend-the-tools-library).

The `config` parameter is a dictionary that can be used to pass optional or necessary configuration keys to the macro.
For the handles in this package, the following key is currently supported:

* `rpy`: The RPY orientation (in radians) of the handle with respect to the mounting frame.
    * **Type:** string (e.g., `"0 0 0"`)
    * **Default:** `"0 0 0"`
    * **Usage:** If not specified, the default orientation is used.

## Origin of the meshes

The visual and collision meshes used in this package have been produced via SketchUp and later modified in Blender.
