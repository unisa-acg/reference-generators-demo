# acg_resources_generic_mounting_description

This package contains description file for a generic mounting.

## Visualization demo

This package provides a launch file to visualize the generic mounting in RViz with adjustable parameters.

To visualize the generic mounting in RViz, run the following command:

```bash
ros2 launch acg_resources_generic_mounting_description display.launch.py height:=0.05 radius:=0.1
```

The `height` and `radius` parameters can be adjusted to change the size of the mounting.

## Xacro Macro Parameters

The main macro in this package, [`build_mounting`](urdf/generic_mounting.urdf.xacro), accepts several parameters, including a `config` dictionary.
For a general explanation of these parameters, please refer to the [mounting container README](../README.md#how-to-extend-the-mounting-library).

The `config` parameter is a dictionary that can be used to pass optional or necessary configuration keys to the macro.
For the generic mounting, the following keys are currently supported:

* **height**:
    * **Description:** Height of the mounting.
    * **Type:** float
    * **Required:** Yes
* **mass**:
    * **Description:** Mass of the mounting.
    * **Type:** float
    * **Required:** Yes
* **radius**:
    * **Description:** Radius of the mounting.
    * **Type:** float
    * **Required:** Yes
* **rpy**:
    * **Description:** RPY orientation (in radians) of the mounting with respect to the parent frame.
    * **Type:** string (e.g., `"0 0 0"`)
    * **Required:** No
    * **Default:** `"0 0 0"`
