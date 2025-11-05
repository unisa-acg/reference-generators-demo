# acg_resources_robotiq_ft150_description

This package contains description files for the [Robotiq FT150 force-torque sensor](https://assets.robotiq.com/website-assets/support_archives/document_en/FT_Sensor_Instruction_Manual_2016_11_18_PDF_20201105.pdf).

## Visualization Demo

This package includes a launch file for visualizing the Robotiq FT150 force-torque sensor.

To launch the visualization in RViz, use:

```bash
ros2 launch acg_resources_robotiq_ft150_description display.launch.py
```

## Configuration Parameters

The following configuration parameters can be passed via the `config` parameter (a dictionary) to the `build_force_torque_sensor` macro when including the sensor in an end-effector:

* `rpy`: the RPY orientation of the sensor with respect to the mounting frame.
  This parameter is optional and defaults to `"0, 0, 0"`.
