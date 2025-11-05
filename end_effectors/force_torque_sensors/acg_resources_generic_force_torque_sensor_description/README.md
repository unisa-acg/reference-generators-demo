# acg_resources_generic_force_torque_sensor_description

This package contains description files for a generic force-torque sensor.
The sensor is modeled as a cylinder with configurable dimensions and mass.

## Visualization Demo

This package includes a launch file for visualizing the generic force-torque sensor in RViz with customizable parameters.

To launch the visualization in RViz, use:

```bash
ros2 launch acg_resources_generic_force_torque_sensor_description display.launch.py height:=0.05 radius:=0.1
```

You can modify the `height` and `radius` parameters to adjust the sensor's dimensions in the visualization.

## Configuration Parameters

The following configuration parameters can be passed via the `config` parameter (a dictionary) to the `build_force_torque_sensor` macro when including the sensor in an end-effector:

* `height`: the height of the sensor.
  This parameter is mandatory;
* `mass`: the mass of the sensor.
  This parameter is mandatory;
* `radius`: the radius of the sensor.
  This parameter is mandatory;
* `rpy`: the RPY orientation of the sensor with respect to the mounting frame.
  This parameter is optional and defaults to `"[0.0, 0.0, 0.0]"`.
* `sensing_frame_position`: the position of the sensing frame with respect to the sensor's origin.
  This parameter is optional and defaults to `"[0.0, 0.0, 0.0]"`.

An example configuration for the force/torque sensor in the context of an [end-effector configuration file](../../end_effector_builder/README.md#how-to-write-end-effector-configuration-files) is the following:

```yaml
force_torque_sensor:
  description_params:
    height: 0.05
    mass: 0.1
    radius: 0.1
    rpy:
      - 0.0
      - 0.0
      - 0.0
    sensing_frame_position:
      - 0.0
      - 0.0
      - 0.0
```

## Emulating a "Fake" Force/Torque Sensor

To emulate a force/torque sensor with no physical encumbrance, you can configure the sensor as follows:

```yaml
mass: 0.00001
height: 0.0
radius: 0.0
```

This will create a sensor with negligible mass and null dimensions, effectively making it "invisible" in the simulation.
Such a sensor can be useful to emulate robots with integrated F/T sensing capabilities.
