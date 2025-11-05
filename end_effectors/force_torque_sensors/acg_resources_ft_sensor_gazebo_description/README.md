# acg_resources_ft_sensor_gazebo_description

This package collects resources needed for simulating a force/torque sensor in the Gazebo simulator.

## Package contents

* The [`urdf`](urdf/) folder contains:
    * [`gazebo_ft_sensor_description.xacro`](urdf/gazebo_ft_sensor_description.xacro): defines a `xacro` macro that generates the URDF tags required by Gazebo to simulate a force/torque sensor.
    This file is used by the `gazebo_ft_sensor_description.launch.py` launch file.
    * [`gazebo_force_torque_sensor.ros2_control.xacro`](urdf/gazebo_force_torque_sensor.ros2_control.xacro): defines a `xacro` macro that generates the `ros2_control` tags for describing a force/torque sensor in the Gazebo simulator.
* The [`world`](world/) folder contains:
    * [`world_with_ft_sensor.sdf`](world/world_with_ft_sensor.sdf): an empty world loading the F/T sensor plugin for Gazebo. This world corresponds to the empty world of Gazebo with the addition of F/T sensor plugin.
