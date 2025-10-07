# Reference Generators and Repository Overview

This repository provides reference generators components and demos for ROS2 Humble.
Reference generators allow to decouple reference management from control law computation, enabling more flexible, modular, and reusable control systems.
They can handle both online references (received in real time via topics or action servers) and trajectories, and may include features such as interpolation or trajectory execution logic.

For more details about the design, configuration, and usage of reference generators, please refer to the dedicated documentation in the [`reference_generator` package](./reference_generator/README.md).

## What this repository contains

This repository contains a joint space reference generator and a task space reference generator, along with their configurations and example usage.
In particular:

- [`reference_generator`](./reference_generator/README.md): contains all reference generator components.
- [`acg_common_libraries`](./acg_common_libraries/README.md): contains common libraries used by the reference generators and controllers.
- [`acg_common_msgs`](./acg_common_msgs/README.md): contains custom messages used by the reference generators and controllers.
- [`acg_controller_interface`](./acg_controller_interface/README.md): contains custom semantic components and semantic command components used by the reference generators.
- [`acg_hardware_interface_facade`](./acg_hardware_interface_facade/README.md): contains a series of components that facilitate the interaction with the hardware interface,which are used by both controllers and reference generators.
- [`cartesian_pose_controller`](./cartesian_pose_controller/README.md): contains a task space controller, that will be used in the demos.
- [`ur10_reference_generators_demo`](./ur10_reference_generators_demo/README.md): contains a demo that shows how to use both reference generators with a UR10 robot.

## Dependencies

This code requires the installation of Ubuntu 22.04 and [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html).

### System dependencies

To install the packages needed by this repo as system-wide dependencies, run the following command from the repository's root folder:

```bash
rosdep install --from-paths . -i
```

&copy; *2025 Automatic Control Group (DIEM, University of Salerno)*
