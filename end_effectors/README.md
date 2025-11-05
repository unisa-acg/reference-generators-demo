# end_effectors

This is a container for end-effector related packages.
An end-effector is a device connected to a robot manipulator's flange that serves a specific purpose.
As end-effectors enable the robot to interact with the environment, it is often useful to integrate a force/torque sensor between the interaction tool and the robot flange to measure the forces and torques to which the robot is subjected as it interacts with the environment.
Finally, because end-effectors are often custom-made for a specific purpose, it is often necessary to integrate custom mechanical mountings to interface the various components to one another and to the robot flange.

In the scope of this container, an end-effector can be thought of as a combination of one or more of the following components, in the following order:

* a mechanical mounting that acts as an interface between the robot flange and the force/torque sensor;
* a force/torque sensor that measures the forces and torques to which the robot is subjected;
* a mechanical mounting that acts as an interface between the force/torque sensor or the robot flange and the tool;
* a tool that serves a specific purpose, such as a gripper, a vacuum cup, or a welding torch.

## Contents

In this container, the following content is provided:

* [`end_effector_builder`](end_effector_builder/): a package that provides a system to build end-effector descriptions (URDFs) and the associated semantic descriptions (SRDFs) from a `yaml` file that describes the components of the end-effector.
  For more information on how to write end-effector configuration files, see the [related section in the package documentation](end_effector_builder/README.md#how-to-write-end-effector-configuration-files).
* [`mountings`](mountings/): this container includes description packages for mechanical mountings that can be used to attach a force/torque sensor or a tool to a mechanical interface.
  For more information on how to extend this library, see the [related section in the sub-container documentation](mountings/README.md#how-to-extend-the-mountings-library).
* [`force_torque_sensors`](force_torque_sensors/): this container includes description packages for force/torque sensors that can be part of an end-effector.
  For more information on how to extend this library, see the [related section in the sub-container documentation](force_torque_sensors/README.md#how-to-extend-the-forcetorque-sensors-library).
* [`tools`](tools/): this container includes description packages for tools that can be part of an end-effector.
  For more information on how to extend this library, see the [related section in the sub-container documentation](tools/README.md#how-to-extend-the-tools-library).
