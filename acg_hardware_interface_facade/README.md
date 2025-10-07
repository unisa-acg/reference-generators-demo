# acg_hardware_interface_facade

This package provides classes commonly used within ROS2 controllers and chainable controllers.
These classes simplify the configuration and interaction with the robot’s state and command interfaces.
The key classes included are:

- [**`StateReader`**](./include/acg_hardware_interface_facade/state_reader.hpp): This class simplifies the process of reading the robot's state in a controller.
    It provides an easy interface for configuring state interface names and retrieving state values after configuration.

- [**`CommandWriter`**](./include/acg_hardware_interface_facade/command_writer.hpp): This class simplifies the process of writing commands to the robot or another controller, if the latter is chainable.
    It provides an easy interface for configuring command interface names and writing command values once the object is configured.

- [**`ReferenceReader`**](./include/acg_hardware_interface_facade/reference_reader.hpp): This class simplifies the process of reading references received by the controller from a higher-level controller in chainable mode.
    It provides an easy interface for configuring reference interface names and retrieving reference values after configuration.

- [**`ForceTorqueSensorReader`**](./include/acg_hardware_interface_facade/force_torque_sensor_reader.hpp): This class simplifies the process of reading force/torque measurements in a controller.
    It provides an easy interface for configuring state interface names and retrieving state values after configuration.

## Usage

To use any of these classes, the user must first configure them.
For example, the user should call the methods `configure_state_interfaces` or `configure_command_interfaces` within the controller’s `on_init` method.
Afterward, users must assign the appropriate interfaces, such as `LoanedStateInterface`, `LoanedCommandInterface`, or `CommandInterface`, by invoking methods like `assign_loaned_state_interfaces`, `assign_loaned_command_interfaces`, or `assign_command_interfaces`, typically in the `on_activate` method.

Once the configuration and assignment steps are complete, users can retrieve the names of the configured interfaces using the `available_interfaces` method.
The user can then read or write state or command values using the respective `read` or `write` methods provided by the classes.

## Technical details

For more detailed technical information about the implementation, including design decisions and internal architecture, please refer to the [`reference_reader_implementation_notes.md`](./doc/reference_reader_implementation_notes.md) file.

## How to test

If you want to perform the tests contained in this package after the build, you must launch the following command:

```bash
colcon test --packages-select acg_hardware_interface_facade
```

If the terminal does not show the test details, you should run the following command:

```bash
colcon test-result --all --verbose
```

## Optional analysis

If you wish to see the INFO messages printed on console during the test, run the following:

```bash
colcon test --packages-select acg_hardware_interface_facade --event-handlers console_cohesion+
```

The following warnings and errors are expected, because tests cover edge cases where the objects are not configured:

```bash
1: [ERROR] [1744374015.826500838] [state_reader]: State reader not configured. Please configure the object before using this class.
1: [ERROR] [1744374015.826521310] [state_reader]: State reader not configured. Please configure the object before using this class.
...
1: [ERROR] [1744374015.826542049] [command_interface_handler]: Command interface handler not configured. Please configure the object before using it.
...
1: [ERROR] [1744374015.826551152] [command_interface_handler]: Command interface handler not configured. Please configure the object before using it.
1: [ERROR] [1744374015.826553840] [command_interface_handler]: Reference reader not configured. Please configure the object before using it.
1: [ERROR] [1744374015.826555804] [command_interface_handler]: Reference reader not configured. Please configure the object before using it.
1: [ERROR] [1744374015.826558707] [command_interface_handler]: Reference reader not configured. Please configure the object before using it.
...
1: [ERROR] [1750412635.483175914] [force_torque_sensor_reader]: Force/Torque sensor reader not configured. Please configure the object before using this class.
...
1: [ERROR] [1750412635.483200044] [force_torque_sensor_reader]: Force/Torque sensor reader not configured. Please configure the object before using this class.

```
