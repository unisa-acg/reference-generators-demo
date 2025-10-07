# acg_common_libraries

This package contains common libraries and utility functions developed by Automatic Control Group for the ROS2 ecosystem.
Refer to the header file for more documentation of the provided functions.

## Package contents

This package contains the following directories:

* [`include`](include/): contains the header files for the common C++ libraries;
* [`src`](src/): contains the source files for the common C++ libraries;
* [`test`](test/): contains the test files for the common C++ libraries;
* [`xacro`](xacro/): contains the common xacro macros.

## Using the `define_property_from_dict` macro

The [`define_property_from_dict`](xacro/define_property_from_dict.xacro) macro is a helper macro that defines a xacro property from a dictionary.
It takes the following parameters:

* `input_dict`: the dictionary from which to extract the property;
* `name`: the name of the property to define;
* `default_value`: the default value of the property.
  By default, it is set to `None`.

If the `input_dict` is defined and contains the `name` key, the property will be set to the value of the `name` key in the `input_dict`.
If the `input_dict` is not defined or does not contain the `name` key, the property will be set to `default_value` if it is not `None`.
In the latter case, if the `default_value` is `None`, an error will be raised.

## How to test

If you want to perform the tests contained in this package after the build, you must launch the following command:

```bash
colcon test --packages-select acg_common_libraries
```

If the terminal does not show the test details, you should run the following command:

```bash
colcon test-result --all --verbose
```

## Optional analysis

If you wish to see the INFO messages printed on console during the test, run the following:

```bash
colcon test --packages-select acg_common_libraries --event-handlers console_cohesion+
```

The expected output should contain the following line:

```text
100% tests passed, 0 tests failed out of 1
```
