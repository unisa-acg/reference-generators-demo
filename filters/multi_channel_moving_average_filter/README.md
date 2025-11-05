# multi_channel_moving_average_filter

In this package, the `MultiChannelMovingAverageFilter` class is provided, extending the `filters::MultiChannelFilterBase` base class of the [filters](https://github.com/ros/filters/tree/ros2) ROS2 package.

This package is a multi-channel version of the [`moving_average_filter`](../moving_average_filter/README.md) for standard vectors.

## How to configure

In the case of the `MultiChannelMovingAverageFilter` class, the filter-specific parameters to configure are:

- `number_of_observations`: the size *n* of the moving window;

This parameter must be a node-specific parameter, and you need to pass the node's parameter interface to the configure method of the `MultiChannelFilterBase` class to properly configure the filter.

Examples of configuration are provided in the [test/test_multi_channel_moving_average_filter.cpp](./test/test_multi_channel_moving_average_filter.cpp) file.

## How to build

For importing and using this package, you need to build it first:

```bash
colcon build --packages-up-to multi_channel_moving_average_filter
source install/setup.bash
```

## How to test

If you want to perform the tests contained in this package, after the build, execute:

```bash
colcon test --packages-select multi_channel_moving_average_filter
```

If the terminal does not show the test details, you should run the following command:

```bash
colcon test-result --all --verbose
```

## Optional analysis

If you wish to see the INFO messages printed to the console during the test, run the following:

```bash
colcon test --packages-select multi_channel_moving_average_filter --event-handlers console_cohesion+
```

The expected output should contain the following line:

```text
100% tests passed, 0 tests failed out of 1
```
