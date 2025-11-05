# filters

This is a container for extensions of the `filters` package, which provides filters based on [pluginlib](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html).
For further information about the `filters` package and other filter implementations, please refer to [this page](https://github.com/ros/filters/tree/ros2).

## How to configure

Filters' configuration are retrieved from the node parameters interface.
The configuration process is carried out by the `configure()` method of the [base class](https://github.com/ros/filters/blob/ros2/include/filters/filter_base.hpp), which expects the desired namespace configuration.

Under the filter namespace, the filter expects two parameters:

- `type`: the filter's typename as declared in its `pluginlib` registration;
- `params`: filter-specific parameters, usually different for each filter.

Filter parameters can be stored by either declaring them directly as node parameters or by saving them in a `.yaml` file.

For single-channel samples, a class derived from `filters::FilterBase` should be utilized, on which the `configure()` method has to be called with the following parameters:

- `param_prefix`: the prefix of the filter's parameters;
- `filter_name`: a unique name for the filter;
- `node_logger`: the `NodeLoggingInterface` instance;
- `node_params`: the `NodeParametersInterface` instance of the node where the parameters are stored;

Below a template of the method to call to configure the filter:

```c++
filter->configure("param_prefix",
                  "filter_name",
                  node_logger,
                  node_params);
```

For samples containing multiple channels, a class derived from `filters::MultiChannelFilterBase` should be used instead.
In this case, the `configure()` method expects also the number of channels, i.e.:

```c++
filter->configure(number_of_channels,
                  "param_prefix",
                  "filter_name",
                  node_logger,
                  node_params);
```

## How to use

After configured, the `update()` method receives a new sample input and retrieves the filtered value.

## Filter chain

The filter classes can be used in a chain consisting of several filters.

All filters in the chain are configured with the same number of channels.
Since each filter may have different channel requirements, the total number of channels for the chain should be set to the maximum number of channels required by any filter in the chain.
For instance, when using an [`UnbiasingFilter`](./unbiasing_filter/include/unbiasing_filter/unbiasing_filter.hpp) class, the number of channels in the entire chain must be set to the size of the measurements to be filtered plus 1.
For more details, refer to the `unbiasing_filter` package [documentation](./unbiasing_filter/README.md#how-to-configure).

For further detail on how to configure the chain, see the source file [filter_chain.hpp](https://github.com/ros/filters/blob/ros2/include/filters/filter_chain.hpp).

## Known issues

When building packages in the container, the following warning might show up in the console:

```text
[0.311s] WARNING:colcon.colcon_core.package_selection:Some selected packages are already built in one or more underlay workspaces:
    'filters' is in: /opt/ros/humble
If a package in a merged underlay workspace is overridden and it installs headers, then all packages in the overlay must sort their include directories by workspace order. Failure to do so may result in build failures or undefined behavior at run time.
If the overridden package is used by another package in any underlay, then the overriding package in the overlay must be API and ABI compatible or undefined behavior at run time may occur.

If you understand the risks and want to override a package anyways, add the following to the command line:
    --allow-overriding filters

This may be promoted to an error in a future release of colcon-override-check.
```

This happens when `filters` is a system dependency of your system.
As shown in our [.rosinstall file](../../reference_generators_demo.rosinstall), we prefer using it as a source dependency, because of some bugs in configuring filter chains in the version installed via aptitude.
