# Motion Based Interaction Controller Parameters

Default Config
```yaml
motion_based_interaction_controller:
  ros__parameters:
    command_interfaces: '{pose}'
    command_interfaces_names_override:
      pose: '{}'
      twist: '{}'
    command_tolerance:
      rotational_tolerance: 0.01
      translational_tolerance: 0.01
    filter_chain:
      fixed_orientation:
        filter1:
          name: unbiasing_filter
          params:
            number_of_stable_samples_for_bias: 10.0
            tolerance: 0.5
          type: unbiasing_filter/UnbiasingFilterDouble
        filter2:
          name: multi_channel_moving_average_filter
          params:
            number_of_observations: 15.0
          type: moving_average_filter/MultiChannelMovingAverageFilterDouble
      variable_orientation:
        filter1:
          name: gravity_compensation_filter
          params:
            ft_sensing_frame_name: ft_sensing_frame
            gravity: '{0.0, 0.0, -9.81}'
          type: gravity_compensation_filter/GravityCompensationFilterDouble
        filter2:
          name: unbiasing_filter
          params:
            number_of_stable_samples_for_bias: 50.0
            tolerance: 2.0
          type: unbiasing_filter/UnbiasingFilterDouble
        filter3:
          name: multi_channel_moving_average_filter
          params:
            number_of_observations: 15.0
          type: moving_average_filter/MultiChannelMovingAverageFilterDouble
    fixed_robot_orientation: false
    force_torque_sensor:
      measure_frame: ''
      name: ''
    ft_sensor_state_interfaces_names_override: '{}'
    interaction_filter:
      name: ''
      type: ''
    joints: ''
    kinematics:
      plugin_name: ''
      plugin_package: ''
      tip: ''
    reference_interfaces: '{pose}'
    reference_interfaces_names_override:
      pose: '{}'
      twist: '{}'
      wrench: '{}'
    robot_name: ''
    state_interfaces_names_override:
      position: '{}'
      velocity: '{}'
    task_space_command_controller: ''
    task_space_reference_frame: ''
    wrench_reference_frame: ''

```

## robot_name

Specifies the name of the robot.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## joints

Specifies which joints will be used by the controller.

* Type: `string_array`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## force_torque_sensor.name

Specifies the name of the force/torque sensor providing measurements.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## force_torque_sensor.measure_frame

Specifies the name of the frame in which the force/torque measures are expressed.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## state_interfaces_names_override.position

If this parameter is set, it will override the default names (<robot_name>/<joint_name>/position) for the joint position state interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## state_interfaces_names_override.velocity

If this parameter is set, it will override the default names (<robot_name>/<joint_name>/velocity) for the joint velocity state interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## ft_sensor_state_interfaces_names_override

If this parameter is set, it will override the default names (<sensor_name>/[force/torque].[x/y/z]) for the force/torque sensor state interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces

Specifies the interfaces that will be used to send commands.

* Type: `string_array`
* Default Value: {"pose"}
* Read only: True

*Constraints:*
 - every element is one of the list ['pose', 'twist']
 - contains no duplicates

*Additional Constraints:*



## command_interfaces_names_override.pose

If this parameter is set, it will override the default names (<downstream_controller_name/[position/orientation].[x/y/z/w]) for the pose command interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.twist

If this parameter is set, it will override the default names (<downstream_controller_name/[linear_velocity/angular_velocity].[x/y/z]) for the twist command interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## reference_interfaces

Specifies the interfaces that will be used to receive references.

* Type: `string_array`
* Default Value: {"pose"}
* Read only: True

*Constraints:*
 - every element is one of the list ['pose', 'twist', 'wrench']
 - contains no duplicates

*Additional Constraints:*



## reference_interfaces_names_override.pose

If this parameter is set, it will override the default names (<controller_name>/[position/orientation].[x/y/z/w]) for the pose reference interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## reference_interfaces_names_override.twist

If this parameter is set, it will override the default names (<controller_name/[linear_velocity/angular_velocity]/[x/y/z]) for the twist reference interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## reference_interfaces_names_override.wrench

If this parameter is set, it will override the default names (<controller_name/[force/torque].[x/y/z]) for the wrench reference interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## kinematics.plugin_name

Specifies the name of the kinematics plugin to load.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## kinematics.plugin_package

Specifies the package name that contains the kinematics plugin.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## kinematics.tip

Specifies the end effector link of the robot description used by the kinematics plugin.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## task_space_reference_frame

Specifies the reference frame for the task space.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## interaction_filter.type

Specifies the name of the interaction filter plugin to load.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## interaction_filter.name

Specifies the name of the interaction filter.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## wrench_reference_frame

Specifies the reference frame for the wrench.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## task_space_command_controller

Specifies the controller that will receive the task space commands.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## fixed_robot_orientation

If enabled, the controller will be able to work only with fixed orientation.

* Type: `bool`
* Default Value: false
* Read only: True

## command_tolerance.translational_tolerance

Specifies the translational tolerance for the command in meters.

* Type: `double`
* Default Value: 0.01
* Read only: True

*Constraints:*
 - greater than or equal to 0.0

*Additional Constraints:*



## command_tolerance.rotational_tolerance

Specifies the rotational tolerance for the command in radians.

* Type: `double`
* Default Value: 0.01
* Read only: True

*Constraints:*
 - greater than or equal to 0.0

*Additional Constraints:*



## filter_chain.fixed_orientation.filter1.name

Specifies the name of the first filter in the filter chain for fixed orientation.

* Type: `string`
* Default Value: "unbiasing_filter"
* Read only: True

*Constraints:*
 - one of the specified values: ['unbiasing_filter']

*Additional Constraints:*



## filter_chain.fixed_orientation.filter1.type

Specifies the type of the first filter in the filter chain for fixed orientation.

* Type: `string`
* Default Value: "unbiasing_filter/UnbiasingFilterDouble"
* Read only: True

*Constraints:*
 - one of the specified values: ['unbiasing_filter/UnbiasingFilterDouble']

*Additional Constraints:*



## filter_chain.fixed_orientation.filter1.params.number_of_stable_samples_for_bias

Specifies the number of stable samples required to compute the bias.

* Type: `int`
* Default Value: 10
* Read only: True

*Constraints:*
 - greater than or equal to 0

*Additional Constraints:*



## filter_chain.fixed_orientation.filter1.params.tolerance

Specifies the tolerance to consider a sample as stable.

* Type: `double`
* Default Value: 0.5
* Read only: True

*Constraints:*
 - greater than or equal to 0.0

*Additional Constraints:*



## filter_chain.fixed_orientation.filter2.name

Specifies the name of the second filter in the filter chain for fixed orientation.

* Type: `string`
* Default Value: "multi_channel_moving_average_filter"
* Read only: True

*Constraints:*
 - one of the specified values: ['multi_channel_moving_average_filter']

*Additional Constraints:*



## filter_chain.fixed_orientation.filter2.type

Specifies the type of the second filter in the filter chain for fixed orientation.

* Type: `string`
* Default Value: "moving_average_filter/MultiChannelMovingAverageFilterDouble"
* Read only: True

*Constraints:*
 - one of the specified values: ['moving_average_filter/MultiChannelMovingAverageFilterDouble']

*Additional Constraints:*



## filter_chain.fixed_orientation.filter2.params.number_of_observations

Specifies the number of observations for the moving average filter.

* Type: `int`
* Default Value: 15
* Read only: True

*Constraints:*
 - greater than or equal to 1

*Additional Constraints:*



## filter_chain.variable_orientation.filter1.name

Specifies the name of the first filter in the filter chain for variable orientation.

* Type: `string`
* Default Value: "gravity_compensation_filter"
* Read only: True

*Constraints:*
 - one of the specified values: ['gravity_compensation_filter']

*Additional Constraints:*



## filter_chain.variable_orientation.filter1.type

Specifies the type of the first filter in the filter chain for variable orientation.

* Type: `string`
* Default Value: "gravity_compensation_filter/GravityCompensationFilterDouble"
* Read only: True

*Constraints:*
 - one of the specified values: ['gravity_compensation_filter/GravityCompensationFilterDouble']

*Additional Constraints:*



## filter_chain.variable_orientation.filter1.params.ft_sensing_frame_name

Specifies the name of the frame in which the force/torque measures are expressed.

* Type: `string`
* Default Value: "ft_sensing_frame"
* Read only: True

## filter_chain.variable_orientation.filter1.params.gravity

Specifies the gravity vector.

* Type: `double_array`
* Default Value: {0.0, 0.0, -9.81}
* Read only: True

*Constraints:*
 - length must be equal to 3

*Additional Constraints:*



## filter_chain.variable_orientation.filter2.name

Specifies the name of the second filter in the filter chain for variable orientation.

* Type: `string`
* Default Value: "unbiasing_filter"
* Read only: True

*Constraints:*
 - one of the specified values: ['unbiasing_filter']

*Additional Constraints:*



## filter_chain.variable_orientation.filter2.type

Specifies the type of the second filter in the filter chain for variable orientation.

* Type: `string`
* Default Value: "unbiasing_filter/UnbiasingFilterDouble"
* Read only: True

*Constraints:*
 - one of the specified values: ['unbiasing_filter/UnbiasingFilterDouble']

*Additional Constraints:*



## filter_chain.variable_orientation.filter2.params.number_of_stable_samples_for_bias

Specifies the number of stable samples required to compute the bias.

* Type: `int`
* Default Value: 50
* Read only: True

*Constraints:*
 - greater than or equal to 0

*Additional Constraints:*



## filter_chain.variable_orientation.filter2.params.tolerance

Specifies the tolerance to consider a sample as stable.

* Type: `double`
* Default Value: 2.0
* Read only: True

*Constraints:*
 - greater than or equal to 0.0

*Additional Constraints:*



## filter_chain.variable_orientation.filter3.name

Specifies the name of the third filter in the filter chain for variable orientation.

* Type: `string`
* Default Value: "multi_channel_moving_average_filter"
* Read only: True

*Constraints:*
 - one of the specified values: ['multi_channel_moving_average_filter']

*Additional Constraints:*



## filter_chain.variable_orientation.filter3.type

Specifies the type of the third filter in the filter chain for variable orientation.

* Type: `string`
* Default Value: "moving_average_filter/MultiChannelMovingAverageFilterDouble"
* Read only: True

*Constraints:*
 - one of the specified values: ['moving_average_filter/MultiChannelMovingAverageFilterDouble']

*Additional Constraints:*



## filter_chain.variable_orientation.filter3.params.number_of_observations

Specifies the number of observations for the moving average filter.

* Type: `int`
* Default Value: 15
* Read only: True

*Constraints:*
 - greater than or equal to 1

*Additional Constraints:*



