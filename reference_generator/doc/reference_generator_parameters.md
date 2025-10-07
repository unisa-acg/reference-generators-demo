# Reference Generator Parameters

Default Config
```yaml
reference_generator:
  ros__parameters:
    joints: ''
    kinematics:
      plugin_name: ''
      plugin_package: ''
      tip: ''
    publish_task_space_reference:
      enable: true
      frequency: 5.0
    robot_name: ''
    state_interfaces: '{position}'
    state_interfaces_names_override:
      acceleration: '{}'
      effort: '{}'
      position: '{}'
      velocity: '{}'
    task_space_reference_frame: ''
    wrench_reference_frame: ''

```

## joints

Specifies which joints will be used by the controller. Note that the joints specified in this array must match the number and the order of the references received by the controller.

* Type: `string_array`
* Read only: True

## state_interfaces

Specifies the state interfaces that the controller will read from. Position is always required.

* Type: `string_array`
* Default Value: {"position"}
* Read only: True

*Constraints:*
 - contains no duplicates
 - parameter is not empty
 - every element is one of the list ['position', 'velocity', 'acceleration', 'effort']

*Additional Constraints:*



## robot_name

Specifies the name of the robot. This name is used to construct the names of the state interfaces.

* Type: `string`

## state_interfaces_names_override.position

If this parameter is set, it will override the default names for state interfaces, which are generated as described in the README.md file, specifically for the position interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## state_interfaces_names_override.velocity

If this parameter is set, it will override the default names for state interfaces, which are generated as described in the README.md file, specifically for the velocity interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## state_interfaces_names_override.acceleration

If this parameter is set, it will override the default names for state interfaces, which are generated as described in the README.md file, specifically for the acceleration interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## state_interfaces_names_override.effort

If this parameter is set, it will override the default names for state interfaces, which are generated as described in the README.md file, specifically for the effort interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## kinematics.plugin_name

Specifies the name of the kinematics plugin to load.

* Type: `string`

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## kinematics.plugin_package

Specifies the package name that contains the kinematics plugin.

* Type: `string`

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## kinematics.tip

Specifies the end effector link of the robot description used by the kinematics plugin.

* Type: `string`

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## task_space_reference_frame

Specifies the frame with respect to which task space references are output.

* Type: `string`

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## wrench_reference_frame

Specifies the wrench frame with respect to which wrench references are output.

* Type: `string`

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## publish_task_space_reference.enable

Specifies whether the task space reference of the end effector should be published.

* Type: `bool`
* Default Value: true

## publish_task_space_reference.frequency

Specifies the maximum frequency at which the task space reference tries to publish [Hz].

* Type: `int`
* Default Value: 5
* Read only: True

*Constraints:*
 - greater than 0

*Additional Constraints:*



