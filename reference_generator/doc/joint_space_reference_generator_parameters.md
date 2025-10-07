# Joint Space Reference Generator Parameters

Default Config
```yaml
joint_space_reference_generator:
  ros__parameters:
    command_interfaces_names_override:
      joint_acceleration: '{}'
      joint_effort: '{}'
      joint_position: '{}'
      joint_velocity: '{}'
      joint_wrench: '{}'
      task_space_acceleration: '{}'
      task_space_pose: '{}'
      task_space_velocity: '{}'
      task_space_wrench: '{}'
      task_space_wrench_derivative: '{}'
    joint_space_command_controller: ''
    joint_space_command_interfaces: '{}'
    joints: ''
    task_space_command_controller: ''
    task_space_command_interfaces: '{}'

```

## joints

Specifies which joints will be used by the controller.

* Type: `string_array`
* Read only: True

## joint_space_command_controller

Name of the controller that receives joint space commands and/or wrench commands. If no joint space controller is used, leave joint_space_command_interfaces empty and this parameter will be ignored.

* Type: `string`
* Default Value: ""
* Read only: True

## joint_space_command_interfaces

List of command interfaces used to send joint space commands. If no joint space controller is used, leave this list empty.

* Type: `string_array`
* Default Value: {}
* Read only: True

*Constraints:*
 - contains no duplicates
 - every element is one of the list ['position', 'velocity', 'acceleration', 'effort', 'wrench']

*Additional Constraints:*



## task_space_command_controller

Name of the controller that receives task space commands. If no task space controller is used, leave task_space_command_interfaces empty and this parameter will be ignored.

* Type: `string`
* Default Value: ""
* Read only: True

## task_space_command_interfaces

List of command interfaces used to send task space commands. If no task space controller is used, leave this list empty.

* Type: `string_array`
* Default Value: {}
* Read only: True

*Constraints:*
 - contains no duplicates
 - every element is one of the list ['pose', 'twist', 'acceleration', 'wrench', 'wrench_derivative']

*Additional Constraints:*



## command_interfaces_names_override.joint_position

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the joint position interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.joint_velocity

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the joint velocity interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.joint_acceleration

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the joint acceleration interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.joint_effort

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the joint effort interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.joint_wrench

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the joint wrench interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.task_space_pose

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the task space pose interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.task_space_velocity

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the task space velocity interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.task_space_acceleration

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the task space acceleration interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.task_space_wrench

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the task space wrench interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.task_space_wrench_derivative

If this parameter is set, it will override the default names for command interfaces, which are generated as described in the README.md file, specifically for the task space wrench derivative interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

