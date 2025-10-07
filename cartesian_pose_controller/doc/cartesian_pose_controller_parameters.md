# Cartesian Pose Controller Parameters

Default Config
```yaml
cartesian_pose_controller:
  ros__parameters:
    alpha: 5.0e-06
    command_interfaces: '{position}'
    command_interfaces_names_override:
      joint_position: '{}'
      joint_velocity: '{}'
    joint_position_tolerance: 0.01
    joint_space_command_controller: ''
    joint_velocity_tolerance: 0.01
    joints: ''
    k_matrix_gains: '{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}'
    kinematics:
      plugin_name: ''
      plugin_package: ''
      tip: ''
    reference_interfaces_names_override:
      task_space_pose: '{}'
      task_space_twist: '{}'
    robot_name: ''
    root_link_frame: world
    state_interfaces_names_override:
      position: '{}'
    use_twist_reference: false

```

## joints

Specifies which joints will be used by the controller.

* Type: `string_array`
* Read only: True

## robot_name

Specifies the name of the robot.

* Type: `string`

## state_interfaces_names_override.position

If this parameter is set, it will override the default names (<robot_name>/<joint_name>/position) for state interfaces, specifically for the position interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## joint_space_command_controller

Specifies the down-stream controller that will receive the joint space commands.

* Type: `string`
* Read only: True

## command_interfaces

Specifies the interfaces that will be used to send commands to the joints.

* Type: `string_array`
* Default Value: {"position"}
* Read only: True

*Constraints:*
 - every element is one of the list ['position', 'velocity']
 - contains no duplicates
 - parameter is not empty

*Additional Constraints:*



## command_interfaces_names_override.joint_position

If this parameter is set, it will override the default names (<downstream_controller_name>/<joint_name>/position) for command interfaces for the joint position interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.joint_velocity

If this parameter is set, it will override the default names (<downstream_controller_name>/<joint_name>/velocity) for command interfaces for the joint velocity interface.

* Type: `string_array`
* Default Value: {}
* Read only: True

## use_twist_reference

Specifies if the task space twist interface is used. If not, the controller will compute the twist from the pose.

* Type: `bool`
* Default Value: false
* Read only: True

## reference_interfaces_names_override.task_space_pose

If this parameter is set, it will override the default names (<controller_name>/<interface_type>) for the pose reference interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## reference_interfaces_names_override.task_space_twist

If this parameter is set, it will override the default names (<controller_name>/<interface_type>) for the twist reference interfaces.

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



## root_link_frame

Specifies the root link frame. This frame is used as the reference frame for the task space references received by the controller. Note that this parameter must match the root link, as the kinematics interface assumes this.

* Type: `string`
* Default Value: "world"
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## k_matrix_gains

Specifies the gains for the K matrix.

* Type: `double_array`
* Default Value: {10.0, 10.0, 10.0, 10.0, 10.0, 10.0}
* Read only: True

*Constraints:*
 - length must be equal to 6

*Additional Constraints:*



## alpha

Specifies the damping factor when computing the inverse of the Jacobian.

* Type: `double`
* Default Value: 5e-06
* Read only: True

*Constraints:*
 - greater than 0.0

*Additional Constraints:*



## joint_position_tolerance

Specifies the joint position tolerance (unit: radians).

* Type: `double`
* Default Value: 0.01
* Read only: True

*Constraints:*
 - greater than 0.0

*Additional Constraints:*



## joint_velocity_tolerance

Specifies the joint velocity tolerance (unit: radians/second).

* Type: `double`
* Default Value: 0.01
* Read only: True

*Constraints:*
 - greater than 0.0

*Additional Constraints:*



