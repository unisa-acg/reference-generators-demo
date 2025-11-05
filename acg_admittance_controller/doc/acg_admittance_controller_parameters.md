# Acg Admittance Controller Parameters

Default Config
```yaml
acg_admittance_controller:
  ros__parameters:
    force_limitation_enabled: false
    logging_enabled: false
    reference_scaling_gain: ''
    state_publish_frequency: 125.0

```

## force_limitation_enabled

Specifies whether the force limitation is enabled or not.

* Type: `bool`
* Default Value: false

## reference_scaling_gain

Specifies the scaling gain for the position reference.

* Type: `double`

*Constraints:*
 - parameter must be within bounds 0.0

*Additional Constraints:*



## logging_enabled

Specifies whether logging is enabled or not.

* Type: `bool`
* Default Value: false

## state_publish_frequency

The frequency (in Hz) at which the controller state is published.

* Type: `double`
* Default Value: 125.0

*Constraints:*
 - greater than or equal to 0.0

*Additional Constraints:*



