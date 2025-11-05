# Admittance Filter Parameters

Default Config
```yaml
admittance_filter:
  ros__parameters:
    compliant_axis: ''
    damping_ratio: ''
    mass: ''
    order: ''
    stiffness: ''

```

## mass

Specifies the mass (M_d) values for the translational (x, y, z) and rotational (rx, ry, rz) directions used in the admittance law.

* Type: `double_array`

*Constraints:*
 - length must be equal to 6
 - each element of array must be within bounds 0.0001

*Additional Constraints:*



## damping_ratio

Specifies damping ratio (zeta) values for the translational (x, y, z) and rotational (rx, ry, rz) directions used in the admittance law. The corresponding damping (K_D) is obtained as: K_D = zeta * (2 * sqrt( M_d * K_P )).

* Type: `double_array`

*Constraints:*
 - length must be equal to 6
 - each element of array must be within bounds 0.0

*Additional Constraints:*



## stiffness

Specifies the stiffness (K_P) values for the translational (x, y, z) and rotational (rx, ry, rz) directions used in the admittance law.

* Type: `double_array`

*Constraints:*
 - length must be equal to 6
 - each element of array must be within bounds 0.0

*Additional Constraints:*



## order

Specifies the order of the admittance filter in the set {0, 1, 2}.

* Type: `int`

*Constraints:*
 - parameter must be within bounds 0

*Additional Constraints:*



## compliant_axis

Specifies the axes to be used in the admittance filter. The axes are defined as follows: x, y, z, rx, ry, rz

* Type: `bool_array`

*Constraints:*
 - length must be equal to 6

*Additional Constraints:*



