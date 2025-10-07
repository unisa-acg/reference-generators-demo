# Admittance parameters
float64[] mass                # 6-vector of mass terms used in the admittance control law
float64[] damping_ratio       # 6-vector of damping ratio terms used in the admittance control law
float64[] stiffness           # 6-vector of stiffness terms used in the admittance control law

# Frame information
std_msgs/String control_frame              # name of the control frame
std_msgs/String base_frame                 # name of the the base frame of the robot description used by the kinematics plugin
std_msgs/String ft_sensor_frame            # name of the force/torque sensor frame
bool[] selected_axes                       # 6-vector of booleans describing if admittance is enabled in the corresponding control frame axis

# State information
geometry_msgs/PoseStamped admittance_pose_state       # admittance actual pose
geometry_msgs/PoseStamped admittance_pose_reference   # admittance reference pose
geometry_msgs/PoseStamped admittance_pose_error       # admittance pose error

geometry_msgs/TwistStamped admittance_velocity_state      # admittance actual velocity
geometry_msgs/TwistStamped admittance_velocity_reference  # admittance reference velocity
geometry_msgs/TwistStamped admittance_velocity_error      # admittance velocity error

geometry_msgs/AccelStamped admittance_acceleration_state         # admittance actual acceleration
geometry_msgs/AccelStamped admittance_acceleration_reference     # admittance reference acceleration
geometry_msgs/AccelStamped admittance_acceleration_error         # admittance acceleration error

geometry_msgs/WrenchStamped wrench_state            # admittance actual wrench
geometry_msgs/WrenchStamped wrench_reference        # admittance reference wrench
geometry_msgs/WrenchStamped wrench_error            # admittance wrench error
