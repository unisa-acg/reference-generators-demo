# Frame information
std_msgs/String motion_frame               # name of the the base reference frame
std_msgs/String wrench_frame               # name of the wrench reference frame

# State information
geometry_msgs/Pose admittance_pose_state             # admittance actual pose
geometry_msgs/Pose admittance_pose_reference         # admittance reference pose
geometry_msgs/Pose admittance_pose_scaled_reference  # admittance scaled reference pose
geometry_msgs/Pose admittance_pose_error             # admittance pose error
geometry_msgs/Pose admittance_pose_scaled_error      # admittance scaled pose error

geometry_msgs/Twist admittance_twist_state            # admittance actual twist
geometry_msgs/Twist admittance_twist_reference        # admittance reference twist
geometry_msgs/Twist admittance_twist_scaled_reference # admittance reference twist
geometry_msgs/Twist admittance_twist_error            # admittance twist error
geometry_msgs/Twist admittance_twist_scaled_error     # admittance twist scaled error

geometry_msgs/Accel admittance_acceleration_state     # admittance actual acceleration
geometry_msgs/Accel admittance_acceleration_reference # admittance reference acceleration
geometry_msgs/Accel admittance_acceleration_error     # admittance acceleration error

geometry_msgs/Wrench wrench_state            # admittance actual wrench
geometry_msgs/Wrench wrench_reference        # admittance reference wrench
geometry_msgs/Wrench wrench_error            # admittance wrench error

builtin_interfaces/Duration time_from_start       # time from the start of the controller
