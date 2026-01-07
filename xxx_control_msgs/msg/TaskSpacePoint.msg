# A task-space point defines pose, twist, acceleration, wrench, and wrench derivative typically used by controllers.

# Motion data are expressed in this reference frame.
string motion_frame
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel acceleration

# Wrenches are expressed in this reference frame.
string wrench_frame
geometry_msgs/Wrench wrench
geometry_msgs/Wrench wrench_derivative
