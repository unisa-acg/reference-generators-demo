# The header is used to specify the reference time for the trajectory durations
std_msgs/Header header

# Array of task-space trajectory points, which describe the poses, twists,
# accelerations and wrenches of the controlled frame at each time point.
acg_control_msgs/TaskSpaceTrajectoryPoint[] points
