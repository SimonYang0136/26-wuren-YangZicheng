# Custom message for turtle velocity control
# This message contains velocity information for the turtle

# Header for timestamp and frame information
std_msgs/Header header

# Linear velocity components
float32 linear_x
float32 linear_y

# Angular velocity
float32 angular_z

# Additional control parameters
float32 velocity_magnitude
bool is_emergency_stop

# Trajectory information
string trajectory_type
float32 trajectory_progress
