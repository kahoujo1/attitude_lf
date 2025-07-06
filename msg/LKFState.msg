# UAVState.msg

# Timestamp
std_msgs/Header header

# Position (x, y, z)
geometry_msgs/Vector3 position

# Velocity (x, y, z)
geometry_msgs/Vector3 velocity

# Attitude (roll, pitch, yaw)
Attitude attitude

# Angular velocity (roll, pitch, yaw)
Attitude angular_vel

# Acceleration (x, y, z)
geometry_msgs/Vector3 acceleration

# Thrust
float64 thrust

# acceleration offset
float64 ax_offset
float64 ay_offset
