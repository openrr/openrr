# Used in time-stamping the message.
Header header

# Name list of the joints. You don't need to specify all joints of the
# robot. Joint names are case-sensitive.
string[] joint_names

# A position command to the joints listed in joint_names.
# The order must be identical.
# Units are meters or radians.
# If displacements and velocities are filled, a profiled motion is requested.
float64[] displacements # or position_deltas

# A velocity command to the joints listed in joint_names.
# The order must be identical.
# Units are m/s or rad/s.
# If displacements and velocities are filled, a profiled motion is requested.
float64[] velocities

float64 duration
