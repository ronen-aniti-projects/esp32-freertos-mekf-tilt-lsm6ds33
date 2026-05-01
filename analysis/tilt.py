
# Load the data file and extract the quaternion estimate vs. time

# Form the rotation matrix W->B from the quaternion 

# Take the transpose to get B -> W rotation matrix

# Extract the z-axis basis (third column)

# Compute the tilt angle magnitude from the closed form cos^-1 expression

# Use the cross product to infer the direction of the tilt rotation 

# Plot two plots on the same figure: 

# 1. Tilt axis vector (3D) time trajectory (use arrow for initial point and points for the rest)

# 2. Tilt angle magnitude vs time in radians noting cos^-1 range is [0, pi)