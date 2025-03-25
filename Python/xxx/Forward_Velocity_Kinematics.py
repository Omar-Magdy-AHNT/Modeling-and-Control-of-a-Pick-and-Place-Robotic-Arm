import numpy as np
from Python.xxx.Forward_Velocity_Kinematics_Function import forward_velocity_kinematics_numeric

# Function to convert degrees to radians
def deg2rad(degrees):
    return np.radians(degrees)

# Prompt the user for joint angles and angular velocities
angles = eval(input("Enter angles for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3]: "))
angularV = eval(input("Enter q dot for q1, q2, and q3 (rad/s) as a vector [q1_dot, q2_dot, q3_dot]: "))

# Convert the angles from degrees to radians and adjust
q10 = deg2rad(-1 * angles[0])
q20 = deg2rad(-1 * angles[1] + 90)
q30 = deg2rad(-1 * angles[2] + 180)
q = np.array([q10, q20, q30])  # Joint angles in radians

# Angular velocities
q_dot = np.array([angularV[0], angularV[1], angularV[2]])  # Joint angular velocities in rad/s

# Call the forward velocity kinematics function
V_F = forward_velocity_kinematics_numeric(q, q_dot)

# Display the result
print("Forward velocity (V_F):", V_F)
