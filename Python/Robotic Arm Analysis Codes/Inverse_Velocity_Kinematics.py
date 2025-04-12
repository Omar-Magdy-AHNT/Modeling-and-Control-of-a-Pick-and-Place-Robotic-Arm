import numpy as np
from Python.xxx.Inverse_Velocity_Kinematics_Function import inverse_velocity_kinematics_numeric

# Function to convert degrees to radians
def deg2rad(degrees):
    return np.radians(degrees)


anglesz = eval(input("Enter angles for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3]: "))
q1z = deg2rad(-1 * anglesz[0])
q2z = deg2rad(-1 * anglesz[1] + 90)
q3z = deg2rad(-1 * anglesz[2] + 180)
q = np.array([q1z, q2z, q3z])  # Joint angles in radians

V = eval(input("Enter Velocities as a vector [Vx,Vy,Vz,wx,wy,wz]: "))
V_F = np.array([V[0],V[1],V[2],V[3],V[4],V[5]]) 

#q = np.array([deg2rad(-15), deg2rad(-30 + 90), deg2rad(-45 + 180)])  # Example angles in radians
#V_F = np.array([0.0061, 0.0029, 0.0098, -0.0161, -0.043, 0.0209])  # Example velocity values

# Call the inverse velocity kinematics function
q_dot_numeric = inverse_velocity_kinematics_numeric(q, V_F)

# Display numeric joint velocities
print("Numeric joint velocities (q_dot):")
print(q_dot_numeric)
