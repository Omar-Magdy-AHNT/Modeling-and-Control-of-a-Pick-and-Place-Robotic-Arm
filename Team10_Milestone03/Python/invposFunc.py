import numpy as np
from InvPos import inverse_pos_kinematics_func

# Function to convert degrees to radians
def deg2rad(degrees):
    return np.radians(degrees)

# Prompt the user for desired position and initial angle guesses
desPos = eval(input("Enter desired Position to get to (as a vector [X, Y, Z]): "))
angles = eval(input("Enter INITIAL GUESSES for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3]: "))

# Convert to numpy arrays
desPos = np.array(desPos, dtype=float)
angles = np.array(angles, dtype=float)

# Convert initial angle guesses from degrees to radians and adjust
q10 = deg2rad(-1 * angles[0])
q20 = deg2rad(-1 * angles[1] + 90)
q30 = deg2rad(-1 * angles[2] + 180)
q0 = np.array([q10, q20, q30])

# Desired position vector
X = np.array([desPos[0], desPos[1], desPos[2]])

# Call the inverse kinematics function
q_req = inverse_pos_kinematics_func(q0, X)

# Display the required joint angles in degrees to reach the target position
print("Joint angles (in degrees) to reach the target position:")
print(q_req)
