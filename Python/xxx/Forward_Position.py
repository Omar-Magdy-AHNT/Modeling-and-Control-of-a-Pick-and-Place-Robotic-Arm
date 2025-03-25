import numpy as np
from sympy import cos, sin, Matrix

# Define symbolic variables

angles = eval(input("Enter INITIAL GUESSES for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3]: "))

# Set numerical values for q1, q2, and q3
q1 = np.deg2rad(-1* (float)(angles[0]))
q2 = np.deg2rad((-1* (float)(angles[1])) +90)
q3 = np.deg2rad((-1* (float)(angles[2])) +180)

# Define the transformation matrices
A1 = Matrix([
    [cos(q1), 0, sin(q1), 0.013 * cos(q1)],
    [sin(q1), 0, -cos(q1), 0.013 * sin(q1)],
    [0, -1, 0, 0.04],
    [0, 0, 0, 1]
])

A2 = Matrix([
    [cos(q2), -sin(q2), 0, -0.12 * cos(q2)],
    [sin(q2), cos(q2), 0, -0.12 * sin(q2)],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

A3 = Matrix([
    [cos(q3), -sin(q3), 0, 0.09 * cos(q3)],
    [sin(q3), cos(q3), 0, 0.09 * sin(q3)],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

A4 = Matrix([
    [1, 0, 0, 0.1],
    [0, 1, 0, 0],
    [0, 0, 1, -0.03],
    [0, 0, 0, 1]
])

# Compute the overall transformation matrix
T = A1 * A2 * A3 * A4

# Display the result
print("Transformation matrix T:")
print(T)

# Extract end-effector position
py, px, pz = T[0, 3], T[1, 3], T[2, 3]

print("End-effector Equations:")
print("Px =", px)
print("Py =", py)
print("Pz =", pz)

# Substitute q1, q2, q3 with numerical values
T_numeric = T.subs({q1: q1, q2: q2, q3: q3}).evalf()

# Extract end-effector position numerically
px_numeric = -T_numeric[1, 3]
py_numeric = T_numeric[0, 3]
pz_numeric = T_numeric[2, 3]

# Display the result
print("Transformation matrix T (numeric):")
print(T_numeric)

print("End-effector Position:")
print(f"Px: {px_numeric}")
print(f"Py: {py_numeric}")
print(f"Pz: {pz_numeric}")
