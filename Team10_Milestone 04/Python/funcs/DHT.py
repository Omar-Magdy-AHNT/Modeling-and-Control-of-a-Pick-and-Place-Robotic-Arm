import sympy as sp
import numpy as np
def compute_transformation_matrix():
    """
    Computes the symbolic transformation matrix for a 3-DOF robotic arm.
    Returns:
        T (sympy.Matrix): The symbolic transformation matrix.
    """
    # Define symbolic variables
    theta1, theta2, theta3 = sp.symbols('q1 q2 q3')

    # Define transformation matrices
    A1 = sp.Matrix([
        [sp.cos(theta1), 0, sp.sin(theta1), 0.019 * sp.cos(theta1)],
        [sp.sin(theta1), 0, -sp.cos(theta1), 0.019 * sp.sin(theta1)],
        [0, -1, 0, 0.1],
        [0, 0, 0, 1]
    ])

    A2 = sp.Matrix([
        [sp.cos(theta2), -sp.sin(theta2), 0, -0.12 * sp.cos(theta2)],
        [sp.sin(theta2), sp.cos(theta2), 0, -0.12 * sp.sin(theta2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    A3 = sp.Matrix([
        [sp.cos(theta3), -sp.sin(theta3), 0, 0.09 * sp.cos(theta3)],
        [sp.sin(theta3), sp.cos(theta3), 0, 0.09 * sp.sin(theta3)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    A4 = sp.Matrix([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0],
        [0, 0, 1, -0.03],
        [0, 0, 0, 1]
    ])

    # Compute the overall transformation matrix
    T = sp.simplify(A1 * A2 * A3 * A4)

    return T

def extract_position(T, q1, q2, q3):

    theta1, theta2, theta3 = sp.symbols('q1 q2 q3')
    
    q1 = np.deg2rad(-q1)
    q2 = np.deg2rad(-q2 + 90)
    q3 = np.deg2rad(-q3 + 180)
    # Substitute joint angles into the transformation matrix
    T_numeric = T.subs({theta1: q1, theta2: q2, theta3: q3})

    # Extract px, py, pz
    px = -1*float(T_numeric[1, 3])
    py = float(T_numeric[0, 3])
    pz = float(T_numeric[2, 3])

    return px, py, pz
