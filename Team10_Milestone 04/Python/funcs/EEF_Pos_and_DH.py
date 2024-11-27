import sympy as sp
import numpy as np

def EEF_Pos_and_DH(q1_deg, q2_deg, q3_deg):
    
    # Define symbolic variables
    theta1, theta2, theta3 = sp.symbols('q1 q2 q3')

    # Convert input angles to radians
    q1 = np.deg2rad(-q1_deg)
    q2 = np.deg2rad(-q2_deg + 90)
    q3 = np.deg2rad(-q3_deg + 180)

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

    # Extract px, py, pz from the symbolic transformation matrix
    py_sym = T[0, 3]
    px_sym = T[1, 3]
    pz_sym = T[2, 3]

    # Substitute the joint angles into the symbolic matrix
    T_numeric = T.subs({theta1: q1, theta2: q2, theta3: q3}).evalf()

    # Convert px, py, pz to numeric values
    px = -1 * T_numeric[1, 3]
    py = T_numeric[0, 3]
    pz = T_numeric[2, 3]

    return T_numeric, float(px), float(py), float(pz)
