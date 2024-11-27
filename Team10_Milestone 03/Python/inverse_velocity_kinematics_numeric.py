import sympy as sp
import numpy as np
from jacobian import calculate_jacobian 

def inverse_velocity_kinematics_numeric(q, V_F):

    J = calculate_jacobian(q[0], q[1], q[2])

    # Convert the symbolic Jacobian to a numeric matrix
    J_numeric = np.array(J).astype(np.float64)

    # Calculate the pseudo-inverse of the Jacobian using numpy
    J_inv = np.linalg.pinv(J_numeric)

    # Compute q_dot (joint velocities)
    q_dot = J_inv @ np.array(V_F, dtype=np.float64)

    return q_dot
