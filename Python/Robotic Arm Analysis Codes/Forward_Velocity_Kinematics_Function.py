import sympy as sp
from Python.xxx.jacobian import calculate_jacobian  # Assuming you saved the Jacobian function in robot_jacobian.py

def forward_velocity_kinematics_numeric(q, q_dot):

    # Define symbolic variables for joint angles and their velocities
    q1_sym, q2_sym, q3_sym = sp.symbols('q1 q2 q3')
    q1_dot, q2_dot, q3_dot = sp.symbols('q1_dot q2_dot q3_dot')
    
    # Obtain the symbolic Jacobian matrix with substituted joint angles
    J = calculate_jacobian(q[0], q[1], q[2])

    # Define symbolic joint velocity vector
    q_dot_sym = sp.Matrix([q1_dot, q2_dot, q3_dot])

    # Symbolic forward velocity kinematics
    V_F_symbolic = J * q_dot_sym

    # Substitute numerical values for q_dot and evaluate the expression
    V_F = V_F_symbolic.subs({q1_dot: q_dot[0], q2_dot: q_dot[1], q3_dot: q_dot[2]}).evalf()

    return V_F
