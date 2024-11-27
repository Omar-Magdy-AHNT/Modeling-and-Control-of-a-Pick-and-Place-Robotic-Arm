import sympy as sp

def calculate_jacobian(q1_val, q2_val, q3_val):
    # Define symbolic variables for joint angles
    q1, q2, q3 = sp.symbols('q1 q2 q3')

    # Define transformation matrices for each link
    A1 = sp.Matrix([
        [sp.cos(q1), 0, sp.sin(q1), 0.013 * sp.cos(q1)],
        [sp.sin(q1), 0, -sp.cos(q1), 0.013 * sp.sin(q1)],
        [0, -1, 0, 0.04],
        [0, 0, 0, 1]
    ])

    A2 = sp.Matrix([
        [sp.cos(q2), -sp.sin(q2), 0, -0.12 * sp.cos(q2)],
        [sp.sin(q2), sp.cos(q2), 0, -0.12 * sp.sin(q2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    A3 = sp.Matrix([
        [sp.cos(q3), -sp.sin(q3), 0, 0.09 * sp.cos(q3)],
        [sp.sin(q3), sp.cos(q3), 0, 0.09 * sp.sin(q3)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    A4 = sp.Matrix([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0],
        [0, 0, 1, -0.03],
        [0, 0, 0, 1]
    ])

    T = A1 * A2 * A3 * A4

    py = T[0, 3]
    px = T[1, 3]
    pz = T[2, 3]


    F = sp.Matrix([-px, py, pz])


    variables = sp.Matrix([q1, q2, q3])
    Jv = F.jacobian(variables)


    z0 = sp.Matrix([0, 0, 1])  
    z1 = A1[0:3, 2]  
    z2 = (A1 * A2)[0:3, 2]  
    Jw = sp.Matrix.hstack(z0, z1, z2)


    J = Jv.col_join(Jw)

    J = J.subs({q1: q1_val, q2: q2_val, q3: q3_val})

    return J
