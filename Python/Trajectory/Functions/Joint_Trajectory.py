import numpy as np

from Functions import DH_Table as DHT

def joint_traj(q0, qf, qdot0, qdotf, Tf, Ts, EEF_Pos_and_DH):

    T = DHT.compute_transformation_matrix()
    # Define time vector
    t = np.arange(0, Tf + Ts, Ts)

    # Number of joints
    n_joints = len(q0)

    # Initialize the Joint_Space array
    joint_space = np.zeros((len(t), 6))

    # Generate trajectory for each joint
    for i in range(n_joints):
        # Define the matrix A for cubic polynomial coefficients
        A = np.array([
            [1, 0, 0, 0],                 # q(0) = q0
            [1, Tf, Tf**2, Tf**3],       # q(Tf) = qf
            [0, 1, 0, 0],                 # qdot(0) = qdot0
            [0, 1, 2*Tf, 3*Tf**2]        # qdot(Tf) = qdotf
        ])

        # Define the boundary conditions vector
        b = np.array([q0[i], qf[i], qdot0[i], qdotf[i]])

        # Solve for the cubic polynomial coefficients
        coeffs = np.linalg.solve(A, b)  # [a0, a1, a2, a3]

        # Compute the trajectory for this joint
        for j, time in enumerate(t):
            joint_space[j, i] = (coeffs[0] + coeffs[1]*time + coeffs[2]*time**2 + coeffs[3]*time**3)

        
        for j in range(len(t)):
            px, py, pz = DHT.extract_position(T,
                joint_space[j, 0], 
                joint_space[j, 1], 
                joint_space[j, 2]
            )
            joint_space[j, 3] = px
            joint_space[j, 4] = py
            joint_space[j, 5] = pz

    return joint_space
