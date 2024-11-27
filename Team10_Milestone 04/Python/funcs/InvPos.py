import numpy as np
import sympy as sp
def rtd(r):
    return np.rad2deg(r)

def inverse_pos_kinematics_func(q0, X_desired):
    # Define symbolic variables for joint angles
    q1, q2, q3 = sp.symbols('q1 q2 q3', real=True)
    
    # Define transformation matrices
    A1 = sp.Matrix([
        [sp.cos(q1), 0, sp.sin(q1), 0.019 * sp.cos(q1)],
        [sp.sin(q1), 0, -sp.cos(q1), 0.019 * sp.sin(q1)],
        [0, -1, 0, 0.1],
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

    # Compute the overall transformation matrix
    T = sp.simplify(A1 * A2 * A3 * A4)
    
    # Extract end-effector position
    px, py, pz = T[1, 3], T[0, 3], T[2, 3]
    
    # Initialize variables for the iterative solution
    q = np.array(q0).reshape((3,))  # Ensure q is a 1D array
    tol = np.array([1e-8, 1e-8, 1e-8])
    max_iter = 1000  # Maximum number of iterations
    
    # Calculate the Jacobian matrix symbolically
    J = sp.Matrix([[-px], [py], [pz]]).jacobian([q1, q2, q3])
    
    # Iterative inverse kinematics
    for i in range(max_iter):
        # Substitute current guess of q into px, py, pz
        current_position = np.array(sp.Matrix([[-px], [py], [pz]]).subs({q1: q[0], q2: q[1], q3: q[2]})).astype(np.float64).flatten()

        # Compute error between current and desired position
        error = X_desired - current_position
        
        # Debugging information
        #print(f"Iteration: {i+1}")
        #print(f"Current position: {current_position}")
        #print(f"Error: {error}")
        
        # Check if each component of the error is within tolerance
        if np.all(np.abs(error) <= tol):
         #   print("Convergence achieved.")
          #  print(f"Error: {error}")
           # print(f"Tolerance: {tol}")
            break  # Exit loop if within tolerance for all dimensions
        
        # Calculate the Jacobian numerically
        J_numeric = np.array(J.subs({q1: q[0], q2: q[1], q3: q[2]})).astype(np.float64)
        
        # Check if Jacobian is invertible by determinant
        determinant = np.linalg.det(J_numeric)
        #print(f"Jacobian determinant: {determinant}")
        
        if determinant != 0:
            J_inv = np.linalg.inv(J_numeric)
            delta_q = J_inv.dot(error)  # Calculate the update for joint angles

            # Limit the maximum change in joint angles
            max_delta = 0.1
            q += np.sign(delta_q) * np.minimum(np.abs(delta_q), max_delta)
        else:
            #print("Jacobian is singular at this configuration.")
            q += 0.01 * np.random.randn(*q.shape)  # Slightly perturb the angles
            #print("Adjusted joint angles to escape singularity.")
            continue  # Skip to the next iteration

        # Display updated joint angles in degrees
        #print(f"Updated joint angles: {np.degrees(q)}")

    # Display a message if max iterations were reached without convergence
    if i == max_iter - 1:
        print("Warning: Max iterations reached without convergence")
    
    # Adjust final joint angles to match the expected transformation
    q[0] = rtd(-q[0])
    q[1] = rtd(-q[1] + np.pi / 2)
    q[2] = rtd(-q[2] + np.pi)

    
    
    # Convert to degrees for the final result
    return np.round(q, decimals=2)
