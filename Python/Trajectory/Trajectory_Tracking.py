import json
import numpy as np
import matplotlib.pyplot as plt
from Functions import joint_traj, inverse_pos_kinematics_func, EEF_Pos_and_DH

def deg_to_rad(deg):
    return np.deg2rad(deg)

# Inputs for initial position
angles = np.array([40, 40, 80])  # Initial guesses for [q1, q2, q3] in degrees

q1i = deg_to_rad(-1 * angles[0])
q2i = deg_to_rad(-1 * angles[1] + 90)
q3i = deg_to_rad(-1 * angles[2] + 180)
qi = np.array([q1i, q2i, q3i])

# Inputs for final position
final_pos = np.array([-0.1951, 0.03, 0.0316])  
angles = np.array([5, 40, 80])  

q1f = deg_to_rad(-1 * angles[0])
q2f = deg_to_rad(-1 * angles[1] + 90)
q3f = deg_to_rad(-1 * angles[2] + 180)
qf = qi
qm=qi

Xi=[-0.03,-0.1951,0.0316]
Xm=[-0.1666, -0.1241, 0.1089]
Xf=[-0.1951, 0.03, 0.0316]
Xz=[-0.03,0.019,0.41]
q1z = np.deg2rad(-1*5)
q2z = np.deg2rad(-1*5+90)  
q3z = np.deg2rad(-1*10+180)

qz = np.array([q1z, q2z, q3z])

# Compute required joint angles for initial and final positions
q_req_init = inverse_pos_kinematics_func(qi, Xi)
q_req_mid = inverse_pos_kinematics_func(qm, Xm)
q_req_fin = inverse_pos_kinematics_func(qf, Xf)
q_req_zero = inverse_pos_kinematics_func(qz, Xz)

# Joint trajectory generation
qdot0 = np.array([0, 0, 0])  
qdotf = np.array([0, 0, 0])  
Tf = 18                      
Ts = 0.1                    
time0 = np.arange(0, 6, Ts)
time1 = np.arange(6, 9+Ts, Ts)
time2 = np.arange(9, 12+Ts, Ts)
time3 = np.arange(12, 18+Ts, Ts)

time= np.hstack((time0,time1,time2,time3))
# Generate the joint trajectory
Joint_Space0 = joint_traj(q_req_zero, q_req_init, qdot0, qdotf, 6, Ts, EEF_Pos_and_DH)
Joint_Space1 = joint_traj(q_req_init, q_req_mid, qdot0, qdotf, 3, Ts, EEF_Pos_and_DH)
Joint_Space2 = joint_traj(q_req_mid, q_req_fin, qdot0, qdotf, 3, Ts, EEF_Pos_and_DH)
Joint_Space3 = joint_traj(q_req_fin, q_req_zero, qdot0, qdotf, 6, Ts, EEF_Pos_and_DH)


angles_matrix0 = Joint_Space0[:, :3] 
angles_matrix1 = Joint_Space1[:, :3]
angles_matrix2 = Joint_Space2[:, :3] 
angles_matrix3 = Joint_Space3[:, :3]  
eef_positions = np.vstack((Joint_Space0[:, 3:6],Joint_Space1[:, 3:6], Joint_Space2[:, 3:6],Joint_Space3[:, 3:6]))
angles_matrix = np.vstack((angles_matrix0,angles_matrix1, angles_matrix2,angles_matrix3))

all_joint_angles = angles_matrix.flatten()

print(" ".join(map(str, all_joint_angles)))


# Plot the joint angles
plt.figure()
plt.plot(time, angles_matrix)
plt.xlabel('Time [s]')
plt.ylabel('Joint Positions [rad]')
plt.legend(['Joint 1', 'Joint 2', 'Joint 3'])
plt.title('Joint Trajectories') 
plt.grid(True)

# Plot the EEF trajectory
plt.figure()
plt.plot(time, eef_positions)
plt.xlabel('Time [s]')
plt.ylabel('EEF Positions [m]')
plt.legend(['X', 'Y', 'Z'])
plt.title('EEF Trajectory')
plt.grid(True)

plt.show()


