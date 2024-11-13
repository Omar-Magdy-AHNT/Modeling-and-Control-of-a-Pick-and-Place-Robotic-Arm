clc;
clear;
% Define symbolic joint angles and end-effector velocities
syms q1 q2 q3 vx vy vz wx wy wz
q = [q1; q2; q3];
V_F = [vx; vy; vz; wx; wy; wz];  % End-effector velocity vector

% Calculate the symbolic joint velocities
q_dot = inverse_velocity_kinematics_symbolic(q, V_F);
disp('Joint velocities (q_dot):');
disp(q_dot);
