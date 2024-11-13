clc;
clear;
syms q1 q2 q3 q1_dot q2_dot q3_dot
V_F = forward_velocity_kinematics_symbolic([q1; q2; q3]);
disp(V_F)  % Displays the symbolic forward velocity expression
