clc;
clear;

angles = input('Enter angles for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3]: ');
q10 = deg2rad(-1*angles(1));
q20 = deg2rad(-1*angles(2)+90);  
q30 = deg2rad(-1*angles(3)+180);

angularV = input('Enter q dot for q1, q2, and q3 (rad/s) as a vector [q1_dot, q2_dot, q3_dot]: ');

q = [q10; q20; q30];  % Joint angles in radians
q_dot = [angularV(1); angularV(2); angularV(3)];  % Joint angular velocities
V_F = forward_velocity_kinematics_numeric(q, q_dot);
disp(V_F)  % Displays the numeric forward velocity result
