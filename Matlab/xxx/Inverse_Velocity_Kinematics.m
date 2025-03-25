clc;
clear;
anglesz = input('Enter angles for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3]: ');

q1z = deg2rad(-1*anglesz(1));
q2z = deg2rad(-1*anglesz(2)+90);  
q3z = deg2rad(-1*anglesz(3)+180);

q = [q1z; q2z; q3z];   
V_F = [0.140952158760884;0.299544451053512;0.323930822144476;-0.347296355333861;-1.969615506024416;0.500000000000000];  % Example velocity values

q_dot_numeric = Inverse_Velocity_Kinematics_Function(q, V_F);

disp('Numeric joint velocities (q_dot):');
disp(q_dot_numeric);
