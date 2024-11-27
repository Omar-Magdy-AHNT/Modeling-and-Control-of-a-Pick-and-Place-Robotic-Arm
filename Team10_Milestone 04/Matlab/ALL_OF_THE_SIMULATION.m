
clc;
clear;
% initPos=  input('Enter INITIAL position of EEF to get Qi: ');
%angles = input('Enter INITIAL GUESSES for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3] for the INITIAL position: ');
% Xi = [initPos(1); initPos(2); initPos(3)];
q1i = deg2rad(-1*40);
q2i = deg2rad(-1*40+90);  
q3i = deg2rad(-1*80+180);
qi = [q1i; q2i; q3i];

q1z = deg2rad(-1*5);
q2z = deg2rad(-1*5+90);  
q3z = deg2rad(-1*10+180);

qz=[q1z;q2z;q3z];
% finPos=  input('Enter FINAL Position of EEF to get Qi: ');
 % angles = input('Enter INITIAL GUESSES for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3] for the FINAL position: ');
% Xf = [finPos(1); finPos(2); finPos(3)];
% q1f = deg2rad(-1*angles(1));
% q2f = deg2rad(-1*angles(2)+90);  
% q3f = deg2rad(-1*angles(3)+180);
qf = qi;
qm=qi;
Xi=[-0.03;-0.1951;0.0316];
Xm=[-0.1666; -0.1241; 0.1089];
Xf=[-0.1951; 0.03; 0.0316];
Xz=[-0.03;0.019;0.41];
qlama=qz;
X0=Xz;

q_req_init = inverse_pos_kinematics_func(qi, Xi);
q_req_mid = inverse_pos_kinematics_func(qm, Xm);
q_req_fin = inverse_pos_kinematics_func(qf, Xf);
q_req_zero = inverse_pos_kinematics_func(qz, Xz);
disp('Joint angles (in degrees) for INITIAL position:');
disp(q_req_init);
disp('-------------------------------------------------');
disp('Joint angles (in degrees) for MIDDLE position:');
disp(q_req_mid);
disp('-------------------------------------------------');
disp('Joint angles (in degrees) for FINAL position:');
disp(q_req_fin);
disp('-------------------------------------------------');
disp('Joint angles (in degrees) for FINAL position:');
disp(q_req_zero);
disp('--------------------------------------------------------------------------------------------------');
disp('--------------------------------------------------------------------------------------------------');
disp('--------------------------------------------------------------------------------------------------');
disp('--------------------------------------------------------------------------------------------------');








qdot0 = [0; 0; 0];       
qdotf = [0; 0; 0];       
Tf = 18;              
Ts = 0.1;             
time0=0:Ts:6;
time1=6:Ts:9;
time2=9:Ts:12;
time3=12:Ts:Tf;

Joint_Space0 = joint_traj(q_req_zero, q_req_init , qdot0, qdotf, 6, Ts);
Joint_Space1 = joint_traj(q_req_init, q_req_mid , qdot0, qdotf, 3, Ts);
Joint_Space2 = joint_traj(q_req_mid, q_req_fin , qdot0, qdotf, 3, Ts);
Joint_Space3 = joint_traj(q_req_fin, q_req_zero , qdot0, qdotf, 6, Ts);

PickUpTrajec=Joint_Space0(:,1:3);
angles_matrix1=Joint_Space1(:,1:3);
angles_matrix2=Joint_Space2(:,1:3);
PlaceTrajec=[angles_matrix1;angles_matrix2];
BackToOrigin=Joint_Space3(:,1:3);
pos_matrix=[ (Joint_Space0(:,4:6));(Joint_Space1(:,4:6)); (Joint_Space2(:,4:6));(Joint_Space3(:,4:6)) ];
angles_matrix = [PickUpTrajec;PlaceTrajec;BackToOrigin];

time = [time0,time1, time2,time3];

q1_series = timeseries(angles_matrix(:, 1), time); % First column
q2_series = timeseries(angles_matrix(:, 2), time); % Second column
q3_series = timeseries(angles_matrix(:, 3), time); % Third column
disp('Joint Positions at Each Time Step:');
disp(angles_matrix);



figure;
plot(time, angles_matrix);
xlabel('Time [s]');
ylabel('Joint Positions [deg]');
legend('Joint 1', 'Joint 2', ' Joint 3');
title('Joint Trajectories');
grid on;

figure;
plot(time, pos_matrix);
xlabel('Time [s]');
ylabel('EEF Position [m]');
legend('x', 'y', ' z');
title('EEF Trajectory');
grid on;

%uncomment to run simulation 
%sim('RoboticArm1.slx');

% figure;
% 
% plot3(Joint_Space(:,4), Joint_Space(:,5), Joint_Space(:,6), '-x', 'LineWidth', 1, 'MarkerSize', 6);
% 
% xlim([-5 5]);  
% ylim([-5 5]); 
% zlim([-5 5]);
% 
% xlabel('X-axis');
% ylabel('Y-axis');
% zlabel('Z-axis');
% title('3D Line Plot');
% grid on;
% axis equal;
% sim('RoboticArm1.slx')