clc;
clear;
% initPos=  input('Enter INITIAL position of EEF to get Qi: ');
 angles = input('Enter INITIAL GUESSES for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3] for the INITIAL position: ');
% Xi = [initPos(1); initPos(2); initPos(3)];
q1i = deg2rad(-1*angles(1));
q2i = deg2rad(-1*angles(2)+90);  
q3i = deg2rad(-1*angles(3)+180);
qi = [q1i; q2i; q3i];

Xi=[-0.03; 0.013;0.35];

% finPos=  input('Enter FINAL Position of EEF to get Qi: ');
 angles = input('Enter INITIAL GUESSES for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3] for the FINAL position: ');
% Xf = [finPos(1); finPos(2); finPos(3)];
q1f = deg2rad(-1*angles(1));
q2f = deg2rad(-1*angles(2)+90);  
q3f = deg2rad(-1*angles(3)+180);
qf = [q1f; q2f; q3f];
  
Xf=[-0.03;-0.2062;-0.0095];


q_req_init = inverse_pos_kinematics_func(qi, Xi);
q_req_fin = inverse_pos_kinematics_func(qf, Xf);
disp('Joint angles (in degrees) for INITIAL position:');
disp(q_req_init);
disp('-------------------------------------------------');
disp('Joint angles (in degrees) for FINAL position:');
disp(q_req_fin);
disp('-------------------------------------------------');
disp('-------------------------------------------------');

