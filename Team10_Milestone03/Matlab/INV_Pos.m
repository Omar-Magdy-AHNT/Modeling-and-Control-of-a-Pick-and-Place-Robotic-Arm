clc;
desPos=  input('Enter desired Position to get to: ');
angles = input('Enter INITIAL GUESSES for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3]: ');

q10 = deg2rad(-1*angles(1));
q20 = deg2rad(-1*angles(2)+90);  
q30 = deg2rad(-1*angles(3)+180);

q0 = [q10; q20; q30];

X = [desPos(1); desPos(2); desPos(3)];  

q_req = inverse_pos_kinematics_func(q0, X); 
disp('Joint angles (in degrees) to reach the target position:');
disp(q_req);
