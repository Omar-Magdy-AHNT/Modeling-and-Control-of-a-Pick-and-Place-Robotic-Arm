clear;
clc;
anglesFWD = input('Enter angles for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3]: ');
[T, px, py, pz] = Forward_Position_Kinematics_Function(anglesFWD(1), anglesFWD(2), anglesFWD(3));
arrayPOS=[px py pz];
disp('End-effector Position:');
disp(['Px: ', num2str(px)]);
disp(['Py: ', num2str(py)]);
disp(['Pz: ', num2str(pz)]);
