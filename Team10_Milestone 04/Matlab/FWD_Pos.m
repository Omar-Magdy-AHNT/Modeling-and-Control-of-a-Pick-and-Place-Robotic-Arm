
anglesFWD = input('Enter angles for q1, q2, and q3 (in degrees) as a vector [q1, q2, q3]: ');
[T, px, py, pz] = EEF_Pos_and_DH(anglesFWD(1), anglesFWD(2), anglesFWD(3));
arrayPOS=[px py pz];
disp('End-effector Position:');
disp(arrayPOS);

%disp(['Px: ', num2str(px)]);
%disp(['Py: ', num2str(py)]);
%disp(['Pz: ', num2str(pz)]);
