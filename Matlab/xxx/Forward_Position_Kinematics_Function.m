function [T, px, py, pz] = Forward_Position_Kinematics_Function(q1_deg, q2_deg, q3_deg)
theta1 = sym('q1');
theta2 = sym('q2');
theta3 = sym('q3');
q1 = deg2rad(-1*q1_deg);
q2 = deg2rad(-1*q2_deg+90);  
q3 = deg2rad(-1*q3_deg+180);

% Define the transformation matrices
A1 = [cos(theta1) 0 sin(theta1) 0.013*cos(theta1);
      sin(theta1) 0 -cos(theta1) 0.013*sin(theta1);
      0 -1 0 0.04;
      0 0 0 1];

A2 = [cos(theta2) -sin(theta2) 0 -0.12*cos(theta2);
      sin(theta2) cos(theta2) 0 -0.12*sin(theta2);
      0 0 1 0;
      0 0 0 1];

A3 = [cos(theta3) -sin(theta3) 0 0.09*cos(theta3);
      sin(theta3) cos(theta3) 0 0.09*sin(theta3);
      0 0 1 0;
      0 0 0 1];
  
A4 = [1 0 0 0.1;
      0 1 0 0;
      0 0 1 -0.03
      0 0 0 1];
T = simplify(A1 * A2 * A3 * A4);

py = T(1,4);
px = T(2,4);
pz = T(3,4);

T_numeric = double(subs(T, {theta1, theta2, theta3}, {q1, q2, q3}));

py = 1*T_numeric(1,4);
px = -1*T_numeric(2,4);
pz = T_numeric(3,4);

end