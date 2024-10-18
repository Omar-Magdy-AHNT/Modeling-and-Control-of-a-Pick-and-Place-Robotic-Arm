clc;
clear;

% Define symbolic variables
theta1 = sym('q1');
theta2 = sym('q2');
theta3 = sym('q3');

% Set numerical values for q1, q2, and q3
q1 = 0; % Replace with your desired value
q2 = pi/2; % Replace with your desired value
q3 = 0; % Replace with your desired value

% Define the transformation matrices
A1 = [cos(theta1) 0 sin(theta1) 0;
      sin(theta1) 0 -cos(theta1) 0;
      0 1 0 0.04;
      0 0 0 1];

A2 = [cos(theta2) -sin(theta2) 0 0.12*cos(theta2);
      sin(theta2) cos(theta2) 0 0.12*sin(theta2);
      0 0 1 0;
      0 0 0 1];

A3 = [cos(theta3) -sin(theta3) 0 0.09*cos(theta3);
      sin(theta3) cos(theta3) 0 0.09*sin(theta3);
      0 0 1 0;
      0 0 0 1];
  
A4 = [1 0 0 0.08;
      0 1 0 0;
      0 0 1 -0.02;
      0 0 0 1];

% Compute the overall transformation matrix
T = simplify(A1 * A2 * A3 * A4);

% Display the result
disp('Transformation matrix T:');
disp(T);

% Extract end-effector position
px = T(1,4);
py = T(2,4);
pz = T(3,4);

disp('End-effector Equations:');

disp('Px')
disp(px);

disp('Py')
disp(py);

disp('Pz')
disp(pz);

% Substitute q1, q2, q3 with numerical values
T_numeric = double(subs(T, {theta1, theta2, theta3}, {q1, q2, q3}));

% Extract end-effector position
px = T_numeric(1,4);
py = T_numeric(2,4);
pz = T_numeric(3,4);

% Display the result
disp('Transformation matrix T (numeric):');
%disp(T_numeric);

disp('End-effector Position:');
disp(['Px: ', num2str(px)]);
disp(['Py: ', num2str(py)]);
disp(['Pz: ', num2str(pz)]);