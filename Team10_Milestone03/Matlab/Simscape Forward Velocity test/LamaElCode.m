clc;
clear;

function q = inverse_kinematics_func(q0, X)
    % This function computes the joint angles required to reach a target
    % end-effector position X = [X_desired; Y_desired; Z_desired]
    
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

    % Desired position (target)
    X_desired = X;  % [X_desired; Y_desired; Z_desired]
    % Initialize variables for the iterative solution
    q = q0;   % Starting guess for joint angles
    tol = 1e-3;  % Tolerance for convergence
    max_iter = 100;  % Maximum number of iterations

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
%disp('Transformation matrix T (numeric):');
%disp(T_numeric);

%%disp('End-effector Position:');
%disp(['Px: ', num2str(px)]);
%disp(['Py: ', num2str(py)]);
%disp(['Pz: ', num2str(pz)]);
J = jacobian([px; py; pz], [theta1, theta2, theta3]);

  % Numerical loop for iterative inverse kinematics
    for i = 1:max_iter
        % Substitute current guess of q into px, py, pz and J
        current_position = double(subs([px; py; pz], [q1, q2, q3], q));
        J_numeric = double(subs(J, [q1, q2, q3], q));
        
        % Compute error between current and desired position
        error = X_desired - current_position;
        
        % Check if error is within tolerance
        if norm(error) < tol
            break;
        end
        
        % Update rule (Newton-Raphson method)
        delta_q = J_numeric \ error;
        q = q + delta_q';  % Update joint angles
    end
    
    % Display a message if max iterations were reached without convergence
    if i == max_iter
        disp('Warning: Max iterations reached without convergence');
    end
end



disp('Jacobian matrix J:');
disp(J);
J_inv = simplify(inv(J));
disp('Inverse Jacobian matrix J_inv (symbolic):');
disp(J_inv);
