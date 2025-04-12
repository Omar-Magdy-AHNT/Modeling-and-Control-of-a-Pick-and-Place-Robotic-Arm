function q = inverse_pos_kinematics_func(q0, X)
    % This function computes the joint angles required to reach a target
    % end-effector position X = [X_desired; Y_desired; Z_desired]

    % Define symbolic variables for joint angles
    syms q1 q2 q3 real

    

    theta1 = sym('q1');
    theta2 = sym('q2');
    theta3 = sym('q3');
    
    A1 = [cos(theta1) 0 sin(theta1) 0.019*cos(theta1);
      sin(theta1) 0 -cos(theta1) 0.019*sin(theta1);
      0 -1 0 0.1;
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

% Compute the overall transformation matrix
    T = simplify(A1 * A2 * A3 * A4);
% Extract end-effector position
    py = T(1,4);
    px = T(2,4);
    pz = T(3,4);
    
    % Desired position (target)
    X_desired = X;  % [X_desired; Y_desired; Z_desired]
    
    % Initialize variables for the iterative solution
    q = q0(:);   % Ensure q is a column vector for compatibility
    tol = [1e-8; 1e-8; 1e-8];  % Tolerance for each dimension
    max_iter = 500; % Maximum number of iterations
    
    % Calculate the Jacobian matrix symbolically
    J = jacobian([-1*px; 1*py; pz], [q1, q2, q3]);
    
    % Numerical loop for iterative inverse kinematics
    for i = 1:max_iter
        % Substitute current guess of q into px, py, pz
        current_position = double(subs([-1*px; 1*py; pz], [q1, q2, q3], num2cell(q.')));

        % Compute error between current and desired position
        error = X_desired - current_position;
        
        % Debugging information
        %disp(['Iteration: ', num2str(i)]);
        %disp(['Current position: ', mat2str(current_position)]);
        %disp(['Error: ', mat2str(error)]);
        
        % Check if each component of the error is within tolerance
        if all(abs(error) <= tol)  % Changed < to <= for strict checking
         %    disp('Convergence achieved.');
          %   disp(['Error: ', mat2str(error)]);
           %  disp(['Tolerance: ', mat2str(tol)]);
             break; % Exit loop if within tolerance for all dimensions
        end
        
        % Calculate the Jacobian and its inverse
        J_numeric = double(subs(J, [q1, q2, q3], num2cell(q.')));
        determinant = det(J_numeric);
        %disp(['Jacobian determinant: ', num2str(determinant)]);  % Debugging: Print determinant
        
        if determinant ~= 0  % Check if Jacobian is invertible
            J_inv = inv(J_numeric);
            delta_q = J_inv * error;  % Calculate the update for joint angles

            % Limit the maximum change in joint angles
            max_delta = 0.1; % Maximum allowable change in joint angles
            q = q + sign(delta_q) .* min(abs(delta_q), max_delta);  % Update joint angles
        else
          %  disp('Jacobian is singular at this configuration.');
            q = q + 0.01 * randn(size(q));  % Slightly perturb the angles
         %   disp('Adjusted joint angles to escape singularity.');
            continue;  % Skip to the next iteration
        end

        % After updating, ensure to display updated joint angles
        %disp(['Updated joint angles: ', mat2str(rad2deg(q))]);  % Show in degrees for clarity
    end
    
    % Display a message if max iterations were reached without convergence
    if i == max_iter
        disp('Warning: Max iterations reached without convergence');
    end

    % Ensure that the joint angles are returned from the function
    q = q;  % Ensure q remains a column vector
    q(1) = -1*q(1);
    q(2) = -1*q(2) + pi/2; 
    q(3) = -1*q(3) + pi;  
    % Display the final joint angles in degrees
    q = rad2deg(q);  % Convert radians to degrees
end
