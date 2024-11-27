function V_F = forward_velocity_kinematics_symbolic(q)
    % This function calculates the forward velocity kinematics symbolically
    % Input:
    % q - [q1; q2; q3] symbolic joint angles

    % Define symbolic variables for joint angles and their velocities
    syms q1_dot q2_dot q3_dot

    % Substitute symbolic joint angles in the Jacobian matrix function
    syms q1 q2 q3
    J = jacobian_matrix(q1, q2, q3);
    
    % Define symbolic joint velocity vector
    q_dot_sym = [q1_dot; q2_dot; q3_dot];

    % Forward velocity kinematics (symbolic output)
    V_F = simplify(J * q_dot_sym);

    % Print Jacobian matrix and forward velocity in compact form
    disp('Jacobian Matrix (J):');
    disp(char(J));  % Display Jacobian matrix as a string

    disp('Forward Velocity (V_F):');
    disp(char(V_F));  % Display forward velocity as a string
end

