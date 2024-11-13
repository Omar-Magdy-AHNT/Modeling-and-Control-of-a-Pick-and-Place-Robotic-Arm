function V_F = forward_velocity_kinematics(q, q_dot)
    % This function calculates the forward velocity kinematics
    % Input:
    % q - [q1; q2; q3] joint angles (symbolic or numeric)
    % q_dot - [q1_dot; q2_dot; q3_dot] joint angular velocities (symbolic or numeric)
    
    % Define symbolic variables for joint angles and their velocities
    syms q1_sym q2_sym q3_sym q1_dot q2_dot q3_dot

    % Obtain the Jacobian matrix using symbolic variables
    J = jacobian_matrix(q1_sym, q2_sym, q3_sym);

    % Define symbolic joint velocity vector
    q_dot_sym = [q1_dot; q2_dot; q3_dot];

    % Forward velocity kinematics in symbolic form
    V_F_symbolic = simplify(J * q_dot_sym);

    % Check if both q and q_dot are provided as numeric values
    if nargin == 2 && ~any([isnan(q), isnan(q_dot)]) && isnumeric(q) && isnumeric(q_dot)
        % Substitute numerical values if q and q_dot are provided
        V_F = double(subs(V_F_symbolic, ...
            {q1_sym, q2_sym, q3_sym, q1_dot, q2_dot, q3_dot}, ...
            {q(1), q(2), q(3), q_dot(1), q_dot(2), q_dot(3)}));
    else
        % If inputs are symbolic, return symbolic V_F
        V_F = V_F_symbolic;
    end
end
