function V_F = forward_velocity_kinematics_numeric(q, q_dot)
    % This function calculates the forward velocity kinematics numerically
    % Input:
    % q - [q1; q2; q3] joint angles in radians (numeric)
    % q_dot - [q1_dot; q2_dot; q3_dot] joint angular velocities (numeric)

    % Define symbolic variables for joint angles and their velocities
    syms q1_sym q2_sym q3_sym q1_dot q2_dot q3_dot

    % Obtain the symbolic Jacobian matrix
    J = jacobian_matrix(q1_sym, q2_sym, q3_sym);

    % Define symbolic joint velocity vector
    q_dot_sym = [q1_dot; q2_dot; q3_dot];

    % Symbolic forward velocity kinematics
    V_F_symbolic = simplify(J * q_dot_sym);

    % Substitute numerical values for q and q_dot
    V_F = double(subs(V_F_symbolic, ...
        {q1_sym, q2_sym, q3_sym, q1_dot, q2_dot, q3_dot}, ...
        {q(1), q(2), q(3), q_dot(1), q_dot(2), q_dot(3)}));
end
