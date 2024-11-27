function q_dot = inverse_velocity_kinematics_symbolic(q, V_F)
    % This function calculates the inverse velocity kinematics symbolically
    % Input:
    % q - [q1; q2; q3] joint angles in radians (symbolic)
    % V_F - [vx; vy; vz; wx; wy; wz] end-effector velocity vector (symbolic)

    % Define symbolic variables for joint angles
    syms q1 q2 q3
    J = jacobian_matrix(q1, q2, q3);

    JT = transpose(J);
    
    % Moore-Penrose pseudo-inverse formula
    lol=JT * J;
    J_inv = simplify(inv(lol) * JT);

    % Calculate the joint velocities
    q_dot = simplify(J_inv * V_F);

    % Substitute symbolic joint angles q1, q2, q3 with the input values
    q_dot = subs(q_dot, {q1, q2, q3}, {q(1), q(2), q(3)});
end
