function q_dot = inverse_velocity_kinematics_numeric(q, V_F)
    % This function calculates the inverse velocity kinematics numerically
    % Input:
    %   q   - [q1; q2; q3] joint angles in radians (numeric)
    %   V_F - [vx; vy; vz; wx; wy; wz] end-effector velocity vector (numeric)
    % Output:
    %   q_dot - [q1_dot; q2_dot; q3_dot] joint velocities (numeric)

    % Calculate the numerical Jacobian matrix using the joint angles
    J = jacobian_matrix(q(1), q(2), q(3));

    % Calculate the Moore-Penrose pseudo-inverse of the Jacobian
    % Using pinv() here, which directly computes the pseudo-inverse numerically
    J_inv = pinv(J);

    % Calculate the joint velocities
     q_dot = double(J_inv * V_F);
end
