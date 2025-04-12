function q_dot = Inverse_Velocity_Kinematics_Function(q, V_F)
    % Calculate the numerical Jacobian matrix using the joint angles
    J = jacobian_matrix(q(1), q(2), q(3));

    % Calculate the Moore-Penrose pseudo-inverse of the Jacobian
    % Using pinv() here, which directly computes the pseudo-inverse numerically
    J_inv = pinv(J);

    % Calculate the joint velocities
     q_dot = double(J_inv * V_F);
end
