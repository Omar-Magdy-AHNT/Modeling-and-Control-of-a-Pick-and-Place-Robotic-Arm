function Joint_Space = joint_traj(q0, qf, qdot0, qdotf, Tf, Ts)
    % Joint_Space calculates the joint trajectory for each time step
    % using cubic polynomial interpolation.
    %
    % INPUTS:
    % q0     - Initial joint angles as a vector [rad]
    % qf     - Final joint angles as a vector [rad]
    % qdot0  - Initial angular velocities as a vector [rad/s]
    % qdotf  - Final angular velocities as a vector [rad/s]
    % Tf     - Total duration of the trajectory [s]
    % Ts     - Sampling time [s]
    %
    % OUTPUT:
    % Joint_Space - Matrix of joint positions at each time step. Each row corresponds
    %               to a time step, and each column corresponds to a joint.

    % Define time vector
    t = 0:Ts:Tf;

    % Number of joints
    nJoints = length(q0);

    % Preallocate for joint positions
    Joint_Space = zeros(length(t), 6);

    % Coefficients for cubic polynomial interpolation
    for i = 1:nJoints
        % Solve for coefficients of the cubic polynomial:
        % q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        % Using boundary conditions:
        % q(0) = q0, q(Tf) = qf, qdot(0) = qdot0, qdot(Tf) = qdotf
        A = [1, 0,    0,       0;  % q(0) = q0
             1, Tf,   Tf^2,    Tf^3; % q(Tf) = qf
             0, 1,    0,       0;  % qdot(0) = qdot0
             0, 1,    2*Tf,    3*Tf^2]; % qdot(Tf) = qdotf
        b = [q0(i); qf(i); qdot0(i); qdotf(i)];
        coeffs = A\b; % Solve for coefficients [a0; a1; a2; a3]

        % Generate trajectory for this joint
       
        for j = 1:length(t)
            Joint_Space(j, i) = coeffs(1) + coeffs(2)*t(j) + coeffs(3)*t(j)^2 + coeffs(4)*t(j)^3;

               [T, pxx, pyy, pzz] = EEF_Pos_and_DH(Joint_Space(j, 1), Joint_Space(j, 2), Joint_Space(j, 3));
               Joint_Space(j, 4)=pxx;
               Joint_Space(j, 5)=pyy;
               Joint_Space(j, 6)=pzz;

        end
    end
end
