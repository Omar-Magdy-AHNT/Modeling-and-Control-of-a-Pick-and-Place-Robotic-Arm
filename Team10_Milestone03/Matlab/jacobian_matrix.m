
function J = jacobian_matrix(q1, q2, q3)
syms q1_sym q2_sym q3_sym

    % Define transformation matrices
    A1 = [cos(q1_sym) 0 sin(q1_sym) 0.013*cos(q1_sym);
          sin(q1_sym) 0 -cos(q1_sym) 0.013*sin(q1_sym);
          0 -1 0 0.04;
          0 0 0 1];

    A2 = [cos(q2_sym) -sin(q2_sym) 0 -0.12*cos(q2_sym);
          sin(q2_sym) cos(q2_sym) 0 -0.12*sin(q2_sym);
          0 0 1 0;
          0 0 0 1];

    A3 = [cos(q3_sym) -sin(q3_sym) 0 0.09*cos(q3_sym);
          sin(q3_sym) cos(q3_sym) 0 0.09*sin(q3_sym);
          0 0 1 0;
          0 0 0 1];
      
    A4 = [1 0 0 0.1;
          0 1 0 0;
          0 0 1 -0.03;
          0 0 0 1];

    % Overall transformation matrix
    T = simplify(A1 * A2 * A3 * A4);

    % Extract end-effector position (Px, Py, Pz)
    Py = T(1,4);
    Px = T(2,4);
    Pz = T(3,4);
    Px=Px*-1;
    Py=Py*1;
    % Compute partial derivatives of end-effector position with respect to each angle
    Jv1 = diff([Px; Py; Pz], q1_sym);
    Jv2 = diff([Px; Py; Pz], q2_sym);
    Jv3 = diff([Px; Py; Pz], q3_sym);

    % Linear velocity Jacobian (top 3 rows of J)
    Jv = [Jv1, Jv2, Jv3];

    % Compute rotation part of the Jacobian (for angular velocity)
    % Z axes for each joint's rotation axis in the base frame
    z0 = [0; 0; 1];
    z1 = A1(1:3, 3); % z-axis after first joint

     A12 = A1 * A2;
     z2 = A12(1:3, 3); % z-axis after second joint

    % Origin positions for each frame (relative to the base)
    o0 = [0; 0; 0];
    o1 = A1(1:3, 4);
    o2 = A12(1:3, 4);
    o3 = T(1:3, 4); % end-effector position

    % Angular velocity Jacobian (bottom 3 rows of J)
    Jw1 = z0;
    Jw2 = z1;
    Jw3 = z2;

    % Combine linear and angular Jacobians
    J = [Jv; Jw1, Jw2, Jw3];

    % Substitute the actual angles into the Jacobian if needed
    J = simplify(subs(J, {q1_sym, q2_sym, q3_sym}, {q1, q2, q3}));

end
