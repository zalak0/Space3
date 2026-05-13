function [K_lqr, P0,  Q, R_inv] = lqr_controller(A, B, phi_max, rate_max, m_max)

Q = diag([1/phi_max^2,  1/rate_max^2, ...
          1/phi_max^2,  1/rate_max^2, ...
          1/phi_max^2,  1/rate_max^2]);

R     = diag([1/m_max^2, 1/m_max^2, 1/m_max^2]);
R_inv = diag(1./diag(R));

disp(Q)
disp(R_inv)
disp(R)

% Solve Riccati equation
[K_lqr, P0, ~] = lqr(A, B, Q, R);

disp(P0)

% Check new control authority
x_initial = [0.1*pi/180; 0; 0.1*pi/180; 0; 0.1*pi/180; 0];
Tc_new = -K_lqr * x_initial;
fprintf('New initial control torque:\n');
fprintf('  Max Tc = %.4e N·m\n', max(abs(Tc_new)));
fprintf('  Tc/Td ratio = %.1fx\n', max(abs(Tc_new))/3.024e-6);

end