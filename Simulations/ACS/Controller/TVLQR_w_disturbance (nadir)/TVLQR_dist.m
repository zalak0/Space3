clc
clear all;

%% -- Physical constants and inertia ------------------------------------
mu        = 3.986e14;
earth_rad = 6371000;
r         = earth_rad + 515000;
period    = (2*pi * r^(3/2)) / sqrt(mu);
omega     = (2*pi) / period;

mass = 2.0;  Lx = 0.226;  Ly = 0.100;  Lz = 0.100;
Ix = (1/12)*mass*(Ly^2+Lz^2);
Iy = (1/12)*mass*(Lx^2+Lz^2);
Iz = (1/12)*mass*(Lx^2+Ly^2);

A = [ 0,  1,  0,  0,  0,  0;
     -4*omega^2*(Iy-Iz)/Ix,  0,  0,  0,  0,  omega*(Ix+Iz-Iy)/Ix;
      0,  0,  0,  1,  0,  0;
      0,  0, -3*omega^2*(Ix-Iz)/Iy,  0,  0,  0;
      0,  0,  0,  0,  0,  1;
      0, -omega*(Iz+Ix-Iy)/Iz,  0,  0, -omega^2*(Iy-Ix)/Iz,  0];

B = [0,    0,    0;
     1/Ix, 0,    0;
     0,    0,    0;
     0,    1/Iy, 0;     % theta_ddot from u(2)
     0,    0,    0;
     0,    0,    1/Iz]; % psi_ddot from u(3)

C = eye(6);  D = zeros(6,3);

%% -- Discretise ---------------------------------------------------------
dt_ctrl = 0.1;   % control timestep [s] -- same as TVLQR.m
sys_c   = ss(A, B, C, D);
sys_d   = c2d(sys_c, dt_ctrl, 'zoh');
Ad = sys_d.A;  Bd = sys_d.B;  Cd = sys_d.C;  Dd = sys_d.D;

%% -- Cost weights -- define ONCE, use everywhere ------------------------
phi_max   = 5  * pi/180;
rate_max  = 0.5 * pi/180;
m_max_pss = 0.166;   % A*m^2 -- dipole hardware limit

Q = diag([1/phi_max^2,  1/rate_max^2, ...
          1/phi_max^2,  1/rate_max^2, ...
          1/phi_max^2,  1/rate_max^2]);

R_pss     = diag([1/m_max_pss^2, 1/m_max_pss^2, 1/m_max_pss^2]);
R_inv_pss = diag([m_max_pss^2,   m_max_pss^2,   m_max_pss^2]);

%% -- Warm-start P0 from standard LQR -----------------------------------
% Use torque-space B for warm-start only
[K_lqr, P0, ~] = lqr(A, B, Q, R_pss);

fprintf('P0 diagonal:\n');  disp(diag(P0)')

%% -- Load disturbances --------------------------------------------------
mag  = load('GOOSE_mag_Td.mat');
aero = load('GOOSE_aero_Td.mat');
srp  = load('GOOSE_srp_Td.mat');

[T_disturbance, Td_reordered, idx] = ext_disturbances(mag, aero, srp);

%% -- Magnetic field: ECI to LVLH ---------------------------------------
t_tv      = mag.t;
B_eci     = mag.B_eci;
r_eci_arr = srp.r_eci_m;
v_eci_arr = srp.v_eci_m;
N_tv      = length(t_tv);

B_lvlh = zeros(N_tv, 3);
for i = 1:N_tv
    B_lvlh(i,:) = eci_to_lvlh(B_eci(i,:), r_eci_arr(i,:), v_eci_arr(i,:));
end

%% -- Trim and repeat for Simulink --------------------------------------
t_tv_trimmed   = t_tv(idx);
B_lvlh_trimmed = B_lvlh(idx,:);
Td_trimmed     = Td_reordered(idx,:);

% Convert magnetic disturbance from sun-pointing body frame to the
% nadir/LVLH body frame used by the linearized x dynamics.
Td_mag_sp_trimmed = interp1(mag.t, mag.T_body_sp, t_tv_trimmed, 'linear', 'extrap');
Td_mag_np_trimmed = interp1(mag.t, mag.T_body_np, t_tv_trimmed, 'linear', 'extrap');
Td_trimmed = Td_trimmed - Td_mag_sp_trimmed + Td_mag_np_trimmed;

N_repeat  = 100;
dt_data   = t_tv(2) - t_tv(1);        % 10s -- data resolution
T_total   = length(t_tv_trimmed) * dt_data;

B_lvlh_rep = repmat(B_lvlh_trimmed, N_repeat, 1);
Td_rep     = repmat(Td_trimmed,     N_repeat, 1);

t_offsets = (0:N_repeat-1)' * T_total;
t_rep     = repmat(t_tv_trimmed, N_repeat, 1) + ...
            repelem(t_offsets, length(t_tv_trimmed));

B_lvlh_ts = timeseries(B_lvlh_rep, t_rep);
Td_ts     = timeseries(Td_rep,     t_rep);

%% -- Continuous-time periodic TVLQR on the simulation time grid ---------
% TVLQR.m used one constant Pss.  Here P_tv(:,:,k) changes with time, and
% the design grid is the same 0.1s grid used by the Euler simulation below.
dt_sim = 0.1;
N_orbit = round(T_total / dt_sim);
t_orbit = (0:N_orbit-1) * dt_sim;

B_lvlh_fine = interp1(t_tv_trimmed, B_lvlh_trimmed, t_orbit, ...
                      'linear', 'extrap');

B_tv = zeros(6,3,N_orbit);
for k = 1:N_orbit
    B_tv(:,:,k) = make_B_tv(B_lvlh_fine(k,:), Ix, Iy, Iz);
end

max_iter = 200;
tol = 1e-8;

P_end = P0;
P_tv = zeros(6,6,N_orbit);
K_tv = zeros(3,6,N_orbit);

for iter = 1:max_iter
    P = P_end;

    for k = N_orbit:-1:1
        B_tv_k = B_tv(:,:,k);
        G_k = B_tv_k * R_inv_pss * B_tv_k';

        % Backward Euler step for -Pdot = A'P + PA - PBR^-1B'P + Q.
        Pdot_backward = A' * P + P * A - P * G_k * P + Q;
        P = P + dt_sim * Pdot_backward;
        P = 0.5 * (P + P');
    end

    rel_err = norm(P - P_end, 'fro') / max(1, norm(P_end, 'fro'));
    P_end = P;

    if rel_err < tol
        break
    end
end

% Recompute and store one clean periodic P(t), K(t) table.
P = P_end;
P_tv(:,:,N_orbit) = P;
K_tv(:,:,N_orbit) = R_inv_pss * (B_tv(:,:,N_orbit)' * P);

for k = N_orbit-1:-1:1
    B_tv_k = B_tv(:,:,k+1);   % use B_tv at the "next" step
    G_k = B_tv_k * R_inv_pss * B_tv_k';
    Pdot_backward = A' * P + P * A - P * G_k * P + Q;
    P = P + dt_sim * Pdot_backward;
    P = 0.5 * (P + P');

    P_tv(:,:,k) = P;
    K_tv(:,:,k) = R_inv_pss * (B_tv(:,:,k)' * P);   % use B_tv at THIS step
end

fprintf('Periodic TVLQR iterations: %d\n', iter);
fprintf('Periodic TVLQR relative error: %.3e\n', rel_err);
fprintf('P_tv(:,:,1) diagonal:\n');  disp(diag(P_tv(:,:,1))')

%% -- Floquet stability check -------------------------------------------
Phi = eye(6);
for k = 1:N_orbit
    A_cl = A - B_tv(:,:,k) * K_tv(:,:,k);
    Phi = (eye(6) + A_cl * dt_sim) * Phi;
end

fprintf('TVLQR Floquet multipliers (Euler grid):\n');
disp(abs(eig(Phi)))
fprintf('TVLQR spectral radius: %.6f\n', max(abs(eig(Phi))));


%% -- Standalone linear simulation of the closed loop --------------------
x = [0; 1*pi/180; 0; 0; 0; 0];   % 20deg IC
T_sim = 2 * 5695;
N_steps = round(T_sim / dt_sim);
x_hist = zeros(6, N_steps);
u_hist = zeros(3, N_steps);
t_hist = (0:N_steps-1) * dt_sim;

for k = 1:N_steps
    t = t_hist(k);
    t_mod = mod(t, T_total);

    k_tv = floor(t_mod / dt_sim) + 1;
    k_tv = min(max(k_tv, 1), N_orbit);

    B_tv_k = B_tv(:,:,k_tv);
    K_k = K_tv(:,:,k_tv);

    u_nom = -K_k * x;
    beta = max(abs(u_nom) / m_max_pss);
    u_k = u_nom / max(beta, 1);

    % Plant: xdot = A*x + B_tv*u, same Euler update style as TVLQR.m
    Td_k = interp1(t_tv_trimmed, Td_trimmed, t_mod, 'linear', 'extrap')';
    xdot = A*x + B_tv_k * u_k + B * Td_k;
    x = x + xdot * dt_sim;

    x_hist(:,k) = x;
    u_hist(:,k) = u_k;
end

figure;
subplot(1,1,1);
plot(t_hist/period, x_hist([2,4,6],:)' * 180/pi);
% plot(t_hist/period, x_hist([1,3,5],:)' * 180/pi);
xlabel('orbits');
xlim([0, 10]);
ylabel('angular rate [deg/s]'); 
legend('\phi','\theta','\psi'); 
grid on;
% subplot(2,1,2);
% plot(t_hist/period, u_hist');
% ylabel('u [A*m^2]'); xlabel('orbits'); legend('m_x','m_y','m_z'); grid on;

fprintf('Initial angle norm: %.4f deg\n', norm(x_hist([2,4,6], 1)) * 180/pi);
fprintf('Final angle norm:   %.4f deg\n', norm(x_hist([2,4,6], end)) * 180/pi);
fprintf('Max |u| used:       %.4e A*m^2 (limit %.4e)\n', ...
    max(abs(u_hist(:))), m_max_pss);


% Check K_tv is well-formed
fprintf('K_tv size: %s\n', mat2str(size(K_tv)));
fprintf('K_tv(:,:,1) sample:\n'); disp(K_tv(:,:,1));
fprintf('K_tv(:,:,end) sample:\n'); disp(K_tv(:,:,end));
fprintf('Max |K_tv|: %.3e\n', max(abs(K_tv(:))));

% Compare to a few hand-computed Ks
for k_test = [1, 1000, 10000, 50000, 8e3]
    K_check = R_inv_pss * (B_tv(:,:,k_test)' * P_tv(:,:,k_test));
    K_stored = K_tv(:,:,k_test);
    fprintf('k=%d: error = %.3e\n', k_test, norm(K_check - K_stored));
end

function B_tv_k = make_B_tv(B_vec, Ix, Iy, Iz)
    Bx = B_vec(1);
    By = B_vec(2);
    Bz = B_vec(3);

    B_cross_k = [ 0,   Bz, -By;
                 -Bz,  0,   Bx;
                  By, -Bx,  0];

    B_tv_k = zeros(6,3);
    B_tv_k(2,:) = B_cross_k(1,:) / Ix;
    B_tv_k(4,:) = B_cross_k(2,:) / Iy;
    B_tv_k(6,:) = B_cross_k(3,:) / Iz;
end

% clc; clear all;
% 
% % -- Physical constants and inertia ------------------------------------
% mu        = 3.986e14;
% earth_rad = 6371000;
% r         = earth_rad + 515000;
% period    = (2*pi * r^(3/2)) / sqrt(mu);
% omega     = (2*pi) / period;
% 
% mass = 2.0;  Lx = 0.226;  Ly = 0.100;  Lz = 0.100;
% Ix = (1/12)*mass*(Ly^2+Lz^2);
% Iy = (1/12)*mass*(Lx^2+Lz^2);
% Iz = (1/12)*mass*(Lx^2+Ly^2);
% 
% A = [ 0,  1,  0,  0,  0,  0;
%      -4*omega^2*(Iy-Iz)/Ix,  0,  0,  0,  0,  omega*(Ix+Iz-Iy)/Ix;
%       0,  0,  0,  1,  0,  0;
%       0,  0, -3*omega^2*(Ix-Iz)/Iy,  0,  0,  0;
%       0,  0,  0,  0,  0,  1;
%       0, -omega*(Iz+Ix-Iy)/Iz,  0,  0, -omega^2*(Iy-Ix)/Iz,  0];
% 
% B = [0,    0,    0;
%      1/Ix, 0,    0;
%      0,    0,    0;
%      0,    1/Iy, 0;
%      0,    0,    0;
%      0,    0,    1/Iz];
% 
% C = eye(6);  D = zeros(6,3);
% 
% % -- Cost weights ------------------------------------------------------
% Set these to your VALIDATED APLQR config to reproduce rho ~ 0.15.
% phi_max   = 30 * pi/180;     % <-- validated (benchmark file had 5 deg)
% rate_max  = 0.5 * pi/180;
% m_max_des = 5e-5;            % <-- DESIGN dipole weight, decoupled from HW
% m_max_hw  = 0.166;           % hardware saturation -- used ONLY in saturator
% 
% Q = diag([1/phi_max^2, 1/rate_max^2, 1/phi_max^2, 1/rate_max^2, ...
%           1/phi_max^2, 1/rate_max^2]);
% R_inv_pss = diag([m_max_des^2, m_max_des^2, m_max_des^2]);
% 
% % -- Load disturbances --------------------------------------------------
% mag  = load('GOOSE_mag_Td.mat');
% aero = load('GOOSE_aero_Td.mat');
% srp  = load('GOOSE_srp_Td.mat');
% [T_disturbance, Td_reordered, idx] = ext_disturbances(mag, aero, srp);
% 
% t_tv = mag.t;  B_eci = mag.B_eci;
% r_eci_arr = srp.r_eci_m;  v_eci_arr = srp.v_eci_m;
% N_tv = length(t_tv);
% B_lvlh = zeros(N_tv,3);
% for i = 1:N_tv
%     B_lvlh(i,:) = eci_to_lvlh(B_eci(i,:), r_eci_arr(i,:), v_eci_arr(i,:));
% end
% 
% t_tv_trimmed   = t_tv(idx);
% B_lvlh_trimmed = B_lvlh(idx,:);
% Td_trimmed     = Td_reordered(idx,:);
% Td_mag_sp = interp1(mag.t, mag.T_body_sp, t_tv_trimmed, 'linear','extrap');
% Td_mag_np = interp1(mag.t, mag.T_body_np, t_tv_trimmed, 'linear','extrap');
% Td_trimmed = Td_trimmed - Td_mag_sp + Td_mag_np;
% 
% dt_data = t_tv(2) - t_tv(1);
% T_total = length(t_tv_trimmed) * dt_data;
% 
% % -- Periodic B_tv table over one orbit (fine grid) --------------------
% dt_sim  = 0.1;
% N_orbit = round(T_total / dt_sim);
% t_orbit = (0:N_orbit-1) * dt_sim;
% B_lvlh_fine = interp1(t_tv_trimmed, B_lvlh_trimmed, t_orbit, 'linear','extrap');
% 
% B_tv = zeros(6,3,N_orbit);
% for k = 1:N_orbit
%     B_tv(:,:,k) = make_B_tv(B_lvlh_fine(k,:), Ix, Iy, Iz);
% end
% 
% % ==== CHANGED: constant Pss via orbit-averaged Riccati ===============
% Instantaneous B_tv*R^-1*B_tv' is rank-2 (magnetic underactuation);
% averaged over one orbit it is full-rank on the rate subspace.
% G_avg = zeros(6,6);
% for k = 1:N_orbit
%     G_avg = G_avg + B_tv(:,:,k) * R_inv_pss * B_tv(:,:,k)';
% end
% G_avg = G_avg / N_orbit;
% 
% Factor G_avg = Bbar*Bbar' so icare treats it as the effectiveness term.
% [V,Dg] = eig(0.5*(G_avg + G_avg'));
% Bbar   = V * diag(sqrt(max(real(diag(Dg)),0)));
% 
% Solve  A'Pss + Pss A - Pss G_avg Pss + Q = 0   (R = I, folded into Bbar)
% [Pss, ~, ~] = icare(A, Bbar, Q, eye(6));
% 
% fprintf('Pss diagonal:\n');            disp(diag(Pss)');
% fprintf('Pss asymmetry: %.3e\n', norm(Pss - Pss','fro'));
% 
% % -- Floquet check using the ONLINE law (constant Pss) ----------------
% Phi = eye(6);
% for k = 1:N_orbit
%     K_k  = R_inv_pss * (B_tv(:,:,k)' * Pss);   % same expression as flight C
%     A_cl = A - B_tv(:,:,k) * K_k;
%     Phi  = (eye(6) + A_cl * dt_sim) * Phi;
% end
% fprintf('APLQR spectral radius (constant Pss): %.6f\n', max(abs(eig(Phi))));
% 
% % -- Closed-loop simulation, K built online from Pss ------------------
% x = [60*pi/180; 0; 60*pi/180; 0; 60*pi/180; 0];
% T_sim = 20 * 5695;
% N_steps = round(T_sim / dt_sim);
% x_hist = zeros(6,N_steps);  u_hist = zeros(3,N_steps);
% t_hist = (0:N_steps-1) * dt_sim;
% 
% for k = 1:N_steps
%     t_mod = mod(t_hist(k), T_total);
%     k_tv  = min(max(floor(t_mod/dt_sim)+1, 1), N_orbit);
% 
%     B_tv_k = B_tv(:,:,k_tv);
%     K_k    = R_inv_pss * (B_tv_k' * Pss);   % <-- from constant Pss, not a table
% 
%     u_nom = -K_k * x;
%     beta  = max(abs(u_nom) / m_max_hw);     % <-- saturate at HW limit
%     u_k   = u_nom / max(beta, 1);
% 
%     Td_k  = interp1(t_tv_trimmed, Td_trimmed, t_mod, 'linear','extrap')';
%     xdot  = A*x + B_tv_k*u_k + B*Td_k;
%     x     = x + xdot*dt_sim;
% 
%     x_hist(:,k) = x;  u_hist(:,k) = u_k;
% end
% 
% figure;
% subplot(2,1,1); plot(t_hist/5695, x_hist([1,3,5],:)'*180/pi);
% ylabel('angle [deg]'); legend('\phi','\theta','\psi'); grid on;
% subplot(2,1,2); plot(t_hist/5695, u_hist');
% ylabel('u [A*m^2]'); xlabel('orbits'); legend('m_x','m_y','m_z'); grid on;
% 
% fprintf('Final angle norm: %.4f deg\n', norm(x_hist([1,3,5],end))*180/pi);
% fprintf('Max |u|: %.4e (limit %.4e)\n', max(abs(u_hist(:))), m_max_hw);
% 
% % -- Export Pss (+ Rinv, inertias) to a C header ----------------------
% fid = fopen('pss.h','w');
% fprintf(fid, '/* APLQR steady-state cost-to-go. Generated %s */\n', datestr(now));
% fprintf(fid, '/* phi_max=%.1f deg  rate_max=%.3f deg/s  m_max_des=%.1e */\n', ...
%         phi_max*180/pi, rate_max*180/pi, m_max_des);
% fprintf(fid, 'static const float Pss[6][6] = {\n');
% for i = 1:6
%     fprintf(fid, '  {% .9ef,% .9ef,% .9ef,% .9ef,% .9ef,% .9ef},\n', Pss(i,:));
% end
% fprintf(fid, '};\n');
% fprintf(fid, 'static const float Rinv_diag[3] = {% .9ef,% .9ef,% .9ef};\n', diag(R_inv_pss));
% fprintf(fid, 'static const float I_diag[3]    = {% .9ef,% .9ef,% .9ef};\n', [Ix Iy Iz]);
% fclose(fid);
% 
% function B_tv_k = make_B_tv(B_vec, Ix, Iy, Iz)
%     Bx=B_vec(1); By=B_vec(2); Bz=B_vec(3);
%     B_cross_k = [ 0, Bz,-By; -Bz, 0, Bx;  By,-Bx, 0];
%     B_tv_k = zeros(6,3);
%     B_tv_k(2,:) = B_cross_k(1,:)/Ix;
%     B_tv_k(4,:) = B_cross_k(2,:)/Iy;
%     B_tv_k(6,:) = B_cross_k(3,:)/Iz;
% end