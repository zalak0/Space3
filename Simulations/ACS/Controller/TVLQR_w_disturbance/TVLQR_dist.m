
clc
clear all

%% ── Physical constants and inertia ───────────────────────────────────
mu        = 3.986e14;
earth_rad = 6371000;
r         = earth_rad + 515000;
period    = (2*pi * r^(3/2)) / sqrt(mu);
omega     = (2*pi) / period;

% mass = 2.0;  Lx = 0.226;  Ly = 0.100;  Lz = 0.100;
% Ix = (1/12)*mass*(Ly^2+Lz^2);
% Iy = (1/12)*mass*(Lx^2+Lz^2);
% Iz = (1/12)*mass*(Lx^2+Ly^2);

Ix = 8.7;
Iy = 10;
Iz = 6.5;

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

%% ── Discretise ───────────────────────────────────────────────────────
dt_ctrl = 1;   % control timestep [s] — never overwrite this
sys_c   = ss(A, B, C, D);
sys_d   = c2d(sys_c, dt_ctrl, 'zoh');
Ad = sys_d.A;  Bd = sys_d.B;  Cd = sys_d.C;  Dd = sys_d.D;

%% ── Cost weights — define ONCE, use everywhere ────────────────────────
%% LQR / TVLQR weights — dipole-based control
% phi_max  = 30 * pi/180;     % 30° — loose target, not pointing spec
% rate_max = 1  * pi/180;     % 1°/s
% m_max    = 0.02;            % 10× smaller than hardware → expensive control

% phi_max   = 8165  * pi/180;
% rate_max  = 10 * pi/180;
% m_max = 0.02;   % A*m^2 -- dipole hardware limit
% 
% Q     = diag([1/phi_max^2, 1/rate_max^2, ...
%               1/phi_max^2, 1/rate_max^2, ...
%               1/phi_max^2, 1/rate_max^2])
% R     = diag([1/m_max^2, 1/m_max^2, 1/m_max^2])
% R_inv = diag(1./diag(R));

Q = diag([0.1, 0.1, ...    % phi, phi_dot
          1.0, 1.0, ...    % theta, theta_dot (pitch — Psiaki weights it 10× heavier)
          0.1, 0.1]);      % psi, psi_dot

R = (6.2e7) * eye(3);   % Psiaki's literal value
R_inv = diag(1./diag(R));

%% ── Warm-start P0 from standard LQR ──────────────────────────────────
% Use torque-space B for warm-start only
[K_lqr, P0, ~] = lqr(A, B, Q, R);

fprintf('P0 diagonal:\n');  disp(diag(P0)')

%% ── Load disturbances ────────────────────────────────────────────────
mag  = load('GOOSE_mag_Td.mat');
aero = load('GOOSE_aero_Td.mat');
srp  = load('GOOSE_srp_Td.mat');

[T_disturbance, Td_reordered, idx] = ext_disturbances(mag, aero, srp);
%% ── Magnetic field: ECI → LVLH ───────────────────────────────────────
t_tv      = mag.t;
B_eci     = mag.B_eci;
r_eci_arr = srp.r_eci_m;
v_eci_arr = srp.v_eci_m;
N_tv      = length(t_tv);

B_lvlh = zeros(N_tv, 3);

for i = 1:N_tv
    B_lvlh(i,:) = eci_to_lvlh(B_eci(i,:), r_eci_arr(i,:), v_eci_arr(i,:));
end

%% ── Trim and repeat for Simulink ─────────────────────────────────────
t_tv_trimmed   = t_tv(idx);
B_lvlh_trimmed = B_lvlh(idx,:);
Td_trimmed     = Td_reordered(idx,:);

N_repeat  = 1000;
dt_data   = t_tv(2) - t_tv(1);        % 10s — data resolution
T_total   = length(t_tv_trimmed) * dt_data;

B_lvlh_rep = repmat(B_lvlh_trimmed, N_repeat, 1);
Td_rep     = repmat(Td_trimmed,     N_repeat, 1);

t_offsets = (0:N_repeat-1)' * T_total;
t_rep     = repmat(t_tv_trimmed, N_repeat, 1) + ...
            repelem(t_offsets, length(t_tv_trimmed));

B_lvlh_ts = timeseries(B_lvlh_rep, t_rep);
Td_ts     = timeseries(Td_rep,     t_rep);


% Test 1: Mean of disturbance over one orbit
Td_orbit = Td_trimmed(1:569, :);   % one orbit
fprintf('Mean Td per axis [N.m]: %.3e %.3e %.3e\n', mean(Td_orbit));
fprintf('Peak Td per axis [N.m]: %.3e %.3e %.3e\n', max(abs(Td_orbit)));

% Test 2: What torque can magnetorquer produce against this Td?
% At each timestep, what's the *maximum* torque achievable along Td direction?
B_mag = vecnorm(B_lvlh_trimmed, 2, 2);     % |B| at each step
m_max_hw = 0.2;
T_max_perp = m_max_hw * B_mag;              % max torque perpendicular to B
fprintf('Max achievable |T_ctrl|: %.3e N.m\n', max(T_max_perp));
fprintf('Min achievable |T_ctrl|: %.3e N.m\n', min(T_max_perp));

% Test 3: Component of Td ALONG B (the unrejectable part)
Td_along_B = zeros(569,1);
for k = 1:569
    b_hat = B_lvlh_trimmed(k,:) / norm(B_lvlh_trimmed(k,:));
    Td_along_B(k) = abs(dot(Td_orbit(k,:), b_hat));
end
fprintf('Mean |Td . B_hat|: %.3e N.m   <-- this is what you CAN''T reject\n', ...
        mean(Td_along_B));
fprintf('Peak |Td . B_hat|: %.3e N.m\n', max(Td_along_B));

%% ── Time-averaged Pss via averaged ARE ───────────────────────────────
% Step 1: Build B_tv(t) over one orbital period using your real B_lvlh data
%         But B_tv needs B_body, not B_lvlh — and B_body depends on attitude,
%         which we don't have yet. So we make the linearisation assumption:
%         around nominal pointing, B_body ≈ B_lvlh (since attitude error is small).
%         This is exactly what Psiaki does.

N_orbit = length(t_tv_trimmed);   % samples in one orbit
G_sum = zeros(6,6);

for k = 1:N_orbit
    Bx = B_lvlh_trimmed(k,1);
    By = B_lvlh_trimmed(k,2);
    Bz = B_lvlh_trimmed(k,3);
    
    B_cross_k = [  0,   Bz, -By;
                  -Bz,   0,  Bx;
                   By, -Bx,   0];
    
    B_tv_k = zeros(6,3);
    B_tv_k(2,:) = B_cross_k(1,:) / Ix;
    B_tv_k(4,:) = B_cross_k(2,:) / Iy;
    B_tv_k(6,:) = B_cross_k(3,:) / Iz;
    
    G_sum = G_sum + B_tv_k * R_inv * B_tv_k';
end

G_avg = G_sum / N_orbit;   % time-averaged B*R^-1*B'

[V, D] = eig(G_avg);
d = diag(D);
% Clip tiny/negative eigenvalues from numerical noise
d(d < max(d)*1e-10) = max(d)*1e-10;
L = V * diag(sqrt(d));   % G_avg ≈ L*L'
Pss = care(A, L, Q, eye(6));

fprintf('Constant-B Pss via lqr():\n');
fprintf('  Positive definite: %d\n', all(eig(Pss) > 0));
fprintf('  Pss diagonal:\n');  disp(diag(Pss)')

A_avg_cl = A - 1.34 * G_avg * Pss;
fprintf('Averaged closed-loop A_avg_cl eigenvalues:\n');
disp(eig(A_avg_cl))
fprintf('Max real: %.6e\n', max(real(eig(A_avg_cl))));

%% ── Floquet stability check ─────────────────────────────────────────
% Define an anonymous function for rho(alpha)
rho_fn = @(a0) compute_floquet_rho(a0, A, R_inv, Pss, B_lvlh_trimmed, ...
                                    Ix, Iy, Iz, dt_data, N_orbit);

% Search for optimum in a known-good range
[alpha_opt, rho_opt] = fminbnd(rho_fn, 0.01, 10000);

fprintf('Optimal alpha0 = %.4f, rho = %.4f\n', alpha_opt, rho_opt);

% Helper function (put at end of script or in separate file)
function rho = compute_floquet_rho(a0, A, R_inv, Pss, B_lvlh_trimmed, ...
                                    Ix, Iy, Iz, dt_data, N_orbit)
    Phi_orbit = eye(6);
    for k = 1:N_orbit
        bx = B_lvlh_trimmed(k,1); by = B_lvlh_trimmed(k,2); bz = B_lvlh_trimmed(k,3);
        Bc = [0,bz,-by; -bz,0,bx; by,-bx,0];
        Btv_k = zeros(6,3);
        Btv_k(2,:) = Bc(1,:)/Ix;
        Btv_k(4,:) = Bc(2,:)/Iy;
        Btv_k(6,:) = Bc(3,:)/Iz;
        K_k  = a0 * R_inv * (Btv_k' * Pss);
        A_cl = A - Btv_k * K_k;
        Phi_orbit = expm(A_cl * dt_data) * Phi_orbit;
    end
    rho = max(abs(eig(Phi_orbit)));
end

%% ─── Integrator augmentation ──────────────────────────────────────────
% Pick which states to integrate: φ, θ, ψ (rows 1, 3, 5 of x)
I_aug_sel = [1 0 0 0 0 0;
             0 0 1 0 0 0;
             0 0 0 0 1 0];        % 3×6

A_aug = [zeros(3,3), I_aug_sel;
         zeros(6,3), A];           % 9×9

% Integrator weight — this is your new tuning knob.
% Goal: integrator time constant ~5-15 orbits (slow, gentle bias rejection)
q_int_per_axis = 1.5e-8;             % start here

Q_aug = blkdiag(diag([q_int_per_axis, q_int_per_axis * 10, q_int_per_axis]), Q);   % 9×9

% Build L_aug so G_avg_aug = L_aug · L_augᵀ (top 3 rows zero — integrators have no control input)
L_aug = [zeros(3, size(L,2));
         L];                        % 9×6

Pss_aug = care(A_aug, L_aug, Q_aug, eye(size(L,2)));
fprintf('Augmented Pss eigenvalues:\n'); disp(eig(Pss_aug)')

% Extract the bottom 6×9 block (only this part multiplies B_tvᵀ in the gain)
Pss_aug_bot = Pss_aug(4:9, :);    % 6×9

%% Floquet sweep — augmented
rho_fn_aug = @(a0) compute_floquet_rho_aug(a0, A_aug, R_inv, Pss_aug_bot, ...
                                            B_lvlh_trimmed, Ix, Iy, Iz, dt_data, N_orbit);
[alpha_opt_aug, rho_opt_aug] = fminbnd(rho_fn_aug, 0.01, 10000);
fprintf('Augmented alpha_opt = %.4f, rho = %.4f\n', alpha_opt_aug, rho_opt_aug);

function rho = compute_floquet_rho_aug(a0, A_aug, R_inv, Pss_aug_bot, B_lvlh_trimmed, Ix, Iy, Iz, dt_data, N_orbit)
    Phi_orbit = eye(9);
    for k = 1:N_orbit
        bx = B_lvlh_trimmed(k,1); by = B_lvlh_trimmed(k,2); bz = B_lvlh_trimmed(k,3);
        Bc = [0,bz,-by; -bz,0,bx; by,-bx,0];
        Btv_k = zeros(6,3);
        Btv_k(2,:) = Bc(1,:)/Ix;
        Btv_k(4,:) = Bc(2,:)/Iy;
        Btv_k(6,:) = Bc(3,:)/Iz;
        Btv_aug_k = [zeros(3,3); Btv_k];                  % 9×3
        K_aug_k   = a0 * R_inv * (Btv_k' * Pss_aug_bot);   % 3×9
        A_cl_aug  = A_aug - Btv_aug_k * K_aug_k;
        Phi_orbit = expm(A_cl_aug * dt_data) * Phi_orbit;
    end
    rho = max(abs(eig(Phi_orbit)));
end

% Print K split into integrator/angle/rate columns
bx = mean(B_lvlh_trimmed(:,1)); by = mean(B_lvlh_trimmed(:,2)); bz = mean(B_lvlh_trimmed(:,3));
Bc = [0,bz,-by; -bz,0,bx; by,-bx,0];
Btv = zeros(6,3);
Btv(2,:) = Bc(1,:)/Ix; Btv(4,:) = Bc(2,:)/Iy; Btv(6,:) = Bc(3,:)/Iz;

K_aug_inspect = alpha_opt_aug * R_inv * (Btv' * Pss_aug_bot);   % 3×9

K_int   = K_aug_inspect(:, 1:3);
K_ang   = K_aug_inspect(:, [4 6 8]);
K_rate  = K_aug_inspect(:, [5 7 9]);

fprintf('Max |K_int|:  %.4e\n', max(abs(K_int(:))));
fprintf('Max |K_ang|:  %.4e\n', max(abs(K_ang(:))));
fprintf('Max |K_rate|: %.4e\n', max(abs(K_rate(:))));


% %% ─── Disturbance-aware closed-loop simulation ──────────────────────────
% % Empirical "Floquet with disturbance" — simulates linear closed loop with
% % actual Td(t) and measures steady-state RMS pointing.
% 
% fprintf('Td_trimmed sample row: [%.6e, %.6e, %.6e]\n', Td_trimmed(100,1), Td_trimmed(100,2), Td_trimmed(100,3));
% fprintf('Max abs Td: %.6e\n', max(abs(Td_trimmed(:))));
% 
% function [rms_deg, peak_deg, xaug_hist, t_hist] = sim_closed_loop_aug_with_disturbance( ...
%     Ad_aug, Bd_aug, R_inv, Pss_aug_bot, alpha0, ...
%     B_lvlh_trimmed, Td_trimmed, t_tv_trimmed, ...
%     Ix, Iy, Iz, N_orbits_sim)
% 
%     N_per_orbit = length(t_tv_trimmed);
%     dt_data     = t_tv_trimmed(2) - t_tv_trimmed(1);
%     N_steps     = N_orbits_sim * N_per_orbit;
% 
%     % Initial condition: z = 0, x = 5° on each angle
%     x_aug = [0; 0; 0;
%              5*pi/180; 0;
%              5*pi/180; 0;
%              5*pi/180; 0];
% 
%     xaug_hist = zeros(9, N_steps);
%     t_hist    = (0:N_steps-1) * dt_data;
% 
%     for k = 1:N_steps
%         idx = mod(k-1, N_per_orbit) + 1;
%         bx = B_lvlh_trimmed(idx,1);
%         by = B_lvlh_trimmed(idx,2);
%         bz = B_lvlh_trimmed(idx,3);
% 
%         Bc = [0,bz,-by; -bz,0,bx; by,-bx,0];
% 
%         Btv_k = zeros(6,3);
%         Btv_k(2,:) = Bc(1,:)/Ix;
%         Btv_k(4,:) = Bc(2,:)/Iy;
%         Btv_k(6,:) = Bc(3,:)/Iz;
% 
%         % 3×9 augmented gain
%         K_aug_k    = alpha0 * R_inv * (Btv_k' * Pss_aug_bot);
% 
%         u_dipole   = -K_aug_k * x_aug;
%         u_dipole   = max(-0.2, min(0.2, u_dipole));
% 
%         T_ctrl     = Bc * u_dipole;
%         T_dist     = Td_trimmed(idx,:)';
%         u_eff      = T_ctrl + T_dist;
% 
%         x_aug = Ad_aug * x_aug + Bd_aug * u_eff;
%         xaug_hist(:,k) = x_aug;
%     end
% 
%     % Steady-state metrics over last half
%     settle_idx = round(0.5*N_steps):N_steps;
%     angles_deg = xaug_hist([4 6 8], settle_idx) * 180/pi;   % attitude lives in 4,6,8
%     rms_deg    = sqrt(mean(angles_deg.^2, 2));
%     peak_deg   = max(abs(angles_deg), [], 2);
% 
%     fprintf('alpha0=%.3f | RMS=[%.2f %.2f %.2f] deg | peak=[%.2f %.2f %.2f] deg\n', ...
%         alpha0, rms_deg(1), rms_deg(2), rms_deg(3), peak_deg(1), peak_deg(2), peak_deg(3));
% end
% 
% %% ─── Augmented sweep ──────────────────────────────────────────────────
% % Pre-compute things that don't depend on (phi_max, m_max)
% I_aug_sel = [1 0 0 0 0 0;
%              0 0 1 0 0 0;
%              0 0 0 0 1 0];
% 
% A_aug = [zeros(3,3), I_aug_sel;
%          zeros(6,3), A];                              % 9×9
% 
% B_torque = zeros(6,3);
% B_torque(2,1) = 1/Ix;
% B_torque(4,2) = 1/Iy;
% B_torque(6,3) = 1/Iz;
% 
% B_torque_aug = [zeros(3,3); B_torque];                % 9×3
% 
% % Pre-compute ZOH discretisation of the augmented plant (constant!)
% M_aug = expm([A_aug, B_torque_aug; zeros(3,12)] * dt_data);
% Ad_aug = M_aug(1:9, 1:9);
% Bd_aug = M_aug(1:9, 10:12);
% 
% % L is the same 6-column factor of G_avg — reuse from above
% L_aug = [zeros(3, size(L,2));
%          L];                                          % 9×6
% 
% q_int_per_axis = 1e-8;     % integrator weight — sweep separately later
% 
% 
% phi_max_grid = [30, 100, 120, 360, 720, 1e4, 5e4, 1e5, 1e6, 4.2e7] * pi/180;
% m_max_grid   = [1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 5e-3, 0.01, 0.1, 0.2];
% rate_max_grid = [1e-8, 1e-7, 1e-6, 1e-4, 1e-3, 1e-2, 1e-1, 1, 10, 100];
% q_int_grid = [1e-8, 1e-7, 1e-6, 1e-4, 1e-3, 1e-2, 1e-1, 1, 10, 100];
% 
% results_aug = [];
% 
% for rate_max = rate_max_grid
%     for phi_max = phi_max_grid
%         for m_max = m_max_grid
%             for q_int = q_int_grid
%                 Q_loc      = diag([1/phi_max^2, 1/rate_max^2, ...
%                                    1/phi_max^2, 1/rate_max^2, ...
%                                    1/phi_max^2, 1/rate_max^2]);
%                 Q_aug_loc  = blkdiag(q_int * eye(3), Q_loc);
%                 R_inv      = m_max^2 * eye(3);
% 
%                 try
%                     Pss_aug_loc = care(A_aug, L_aug, Q_aug_loc, eye(size(L_aug,2)));
%                 catch
%                     continue;   % skip if CARE can't solve
%                 end
% 
%                 Pss_aug_bot = Pss_aug_loc(4:9, :);
% 
%                 rho_fn = @(a0) compute_floquet_rho_aug(a0, A_aug, R_inv, ...
%                                   Pss_aug_bot, B_lvlh_trimmed, Ix, Iy, Iz, dt_data, N_orbit);
%                 [a_opt, rho_opt] = fminbnd(rho_fn, 0.01, 100);
% 
%                 %Only sim if Floquet is reasonably stable
%                 if rho_opt > 1.0
%                     fprintf("Rho > 1, phi_max = %f, m_max = %f, q_int = %f.  finding other state\n", phi_max * 180/pi, m_max, q_int);
%                     results_aug = [results_aug; phi_max*180/pi, m_max, q_int, a_opt, rho_opt, NaN, NaN];
%                     continue;
%                 end
% 
%                 [rms_deg, peak_deg] = sim_closed_loop_aug_with_disturbance( ...
%                     Ad_aug, Bd_aug, R_inv, Pss_aug_bot, a_opt, ...
%                     B_lvlh_trimmed, Td_trimmed, t_tv_trimmed, ...
%                     Ix, Iy, Iz, 15);
% 
%                 results_aug = [results_aug; ...
%                     phi_max*180/pi, m_max, q_int, a_opt, rho_opt, mean(rms_deg), mean(peak_deg)];
%             end
%         end
%     end
% end
% 
% % Filter to stable configs only
% stable = results_aug(results_aug(:,5) < 1.0 & ~isnan(results_aug(:,6)), :);
% [~, ibest] = min(stable(:,6));
% fprintf('\nBest stable AUG config:\n');
% fprintf('  phi_max=%.1f, m_max=%.1e, q_int=%.1e, alpha0=%.2f, rho=%.3f, RMS=%.2f deg, peak=%.2f deg\n', ...
%     stable(ibest,:));
% 
% disp(array2table(stable, 'VariableNames', ...
%     {'phi_max_deg','m_max','q_int','alpha0','rho','rms_deg','peak_deg'}));
% 
