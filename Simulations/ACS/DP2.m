%% MATLAB/Python pseudocode
mu = 3.986e14;             %m^3/s^2
earth_rad = 6371000;        %m
r = earth_rad + 515000;     %m
period = (2*pi * r^(3/2))/sqrt(mu);
omega = (2 * pi)/period;

%% GOOSE 2U CubeSat inertia 
mass = 2.0;      % kg — update with actual mass budget
Lx   = 0.226;    % m — long axis
Ly   = 0.100;    % m
Lz   = 0.100;    % m

Ix = (1/12) * mass * (Ly^2 + Lz^2);   % ~0.00333 kg·m²
Iy = (1/12) * mass * (Lx^2 + Lz^2);   % ~0.01020 kg·m²
Iz = (1/12) * mass * (Lx^2 + Ly^2);   % ~0.01020 kg·m²

A = [0,                           1,                           0,                           0,                           0,                           0;
    -4*(omega^(2))*(Iy-Iz)/Ix,    0,                           0,                           omega*(Ix+Iz-Iy)/Ix,                           0,               0;
     0,                           0,                           0,                           1,                           0,                           0;
     0,                           -omega*(Iz+Ix-Iy)/Iz, -(omega^2)*(Iy-Ix)/Iz,                 0,                           0,                           0;
     0,                           0,                           0,                           0,                           0,                           1;
     0,                           0,                           0,                            0,                           -3*(omega^2)*(Ix-Iz)/Iy,                           0];
B = [0 , 0, 0;
    1/Ix, 0, 0;
    0, 0, 0;
    0, 0, 1/Iz;
    0, 0, 0;
    0, 1/Iy,0];

C = eye(6);
D = zeros(6,3);

%% External Disturbances (and debugging)

mag  = load('GOOSE_mag_Td.mat');
aero = load('GOOSE_aero_Td.mat');
srp  = load('GOOSE_srp_Td.mat');

[T_disturbance, Td_reordered, idx] = ext_disturbances(mag, aero, srp, K_lqr);

%% Discretise matricies
% In your setup script, after defining A, B, C, D
dt = 1;
sys_c = ss(A, B, C, D);
sys_d = c2d(sys_c, dt, 'zoh');

Ad = sys_d.A;   % 6×6 — use this in the block
Bd = sys_d.B;   % 6×3 — use this in the block
Cd = sys_d.C;   % 3×6 — use this in the block
Dd = sys_d.D;   % 3×3 — use this in the block

% % PID controller
% zeta = 0.1;
% ts   = 100;    % seconds — faster than before
% 
% K_goose = pid_controller(A, B, zeta, ts);

%% LQR controller: LQR / TVLQR weights — dipole-based control
phi_max  = 2 * pi/180;     % 0.5° in radians ≈ 0.0087 rad
rate_max = 0.5 * pi/180;    % 0.05°/s in radians ≈ 8.7e-4 rad/s
m_max    = 0.2;              % A·m² — actuator hardware limit       
[K_lqr, P0, Q, R_inv] = lqr_controller(A, B, phi_max, rate_max, m_max);

%% TVLQR Implmenetation
% t and B_eci are already on a common 10s grid
t_tv   = mag.t;          % (N×1) [s]
B_eci  = mag.B_eci;      % (N×3) [T]
r_eci_arr = srp.r_eci_m;
v_eci_arr = srp.v_eci_m;
N_tv   = length(t_tv);

% --- Step 2: Rotate B_eci into body frame ---
% Sun-pointing nominal DCM: +X → Sun [1,0,0], orbit in XZ plane (RAAN=90°)
% For a first implementation, treat B_body ≈ B_eci (conservative —
% replace with your actual DCM sequence once attitude propagation is wired in)

%% Extract Magnetic Field metrics
B_lvlh = zeros(N_tv, 3);   % (N×3)

for i = 1:N_tv
    B_lvlh(i,:) = eci_to_lvlh(B_eci(i,:), r_eci_arr(i,:), v_eci_arr(i,:));
end

%% Repeat/fix up disturbance and magnetic field metrics
t_tv_trimmed = t_tv(idx);
B_lvlh_trimmed = B_lvlh(idx, :);
Td_trimmed = Td_reordered(idx, :);


% Assume B_lvlh is (1138×3) and t_tv is (1138×1)
N_repeat = 10;   % enough repeats to cover sim duration
B_lvlh_rep = repmat(B_lvlh_trimmed, N_repeat, 1);          % (1138*N × 3)
Td_rep = repmat(Td_trimmed, N_repeat, 1);

% Offset time for each repeat so it's monotonically increasing
dt = t_tv(2) - t_tv(1);                            % 10s
T_total = length(t_tv_trimmed) * dt;                        % 11380s
t_offsets = (0:N_repeat-1)' * T_total;             % [0, 11380, 22760, ...]
t_rep = repmat(t_tv_trimmed, N_repeat, 1) + repelem(t_offsets, length(t_tv_trimmed));

% Pack into timeseries for 'From Workspace'
B_lvlh_ts = timeseries(B_lvlh_rep, t_rep);
Td_ts = timeseries(Td_rep, t_rep);
