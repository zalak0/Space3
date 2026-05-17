function [T_disturbance, Td_reordered, idx] = ext_disturbances(mag, aero, srp)
%% ============================================================
%%  GOOSE ADCS — Load and combine disturbance torques
%% ============================================================

%% --- Extract relevant fields ---
t_mag   = mag.t;            % (1140×1) time vector [s]
Td_mag  = mag.T_body_sp;    % (1140×3) sun-pointing body-frame torque [N·m]

t_aero  = aero.t;           % (1138×1)
Td_aero = aero.T_body;      % (1138×3)

t_srp   = srp.t;            % (1140×1)
Td_srp  = srp.T_body;       % (1140×3)

%% --- Handle sample count mismatch ---
% mag/srp have 1140 samples, aero has 1138 — need common time base
% Use the shortest time vector as reference and interpolate others onto it
t_common = t_aero;   % shortest — 1138 samples

% Interpolate mag and srp onto aero time base
Td_mag_interp  = interp1(t_mag,  Td_mag,  t_common, 'linear', 'extrap');
Td_srp_interp  = interp1(t_srp,  Td_srp,  t_common, 'linear', 'extrap');
Td_aero_interp = Td_aero;   % already on t_common

Td_combined = Td_mag_interp + Td_aero_interp + Td_srp_interp;  % (1138×3)

% Set t_sim to match your Simulink stop time
% Full orbit: ~5687s | Controller settling only: 1000s
t_sim = 5687*2;   % seconds — adjust as needed
idx   = t_common <= t_sim;

t_trimmed  = t_common(idx);
Td_trimmed = Td_combined(idx, :);

%% --- Load disturbance as simple array (most reliable format) ---
% Array format: first column = time, columns 2-4 = Tx, Ty, Tz
% This is the simplest From Workspace format — no struct needed

clear T_disturbance
Td_reordered = Td_trimmed; % swap columns 2 and 3

T_disturbance = [double(t_trimmed), double(Td_reordered)];
% Result: (569×4) array
% Col 1: time [s]
% Col 2: Tdx [N·m]
% Col 3: Tdy [N·m]  
% Col 4: Tdz [N·m]

%Verify
fprintf('T_disturbance size: %dx%d\n', size(T_disturbance));
fprintf('Time range: %.1f to %.1f s\n', T_disturbance(1,1), T_disturbance(end,1));
fprintf('Max combined torque: %.4f nNm\n', ...
    max(vecnorm(T_disturbance(:,2:4), 2, 2)) * 1e9);

%% --- Sanity check ---
fprintf('=== Disturbance Torque Summary ===\n');
fprintf('  Samples:           %d\n',   length(t_trimmed));
fprintf('  Time span:         %.1f to %.1f s\n', ...
    t_trimmed(1), t_trimmed(end));
fprintf('  Max |Td_mag|:      %.4f nNm\n', ...
    max(vecnorm(Td_mag_interp(idx,:),  2, 2)) * 1e9);
fprintf('  Max |Td_aero|:     %.4f nNm\n', ...
    max(vecnorm(Td_aero_interp(idx,:), 2, 2)) * 1e9);
fprintf('  Max |Td_srp|:      %.4f nNm\n', ...
    max(vecnorm(Td_srp_interp(idx,:),  2, 2)) * 1e9);
fprintf('  Max |Td_combined|: %.4f nNm\n', ...
    max(vecnorm(Td_trimmed,            2, 2)) * 1e9);
[~, dom] = max(max(abs(Td_trimmed)));
axes_names = {'X', 'Y', 'Z'};
fprintf('  Dominant axis:     %s\n', axes_names{dom});

%% Verify K produces sensible torques for realistic inputs
% % % At 60 degree (1.047 rad) misalignment, what torque does K produce?
% % x_initial = [pi/3; 0; pi/3; 0; pi/3; 0];   % 60° on all axes
% % Tc_initial = -K_lqr * x_initial;
% % fprintf('Initial control torque:\n');
% % fprintf('  Tcx = %.4e N·m\n', Tc_initial(1));
% % fprintf('  Tcy = %.4e N·m\n', Tc_initial(2));
% % fprintf('  Tcz = %.4e N·m\n', Tc_initial(3));
% % fprintf('Max disturbance: 3.024e-6 N·m\n');
% % fprintf('Tc/Td ratio: %.1f×\n', max(abs(Tc_initial))/3.024e-6);
end