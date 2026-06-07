function M = magnetic_attitude_controller(x, B_y)
    % Extract states
    phi = x(1);      phi_dot = x(2);
    psi = x(5);      psi_dot = x(6);
    
    % Control parameters
    K_p_roll = 1000;  K_d_roll = 100;
    K_p_yaw = 1000;   K_d_yaw = 100;
    
    % Inertias
    I_x = 36046;
    I_z = 93848;
    
    % References
    phi_ref = 0;      psi_ref = 0;
    phi_dot_ref = 0;  psi_dot_ref = 0;
    
    % Desired angular accelerations (PID control)
    phi_ddot_cmd = K_p_roll*(phi_ref - phi) + K_d_roll*(phi_dot_ref - phi_dot);
    psi_ddot_cmd = K_p_yaw*(psi_ref - psi) + K_d_yaw*(psi_dot_ref - psi_dot);
    
    % Required torques
    T_cx = I_x * phi_ddot_cmd;
    T_cz = I_z * psi_ddot_cmd;
    
    % Convert to magnetic moments
    if abs(B_y) > 1e-6
        M_x = T_cz / B_y;
        M_z = -T_cx / B_y;
    else
        M_x = 0;
        M_z = 0;
    end
    M_y = 0;
    
    M = [M_x; M_y; M_z];
end