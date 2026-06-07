clc; 
clear all;

%% -- Constants / inertia (same as TVLQR) --------------------------------
mu = 3.986e14; earth_rad = 6371000; r = earth_rad + 515000;
period = (2*pi*r^(3/2))/sqrt(mu);   omega_o = (2*pi)/period;
mass = 2.0; Lx=0.226; Ly=0.100; Lz=0.100;
Ix = (1/12)*mass*(Ly^2+Lz^2);
Iy = (1/12)*mass*(Lx^2+Lz^2);
Iz = (1/12)*mass*(Lx^2+Lz^2);   % note: keep your own value
I  = diag([Ix Iy Iz]);
m_max = 0.046;                  % A*m^2 dipole limit

%% -- Load field + disturbances (same as TVLQR) --------------------------
mag  = load('GOOSE_mag_Td.mat');
aero = load('GOOSE_aero_Td.mat');
srp  = load('GOOSE_srp_Td.mat');
[~, Td_reordered, idx] = ext_disturbances(mag, aero, srp);

t_tv   = mag.t;
B_eci  = mag.B_eci;             % raw IGRF inertial field (NOT lvlh here)
t_trim = t_tv(idx);
Beci_trim = B_eci(idx,:);
Td_trim   = Td_reordered(idx,:);   % inertial-ish disturbance, body-applied below
dt_data = t_tv(2)-t_tv(1);
T_total = length(t_trim)*dt_data;  % one orbit of data

%% -- B-dot detumble simulation (nonlinear) ------------------------------
dt    = 0.5;                    % s, fine vs magnetorquer timescale
T_sim = 30 * 5695;                % requirement horizon
N     = round(T_sim/dt);

q = [1;0;0;0];                                  % body<-ECI attitude
w = (50*pi/180)/sqrt(3) * [1;1;1];              % 50 deg/s tip-off, spread
k_bdot = 2*omega_o*(1+sin(97.4*pi/180))*min([Ix Iy Iz]);  % gain rule of thumb

w_hist = zeros(3,N); t_hist=(0:N-1)*dt;
B_body_prev = [];
for n = 1:N
    t_mod = mod(t_hist(n), T_total);
    Beci  = interp1(t_trim, Beci_trim, t_mod, 'linear','extrap')';  % [T] (units per your .mat)
    R = quat2dcm_body(q);            % ECI->body
    B_body = R*Beci;

    % --- B-dot control law (derivative form, saturated) ---
    if isempty(B_body_prev)
        m = [0;0;0];
    else
        Bdot = (B_body - B_body_prev)/dt;
        m = -k_bdot * Bdot / max(norm(B_body)^2, eps);   % |B|^2
        s = max(abs(m))/m_max; if s>1, m = m/s; end     % proportional saturation (preserve dir)
        if s>1, m = m/s; end
    end
    B_body_prev = B_body;

    T_ctrl = cross(m, B_body);
    Td_b   = R*Td_trim_interp(t_trim,Td_trim,t_mod)';   % disturbance in body
    tau = T_ctrl + Td_b;
    [w, q] = rk4_step(w, q, tau, I, dt);
    w = w/1; q = q/norm(q);
    w_hist(:,n) = w;
end
%% -- Plot / check -------------------------------------------------------
wn = vecnorm(w_hist)*180/pi;
figure; plot(t_hist/3600, wn); grid on;
xlabel('time [hr]'); ylabel('|\omega| [deg/s]');
yline(50,'--'); title('B-dot detumble');

thr = 5;  % deg/s "detumbled" threshold (set to your spec)
i_thr = find(wn<thr,1);
if isempty(i_thr)
    fprintf('NOT detumbled below %.1f deg/s within 48 h (min %.2f)\n',thr,min(wn));
else
    fprintf('Detumbled to <%.1f deg/s at %.2f h\n', thr, t_hist(i_thr)/3600);
end

%% -- helpers ------------------------------------------------------------
function v = Td_trim_interp(t,Td,tm)
    v = interp1(t,Td,tm,'linear','extrap');
end

function [w2,q2] = rk4_step(w,q,tau,I,dt)
    f = @(w,q) deal(I\(-cross(w,I*w)+tau), 0.5*quatmul(q,[0;w]));
    [k1w,k1q]=f(w,q);
    [k2w,k2q]=f(w+0.5*dt*k1w, q+0.5*dt*k1q);
    [k3w,k3q]=f(w+0.5*dt*k2w, q+0.5*dt*k2q);
    [k4w,k4q]=f(w+dt*k3w,     q+dt*k3q);
    w2 = w + dt/6*(k1w+2*k2w+2*k3w+k4w);
    q2 = q + dt/6*(k1q+2*k2q+2*k3q+k4q);
end

function R = quat2dcm_body(q)   % ECI->body, q=[w x y z]
    w=q(1);x=q(2);y=q(3);z=q(4);
    R=[1-2*(y^2+z^2), 2*(x*y+w*z), 2*(x*z-w*y);
       2*(x*y-w*z), 1-2*(x^2+z^2), 2*(y*z+w*x);
       2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x^2+y^2)];
end
function p = quatmul(a,b)
    aw=a(1);av=a(2:4); bw=b(1);bv=b(2:4);
    p=[aw*bw-dot(av,bv); aw*bv+bw*av+cross(av,bv)];
end