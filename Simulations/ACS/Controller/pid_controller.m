function [K_goose] = pid_controller(A, B,zeta, ts)

wn   = 4 / (zeta * ts);
fprintf('wn = %.5f rad/s (disturbance freq = %.5f)\n', ...
    wn, 2*pi/5700);
fprintf('Bandwidth ratio = %.1fx\n', wn/(2*pi/5700));

desired_poles_GOOSE = [
    -zeta*wn + 1i*wn*sqrt(1-zeta^2),
    -zeta*wn - 1i*wn*sqrt(1-zeta^2),
    -2*zeta*wn + 1i*wn*sqrt(1-zeta^2),
    -2*zeta*wn - 1i*wn*sqrt(1-zeta^2),
    -3*zeta*wn + 1i*wn*sqrt(1-zeta^2),
    -3*zeta*wn - 1i*wn*sqrt(1-zeta^2)
];

K_goose = place(A, B, desired_poles_GOOSE);
