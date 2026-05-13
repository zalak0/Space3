function [body_vec] = lvlh_to_body(x)
% LVLH to body (from attitude state, small angle ok near nominal)
phi = x(1); theta = x(3); psi = x(5);

Rx = [1,      0,       0;
      0,  cos(phi), sin(phi);
      0, -sin(phi), cos(phi)];

Ry = [cos(theta), 0, -sin(theta);
      0,          1,  0;
      sin(theta), 0,  cos(theta)];

Rz = [cos(psi),  sin(psi), 0;
     -sin(psi),  cos(psi), 0;
      0,         0,        1];

R_lvlh2body = Rz * Ry * Rx;
body_vec = R_lvlh2body * B_lvlh;
end