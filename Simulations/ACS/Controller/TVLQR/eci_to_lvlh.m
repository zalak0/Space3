function [lvlh_vec] = eci_to_lvlh(eci_vec, r_eci, v_eci)
    r_eci = r_eci(:);   % force column
    v_eci = v_eci(:);
    eci_vec = eci_vec(:);

    r_hat = r_eci / norm(r_eci);
    h_hat = cross(r_eci, v_eci) / norm(cross(r_eci, v_eci));
    y_hat = cross(h_hat, r_hat);

    R_eci2lvlh = [y_hat, -h_hat, -r_hat]';  % 3×3
    lvlh_vec = (R_eci2lvlh * eci_vec)';      % return as 1×3 row to match B_lvlh(i,:)
end