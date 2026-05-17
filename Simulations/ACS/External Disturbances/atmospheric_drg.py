#!/usr/bin/env python3
"""
GOOSE 2U CubeSat — Aerodynamic Disturbance Torque Analysis
===========================================================
Free Molecular Flow (FMF) Panel Method + NRLMSISE-00 Atmosphere

This script:
  1. Calls the NRLMSISE-00 model for real atmospheric density at 515 km
  2. Decomposes the 2U CubeSat into flat panels
  3. Computes free molecular flow forces/torques using Sentman (1961) model
  4. Sweeps over all attitudes to find worst-case disturbance torque
  5. Compares against magnetorquer authority
  6. Generates publication-quality figures

References:
  [1] Sentman (1961), "Free molecule flow theory and its application
      to the determination of aerodynamic forces"
  [2] Wertz et al. (2011), Space Mission Engineering: The New SMAD
  [3] Picone et al. (2002), "NRLMSISE-00 empirical model"
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from nrlmsise00 import msise_model
from datetime import datetime

# ======================================================================
#  1. NRLMSISE-00 ATMOSPHERIC MODEL
# ======================================================================
print("=" * 64)
print("  GOOSE 2U CubeSat — Aerodynamic Disturbance Analysis")
print("=" * 64)
print()

# Mission parameters
altitude_km = 515.0  # km
lat = 0.0            # deg (equatorial — representative)
lon = 0.0            # deg

# Solar activity conditions
# Solar Maximum (worst case): F10.7 ~ 200-250, Ap ~ 30-50
# Solar Minimum (nominal):    F10.7 ~ 70,      Ap ~ 4

conditions = {
    'Solar Maximum': {
        'f107': 250.0, 'f107a': 250.0, 'ap': 40,
        'date': datetime(2025, 6, 21, 12, 0, 0),  # summer solstice noon
    },
    'Solar Minimum': {
        'f107': 70.0, 'f107a': 70.0, 'ap': 4,
        'date': datetime(2025, 6, 21, 12, 0, 0),
    },
}

print("NRLMSISE-00 Atmospheric Density at {:.0f} km".format(altitude_km))
print("-" * 64)

density_results = {}
for name, cond in conditions.items():
    # msise_model returns (densities, temperatures)
    # densities indices: [He, O, N2, O2, Ar, H, N, AnomO, rho_total]
    #   index 5 = total mass density in kg/m^3
    # temperatures: [exospheric_temp, temp_at_alt]
    result = msise_model(
        cond['date'],
        altitude_km,
        lat, lon,
        cond['f107a'], cond['f107'], cond['ap'],
        lst=12.0  # local solar time
    )

    densities = result[0]   # number densities + total mass density
    temps = result[1]        # temperatures

    rho_total = densities[5]  # total mass density [kg/m^3] (index 5)
    n_He = densities[0]
    n_O  = densities[1]
    n_N2 = densities[2]
    n_O2 = densities[3]
    n_Ar = densities[4]
    n_H  = densities[6]
    n_N  = densities[7]
    T_exo = temps[0]
    T_alt = temps[1]

    # Total number density
    n_total = n_He + n_O + n_N2 + n_O2 + n_Ar + n_H + n_N

    density_results[name] = {
        'rho': rho_total,
        'T_exo': T_exo,
        'T_alt': T_alt,
        'n_O': n_O,
        'n_N2': n_N2,
        'n_total': n_total,
        'O_frac': n_O / n_total * 100 if n_total > 0 else 0,
    }

    print(f"\n  {name} (F10.7={cond['f107']:.0f}, Ap={cond['ap']}):")
    print(f"    Total mass density:    rho = {rho_total:.4e} kg/m^3")
    print(f"    Exospheric temp:       T_exo = {T_exo:.0f} K")
    print(f"    Temperature at alt:    T = {T_alt:.0f} K")
    print(f"    Number density (O):    n_O = {n_O:.4e} m^-3")
    print(f"    Number density (N2):   n_N2 = {n_N2:.4e} m^-3")
    print(f"    Total number density:  n = {n_total:.4e} m^-3")
    print(f"    Composition:           {n_O/n_total*100:.1f}% O, "
          f"{n_N2/n_total*100:.1f}% N2")

    # Mean free path calculation
    # For atomic oxygen: effective diameter ~ 3.0e-10 m
    d_eff = 3.0e-10  # m
    if n_total > 0:
        lam = 1.0 / (np.sqrt(2) * np.pi * d_eff**2 * n_total)
        print(f"    Mean free path:        lambda = {lam/1e3:.1f} km")
        l_ref = 0.1  # reference length ~ L/3 for 2U
        Kn = lam / l_ref
        print(f"    Knudsen number:        Kn = {Kn:.0f} (>> 1, FMF regime)")

print()

# ======================================================================
#  2. ORBITAL PARAMETERS
# ======================================================================
h = altitude_km * 1e3       # altitude [m]
R_earth = 6371e3             # Earth radius [m]
mu_E = 3.986004418e14        # gravitational parameter [m^3/s^2]
r_orb = R_earth + h
V_orb = np.sqrt(mu_E / r_orb)

print(f"Orbital velocity: {V_orb:.1f} m/s ({V_orb/1e3:.3f} km/s)")
print()

# ======================================================================
#  3. SATELLITE GEOMETRY — Panel Decomposition
# ======================================================================
# 2U CubeSat: 100 x 100 x 226 mm
# Body-mounted solar panels on +X, -X, +Y faces
# CoM at geometric centre (adjust when known)

L_x = 0.100   # m
L_y = 0.100   # m
L_z = 0.226   # m (2U long axis)

# Centre of mass offset from geometric centre.
# For a real 2U CubeSat, the CoM is NEVER at the geometric centre.
# CubeSat spec (ISO 17770) requires CoM within ±2 cm of geometric centre.
# Internal component asymmetry (battery, antenna, probe deployment mechanism)
# typically shifts the CoM by 5-20 mm from the geometric centre.
#
# We model three cases:
#   - Nominal:   CoM at geometric centre (zero aero torque — unrealistic)
#   - Realistic:  CoM offset by 10 mm along Z and 5 mm along X
#   - Worst-case: CoM offset by 20 mm (CubeSat spec limit)
#
# The CoM offset is THE source of aerodynamic disturbance torque.
# A perfectly symmetric satellite has zero aero torque regardless of
# attitude — all panel forces produce cancelling moments.

# Geometric centre
r_gc = np.array([L_x/2, L_y/2, L_z/2])

# Realistic CoM offset (10 mm along Z, 5 mm along X and Y)
# This is representative of a 2U with battery pack and Langmuir probe
# deployment mechanism offset from centre
com_offset = np.array([0.005, 0.005, 0.010])  # [m] offset from GC

# For worst-case analysis, use the CubeSat spec limit
com_offset_wc = np.array([0.010, 0.010, 0.020])  # 20 mm max offset

# Select which offset to use (worst-case for magnetorquer sizing)
r_com = r_gc + com_offset_wc
m_sat = 2.5   # kg (typical 2U)

print(f"\n  CoM offset from GC: [{com_offset_wc[0]*1e3:.0f}, "
      f"{com_offset_wc[1]*1e3:.0f}, {com_offset_wc[2]*1e3:.0f}] mm (worst-case)")
print(f"  NOTE: CoM offset is the primary source of aerodynamic torque.")

# Panel definitions: (name, area, normal, cp_position_from_corner)
panel_defs = [
    ('+X (solar)', L_y * L_z, np.array([1, 0, 0]),  np.array([L_x,  L_y/2, L_z/2])),
    ('-X (solar)', L_y * L_z, np.array([-1, 0, 0]), np.array([0,    L_y/2, L_z/2])),
    ('+Y (solar)', L_x * L_z, np.array([0, 1, 0]),  np.array([L_x/2, L_y,  L_z/2])),
    ('-Y',         L_x * L_z, np.array([0, -1, 0]), np.array([L_x/2, 0,    L_z/2])),
    ('+Z (top)',   L_x * L_y, np.array([0, 0, 1]),  np.array([L_x/2, L_y/2, L_z])),
    ('-Z (bot)',   L_x * L_y, np.array([0, 0, -1]), np.array([L_x/2, L_y/2, 0])),
]

# Convert cp from corner-origin to CoM-origin
panels = []
for name, area, normal, cp_corner in panel_defs:
    panels.append({
        'name': name,
        'area': area,
        'normal': normal.astype(float),
        'r_cp': cp_corner - r_com,
    })

print("Satellite geometry (2U CubeSat):")
print(f"  Dimensions: {L_x*1e3:.0f} x {L_y*1e3:.0f} x {L_z*1e3:.0f} mm")
print(f"  Mass: {m_sat:.1f} kg")
print(f"  CoM: [{r_com[0]*1e3:.1f}, {r_com[1]*1e3:.1f}, {r_com[2]*1e3:.1f}] mm")
print(f"  Panels: {len(panels)}")
print()

for p in panels:
    print(f"    {p['name']:15s}  A={p['area']*1e4:.2f} cm^2  "
          f"n=[{p['normal'][0]:+.0f},{p['normal'][1]:+.0f},{p['normal'][2]:+.0f}]  "
          f"r_cp=[{p['r_cp'][0]*1e3:+6.1f},{p['r_cp'][1]*1e3:+6.1f},{p['r_cp'][2]*1e3:+6.1f}] mm")
print()

# ======================================================================
#  4. FREE MOLECULAR FLOW MODEL — Sentman (1961)
# ======================================================================
# Hyperthermal approximation with diffuse reflection.
# Accommodation coefficient sigma = 1 (atomic oxygen in LEO causes
# near-complete energy accommodation on most surfaces).
#
# Normal drag coefficient per panel:
#   C_D_n = 2 + sigma * (pi/6) * sqrt(T_wall / T_inf)

sigma_ac = 1.0
T_wall   = 300.0  # K (satellite surface temperature)
T_inf    = density_results['Solar Maximum']['T_alt']

C_D_n = 2.0 + sigma_ac * (np.pi / 6) * np.sqrt(T_wall / T_inf)

print(f"Free molecular flow model (Sentman 1961):")
print(f"  Accommodation coeff: sigma = {sigma_ac:.1f}")
print(f"  T_wall = {T_wall:.0f} K,  T_inf = {T_inf:.0f} K")
print(f"  C_D_n = {C_D_n:.4f}")
print()


# ======================================================================
#  5. FORCE & TORQUE COMPUTATION
# ======================================================================
def compute_aero(panels, V_body, rho, V_mag, C_D_n):
    """
    Compute total aerodynamic force and torque on satellite.

    Parameters
    ----------
    panels : list of dict
        Panel definitions with 'area', 'normal', 'r_cp'.
    V_body : ndarray (3,)
        Freestream velocity in body frame [m/s].
    rho : float
        Atmospheric mass density [kg/m^3].
    V_mag : float
        Freestream speed magnitude [m/s].
    C_D_n : float
        Normal drag coefficient.

    Returns
    -------
    F_total : ndarray (3,)  — total force [N]
    T_total : ndarray (3,)  — total torque about CoM [N·m]
    F_panels : list of ndarray — per-panel forces
    """
    V_hat = V_body / np.linalg.norm(V_body)
    F_total = np.zeros(3)
    T_total = np.zeros(3)
    F_panels = []

    for p in panels:
        n_i = p['normal']
        cos_theta = np.dot(n_i, V_hat)

        if cos_theta > 0:  # panel faces into flow
            F_mag = rho * V_mag**2 * p['area'] * cos_theta * C_D_n
            F_i = -F_mag * n_i  # force along inward normal
        else:
            F_i = np.zeros(3)  # shadowed

        F_panels.append(F_i)
        F_total += F_i
        T_total += np.cross(p['r_cp'], F_i)

    return F_total, T_total, F_panels


def rotation_matrix_yz(alpha, beta):
    """
    Combined Rz(beta) @ Ry(alpha) rotation matrix.
    alpha = pitch about Y, beta = yaw about Z.
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta), np.sin(beta)
    Ry = np.array([[ca, 0, sa], [0, 1, 0], [-sa, 0, ca]])
    Rz = np.array([[cb, -sb, 0], [sb, cb, 0], [0, 0, 1]])
    return Rz @ Ry


# ======================================================================
#  6. WORST-CASE TORQUE SWEEP
# ======================================================================
print("=" * 64)
print("  WORST-CASE TORQUE SWEEP (Solar Maximum)")
print("=" * 64)
print()

rho_wc = density_results['Solar Maximum']['rho']

N_alpha = 361
N_beta = 361
alpha_deg = np.linspace(-180, 180, N_alpha)
beta_deg = np.linspace(-180, 180, N_beta)

T_mag_map = np.zeros((N_alpha, N_beta))
F_mag_map = np.zeros((N_alpha, N_beta))
Tx_map = np.zeros((N_alpha, N_beta))
Ty_map = np.zeros((N_alpha, N_beta))
Tz_map = np.zeros((N_alpha, N_beta))

print("  Sweeping {} x {} = {} orientations...".format(
    N_alpha, N_beta, N_alpha * N_beta), end=' ', flush=True)

for ia, alpha in enumerate(np.deg2rad(alpha_deg)):
    for ib, beta in enumerate(np.deg2rad(beta_deg)):
        R = rotation_matrix_yz(alpha, beta)
        V_body = V_orb * R @ np.array([1, 0, 0])

        F_tot, T_tot, _ = compute_aero(panels, V_body, rho_wc, V_orb, C_D_n)

        T_mag_map[ia, ib] = np.linalg.norm(T_tot)
        F_mag_map[ia, ib] = np.linalg.norm(F_tot)
        Tx_map[ia, ib] = T_tot[0]
        Ty_map[ia, ib] = T_tot[1]
        Tz_map[ia, ib] = T_tot[2]

print("done.")
print()

# Find worst case
idx_flat = np.argmax(T_mag_map)
ia_max, ib_max = np.unravel_index(idx_flat, T_mag_map.shape)
alpha_max_deg = alpha_deg[ia_max]
beta_max_deg = beta_deg[ib_max]
max_T = T_mag_map[ia_max, ib_max]

idx_flat_f = np.argmax(F_mag_map)
ia_maxf, ib_maxf = np.unravel_index(idx_flat_f, F_mag_map.shape)
max_F = F_mag_map[ia_maxf, ib_maxf]

print(f"RESULTS — Worst-Case Aerodynamic Disturbance:")
print(f"  Maximum torque:  {max_T:.4e} N·m  ({max_T*1e9:.2f} nN·m)")
print(f"    at alpha = {alpha_max_deg:.1f}°, beta = {beta_max_deg:.1f}°")
print(f"  Maximum force:   {max_F:.4e} N  ({max_F*1e6:.4f} µN)")
print(f"    at alpha = {alpha_deg[ia_maxf]:.1f}°, beta = {beta_deg[ib_maxf]:.1f}°")
print()

# Detailed breakdown at worst case
R_wc = rotation_matrix_yz(np.deg2rad(alpha_max_deg), np.deg2rad(beta_max_deg))
V_body_wc = V_orb * R_wc @ np.array([1, 0, 0])
F_wc, T_wc, F_panels_wc = compute_aero(panels, V_body_wc, rho_wc, V_orb, C_D_n)

print("Worst-case torque components [N·m]:")
print(f"  Tx = {T_wc[0]:.4e}")
print(f"  Ty = {T_wc[1]:.4e}")
print(f"  Tz = {T_wc[2]:.4e}")
print()

print("Per-panel force breakdown at worst-case orientation:")
for i, p in enumerate(panels):
    F_i = F_panels_wc[i]
    mag = np.linalg.norm(F_i)
    if mag > 0:
        print(f"  {p['name']:15s}  |F| = {mag:.4e} N  ({mag*1e6:.4f} µN)")
    else:
        print(f"  {p['name']:15s}  (shadowed)")
print()

# ======================================================================
#  7. MAGNETORQUER AUTHORITY COMPARISON
# ======================================================================
mu_mag = 0.1       # A·m² per axis
B_min = 25e-6      # T (minimum field at 515 km)
B_max = 50e-6      # T (maximum field at 515 km)
T_ctrl_min = mu_mag * B_min
T_ctrl_max = mu_mag * B_max

print("=" * 64)
print("  MAGNETORQUER AUTHORITY COMPARISON")
print("=" * 64)
print(f"  Dipole moment:      mu = {mu_mag:.2f} A·m² per axis")
print(f"  Earth field range:  B = {B_min*1e6:.0f} - {B_max*1e6:.0f} µT")
print(f"  Control torque:     {T_ctrl_min:.2e} to {T_ctrl_max:.2e} N·m")
print(f"  Worst-case aero:    {max_T:.2e} N·m")
if max_T > 0:
    print(f"  Margin (min B):     {T_ctrl_min/max_T:.0f}x")
    print(f"  Margin (max B):     {T_ctrl_max/max_T:.0f}x")
else:
    print(f"  Margin:             inf (zero aero torque — check CoM offset)")

if T_ctrl_min > max_T:
    print("  >> PASS: Magnetorquer authority exceeds worst-case aero torque.")
else:
    print("  >> WARNING: Review margin — aero torque may challenge control.")
print()

# --- CoM Sensitivity Analysis ---
print("  CoM Offset Sensitivity:")
for offset_mm in [1, 5, 10, 15, 20]:
    r_com_test = r_gc + np.array([1, 1, 1]) * offset_mm * 1e-3
    panels_test = []
    for name, area, normal, cp_corner in panel_defs:
        panels_test.append({
            'name': name, 'area': area,
            'normal': normal.astype(float),
            'r_cp': cp_corner - r_com_test,
        })
    # Find max torque over a coarse sweep
    max_T_test = 0
    for alpha in np.deg2rad(np.linspace(-180, 180, 73)):
        for beta in np.deg2rad(np.linspace(-180, 180, 73)):
            R = rotation_matrix_yz(alpha, beta)
            V_body = V_orb * R @ np.array([1, 0, 0])
            _, T_tot, _ = compute_aero(panels_test, V_body, rho_wc, V_orb, C_D_n)
            max_T_test = max(max_T_test, np.linalg.norm(T_tot))
    if max_T_test > 0:
        margin = T_ctrl_min / max_T_test
    else:
        margin = float('inf')
    print(f"    Offset = {offset_mm:2d} mm:  T_max = {max_T_test:.4e} N·m  "
          f"(margin: {margin:.0f}x)")
print()

# ======================================================================
#  8. DENSITY & TORQUE vs ALTITUDE (using real NRLMSISE-00)
# ======================================================================
print("=" * 64)
print("  TORQUE vs ALTITUDE (NRLMSISE-00, Solar Maximum)")
print("=" * 64)
print()

alt_range_km = np.arange(350, 651, 10)
rho_vs_alt = np.zeros_like(alt_range_km, dtype=float)
T_max_vs_alt = np.zeros_like(alt_range_km, dtype=float)
F_max_vs_alt = np.zeros_like(alt_range_km, dtype=float)

cond_max = conditions['Solar Maximum']
for i, alt_km in enumerate(alt_range_km):
    result = msise_model(
        cond_max['date'], alt_km, lat, lon,
        cond_max['f107a'], cond_max['f107'], cond_max['ap'],
        lst=12.0
    )
    rho_i = result[0][5]
    rho_vs_alt[i] = rho_i

    V_i = np.sqrt(mu_E / (R_earth + alt_km * 1e3))
    V_body_i = V_i * R_wc @ np.array([1, 0, 0])

    F_i, T_i, _ = compute_aero(panels, V_body_i, rho_i, V_i, C_D_n)
    T_max_vs_alt[i] = np.linalg.norm(T_i)
    F_max_vs_alt[i] = np.linalg.norm(F_i)

print(f"  {'Alt [km]':>10s}  {'rho [kg/m^3]':>14s}  {'|T_max| [N·m]':>16s}  {'|F_max| [N]':>14s}")
print(f"  {'-'*10}  {'-'*14}  {'-'*16}  {'-'*14}")
for i in range(0, len(alt_range_km), 5):
    print(f"  {alt_range_km[i]:10.0f}  {rho_vs_alt[i]:14.4e}  {T_max_vs_alt[i]:16.4e}  {F_max_vs_alt[i]:14.4e}")
print()

# ======================================================================
#  9. GENERATE FIGURES
# ======================================================================
plt.style.use('default')
fig_dpi = 150

# --- Figure 1: Torque Magnitude Heatmap ---
fig1, ax1 = plt.subplots(figsize=(10, 7), dpi=fig_dpi)
im = ax1.pcolormesh(beta_deg, alpha_deg, T_mag_map * 1e9,
                     shading='auto', cmap='hot')
cb = plt.colorbar(im, ax=ax1)
cb.set_label('Torque Magnitude [nN·m]')
ax1.set_xlabel('Sideslip angle β [deg]')
ax1.set_ylabel('Angle of attack α [deg]')
ax1.set_title(f'Aerodynamic Torque Magnitude at {altitude_km:.0f} km (Solar Max)\n'
              f'ρ = {rho_wc:.2e} kg/m³')
ax1.plot(beta_max_deg, alpha_max_deg, 'c+', markersize=15, markeredgewidth=2)
ax1.annotate(f'Max: {max_T*1e9:.2f} nN·m',
             (beta_max_deg, alpha_max_deg), fontsize=9, color='cyan',
             xytext=(10, 10), textcoords='offset points')
fig1.tight_layout()
fig1.savefig('aero_torque_heatmap.png', dpi=fig_dpi)

# --- Figure 2: Torque Components vs Alpha at beta=0 ---
ib_zero = N_beta // 2  # beta = 0
fig2, ax2 = plt.subplots(figsize=(10, 6), dpi=fig_dpi)
ax2.plot(alpha_deg, Tx_map[:, ib_zero] * 1e9, 'r-', lw=1.5, label='$T_x$')
ax2.plot(alpha_deg, Ty_map[:, ib_zero] * 1e9, 'g-', lw=1.5, label='$T_y$')
ax2.plot(alpha_deg, Tz_map[:, ib_zero] * 1e9, 'b-', lw=1.5, label='$T_z$')
T_total_slice = np.sqrt(Tx_map[:, ib_zero]**2 + Ty_map[:, ib_zero]**2 +
                         Tz_map[:, ib_zero]**2) * 1e9
ax2.plot(alpha_deg, T_total_slice, 'k--', lw=2, label='$|\\mathbf{T}|$')
ax2.axhline(T_ctrl_min * 1e9, color='m', ls='--', lw=1,
            label=f'Magnetorquer (min B={B_min*1e6:.0f} µT)')
ax2.axhline(-T_ctrl_min * 1e9, color='m', ls='--', lw=1)
ax2.set_xlabel('Angle of attack α [deg]')
ax2.set_ylabel('Torque [nN·m]')
ax2.set_title(f'Aerodynamic Torque vs Angle of Attack (β = 0°, {altitude_km:.0f} km)')
ax2.legend(loc='best', fontsize=9)
ax2.grid(True, alpha=0.3)
fig2.tight_layout()
fig2.savefig('aero_torque_vs_alpha.png', dpi=fig_dpi)

# --- Figure 3: Torque and Density vs Altitude ---
fig3, ax3a = plt.subplots(figsize=(10, 6), dpi=fig_dpi)
color1 = 'tab:blue'
ax3a.semilogy(alt_range_km, T_max_vs_alt * 1e9, color=color1, lw=2,
              label='Aero torque')
ax3a.axhline(T_ctrl_min * 1e9, color='r', ls='--', lw=1.5,
             label=f'Magnetorquer min ({T_ctrl_min*1e9:.0f} nN·m)')
ax3a.axhline(T_ctrl_max * 1e9, color='r', ls='-', lw=1.5,
             label=f'Magnetorquer max ({T_ctrl_max*1e9:.0f} nN·m)')
ax3a.axvline(515, color='gray', ls=':', lw=1, alpha=0.5)
ax3a.set_xlabel('Altitude [km]')
ax3a.set_ylabel('Max Aero Torque [nN·m]', color=color1)
ax3a.tick_params(axis='y', labelcolor=color1)
ax3a.legend(loc='upper right', fontsize=9)
ax3a.grid(True, alpha=0.3)

ax3b = ax3a.twinx()
color2 = 'tab:orange'
ax3b.semilogy(alt_range_km, rho_vs_alt, color=color2, lw=1.5, ls='--',
              alpha=0.7)
ax3b.set_ylabel('Mass Density ρ [kg/m³]', color=color2)
ax3b.tick_params(axis='y', labelcolor=color2)

ax3a.set_title('Worst-Case Aerodynamic Torque vs Altitude\n'
               '(NRLMSISE-00, Solar Maximum, F10.7=250)')
fig3.tight_layout()
fig3.savefig('aero_torque_vs_altitude.png', dpi=fig_dpi)

# --- Figure 4: Force Heatmap ---
fig4, ax4 = plt.subplots(figsize=(10, 7), dpi=fig_dpi)
im4 = ax4.pcolormesh(beta_deg, alpha_deg, F_mag_map * 1e6,
                      shading='auto', cmap='viridis')
cb4 = plt.colorbar(im4, ax=ax4)
cb4.set_label('Force Magnitude [µN]')
ax4.set_xlabel('Sideslip angle β [deg]')
ax4.set_ylabel('Angle of attack α [deg]')
ax4.set_title(f'Aerodynamic Force Magnitude at {altitude_km:.0f} km (Solar Max)')
fig4.tight_layout()
fig4.savefig('aero_force_heatmap.png', dpi=fig_dpi)

# --- Figure 5: Atmospheric composition vs altitude ---
alt_comp_km = np.arange(200, 801, 10)
n_O_arr = np.zeros_like(alt_comp_km, dtype=float)
n_N2_arr = np.zeros_like(alt_comp_km, dtype=float)
n_O2_arr = np.zeros_like(alt_comp_km, dtype=float)
n_He_arr = np.zeros_like(alt_comp_km, dtype=float)
rho_arr = np.zeros_like(alt_comp_km, dtype=float)

for i, alt_km in enumerate(alt_comp_km):
    result = msise_model(
        cond_max['date'], alt_km, lat, lon,
        cond_max['f107a'], cond_max['f107'], cond_max['ap'],
        lst=12.0
    )
    n_He_arr[i] = result[0][0]
    n_O_arr[i]  = result[0][1]
    n_N2_arr[i] = result[0][2]
    n_O2_arr[i] = result[0][3]
    rho_arr[i]  = result[0][5]

fig5, ax5 = plt.subplots(figsize=(10, 6), dpi=fig_dpi)
ax5.semilogy(alt_comp_km, n_O_arr, 'r-', lw=1.5, label='O (atomic oxygen)')
ax5.semilogy(alt_comp_km, n_N2_arr, 'b-', lw=1.5, label='N₂')
ax5.semilogy(alt_comp_km, n_O2_arr, 'g-', lw=1.5, label='O₂')
ax5.semilogy(alt_comp_km, n_He_arr, 'm-', lw=1.5, label='He')
ax5.axvline(515, color='gray', ls=':', lw=1.5, label='GOOSE orbit (515 km)')
ax5.set_xlabel('Altitude [km]')
ax5.set_ylabel('Number Density [m⁻³]')
ax5.set_title('NRLMSISE-00 Atmospheric Composition (Solar Maximum)')
ax5.legend(loc='upper right', fontsize=9)
ax5.grid(True, alpha=0.3)
ax5.set_xlim([200, 800])
fig5.tight_layout()
fig5.savefig('atmosphere_composition.png', dpi=fig_dpi)

print("Figures saved.")
print()

# ======================================================================
#  10. SUMMARY
# ======================================================================
print("╔" + "═" * 62 + "╗")
print("║   SUMMARY — Aerodynamic Disturbance (GOOSE, 515 km)        ║")
print("╠" + "═" * 62 + "╣")
print(f"║  Atmospheric model:     NRLMSISE-00                        ║")
print(f"║  Solar activity:        F10.7 = 250, Ap = 40 (solar max)  ║")
print(f"║  Mass density:          {rho_wc:.4e} kg/m³              ║")
print(f"║  Orbital velocity:      {V_orb/1e3:.3f} km/s                   ║")
print(f"║  Flow regime:           Free molecular (Kn ≈ {1/(np.sqrt(2)*np.pi*(3e-10)**2*density_results['Solar Maximum']['n_total'])/0.1:.0f})         ║")
print(f"║  Drag coefficient:      C_D = {C_D_n:.4f}                   ║")
print(f"║  ──────────────────────────────────────────────────────── ║")
print(f"║  Max aero torque:       {max_T:.4e} N·m                ║")
print(f"║  Max aero force:        {max_F:.4e} N                  ║")
print(f"║  Worst α, β:            {alpha_max_deg:.1f}°, {beta_max_deg:.1f}°                      ║")
print(f"║  ──────────────────────────────────────────────────────── ║")
print(f"║  Magnetorquer µ:        {mu_mag:.2f} A·m²                      ║")
print(f"║  Control torque (min):  {T_ctrl_min:.2e} N·m                ║")
if max_T > 0:
    margin_min = f"{T_ctrl_min/max_T:.0f}"
    margin_max = f"{T_ctrl_max/max_T:.0f}"
else:
    margin_min = "∞"
    margin_max = "∞"
print(f"║  Torque margin (min):   {margin_min}x                            ║")
print(f"║  Torque margin (max):   {margin_max}x                            ║")
print("╚" + "═" * 62 + "╝")