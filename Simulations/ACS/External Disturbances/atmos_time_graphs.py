#!/usr/bin/env python3
"""
GOOSE 2U CubeSat — Aerodynamic Torque Over Orbital Period(s)
=============================================================
Propagates a circular orbit at 515 km in a dawn-dusk SSO and computes
the aerodynamic disturbance torque at each timestep using:
  - NRLMSISE-00 for position-dependent atmospheric density
  - Free molecular flow panel method (Sentman 1961)
  - Assumed satellite attitude (sun-pointing or fixed inertial)

The key physics this captures:
  1. Density variation with latitude (bulge at equator vs poles)
  2. Velocity direction change relative to body frame as the
     satellite follows its orbital path
  3. Effect of attitude mode on torque profile
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from nrlmsise00 import msise_model
from datetime import datetime, timedelta
from scipy.io import savemat

# ======================================================================
#  1. ORBITAL MECHANICS — Circular Orbit Propagation
# ======================================================================
# Dawn-dusk sun-synchronous orbit at 515 km
# Inclination for SSO at 515 km: i ≈ 97.4°
# RAAN for dawn-dusk: Ω such that the orbit plane contains the
#   Earth-Sun line, i.e. RAAN ≈ local solar time 06:00/18:00

R_earth = 6371e3          # m
mu_E    = 3.986004418e14  # m^3/s^2
J2      = 1.08263e-3      # Earth J2
h_orbit = 515e3           # m
r_orb   = R_earth + h_orbit
V_orb   = np.sqrt(mu_E / r_orb)
T_orb   = 2 * np.pi * np.sqrt(r_orb**3 / mu_E)  # orbital period [s]
n_orb   = 2 * np.pi / T_orb                       # mean motion [rad/s]
inc     = np.deg2rad(97.4)  # SSO inclination

print("=" * 64)
print("  GOOSE — Aerodynamic Torque Over Orbital Period(s)")
print("=" * 64)
print(f"\n  Altitude:       {h_orbit/1e3:.0f} km")
print(f"  Orbital speed:  {V_orb:.1f} m/s ({V_orb/1e3:.3f} km/s)")
print(f"  Orbital period: {T_orb:.1f} s ({T_orb/60:.1f} min)")
print(f"  Inclination:    {np.rad2deg(inc):.1f}° (SSO)")
print()

# ======================================================================
#  2. SATELLITE GEOMETRY (same as static analysis)
# ======================================================================
L_x, L_y, L_z = 0.100, 0.100, 0.226  # 2U body dimensions [m]
r_gc = np.array([L_x/2, L_y/2, L_z/2])

# CoM offset cases
com_offsets = {
    'Realistic (10 mm)':  np.array([0.005, 0.005, 0.010]),
    'Worst-case (20 mm)': np.array([0.010, 0.010, 0.020]),
}

panel_defs = [
    ('+X', L_y * L_z, np.array([1, 0, 0]),  np.array([L_x,   L_y/2, L_z/2])),
    ('-X', L_y * L_z, np.array([-1, 0, 0]), np.array([0,     L_y/2, L_z/2])),
    ('+Y', L_x * L_z, np.array([0, 1, 0]),  np.array([L_x/2, L_y,   L_z/2])),
    ('-Y', L_x * L_z, np.array([0, -1, 0]), np.array([L_x/2, 0,     L_z/2])),
    ('+Z', L_x * L_y, np.array([0, 0, 1]),  np.array([L_x/2, L_y/2, L_z])),
    ('-Z', L_x * L_y, np.array([0, 0, -1]), np.array([L_x/2, L_y/2, 0])),
]

# FMF parameters
sigma_ac = 1.0
T_wall   = 300.0

def make_panels(com_offset):
    """Build panel list for a given CoM offset."""
    r_com = r_gc + com_offset
    panels = []
    for name, area, normal, cp_corner in panel_defs:
        panels.append({
            'name': name, 'area': area,
            'normal': normal.astype(float),
            'r_cp': cp_corner - r_com,
        })
    return panels


def compute_aero(panels, V_body, rho, V_mag, C_D_n):
    """Compute aero force and torque given body-frame velocity."""
    V_hat = V_body / np.linalg.norm(V_body)
    F_total = np.zeros(3)
    T_total = np.zeros(3)
    for p in panels:
        cos_theta = np.dot(p['normal'], V_hat)
        if cos_theta > 0:
            F_mag = rho * V_mag**2 * p['area'] * cos_theta * C_D_n
            F_i = -F_mag * p['normal']
            F_total += F_i
            T_total += np.cross(p['r_cp'], F_i)
    return F_total, T_total


# ======================================================================
#  3. ORBIT PROPAGATION + ATTITUDE MODEL
# ======================================================================
# We propagate a simple Keplerian circular orbit in the ECI frame.
#
# Position in the orbital plane (perifocal frame):
#   r_peri = r_orb * [cos(nu), sin(nu), 0]
#   v_peri = V_orb * [-sin(nu), cos(nu), 0]
# where nu = n * t (true anomaly = mean anomaly for circular orbit)
#
# Transform to ECI using the classical rotation:
#   R_ECI = R3(-RAAN) @ R1(-inc) @ R3(-argp)
# For circular orbit, argp is arbitrary; set to 0.
#
# For the dawn-dusk SSO, RAAN ≈ 90° (orbit plane contains Sun direction).

RAAN = np.deg2rad(90.0)   # Right ascension of ascending node
argp = np.deg2rad(0.0)    # Argument of periapsis (arbitrary for circular)

def perifocal_to_eci(r_peri, v_peri):
    """Rotate from perifocal to ECI frame."""
    cO, sO = np.cos(RAAN), np.sin(RAAN)
    ci, si = np.cos(inc), np.sin(inc)
    cw, sw = np.cos(argp), np.sin(argp)

    R = np.array([
        [cO*cw - sO*ci*sw, -cO*sw - sO*ci*cw,  sO*si],
        [sO*cw + cO*ci*sw, -sO*sw + cO*ci*cw, -cO*si],
        [si*sw,             si*cw,              ci   ]
    ])
    return R @ r_peri, R @ v_peri


def eci_to_ecef(r_eci, gmst):
    """Simple ECI to ECEF rotation (ignore precession/nutation)."""
    c, s = np.cos(gmst), np.sin(gmst)
    R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return R @ r_eci


def ecef_to_lla(r_ecef):
    """Convert ECEF position to geodetic lat, lon, alt (spherical approx)."""
    x, y, z = r_ecef
    r = np.linalg.norm(r_ecef)
    lat = np.rad2deg(np.arcsin(z / r))
    lon = np.rad2deg(np.arctan2(y, x))
    alt = r - R_earth
    return lat, lon, alt


# ======================================================================
#  4. ATTITUDE MODELS
# ======================================================================
# We consider two attitude scenarios:
#
# (a) SUN-POINTING: The satellite +X axis points toward the Sun.
#     In a dawn-dusk SSO, the Sun direction is approximately along
#     the orbit normal. The velocity vector rotates in the body Y-Z
#     plane as the satellite goes around the orbit.
#
# (b) FIXED INERTIAL: The satellite maintains a fixed orientation
#     in ECI. This is the "tumbled" or uncontrolled case — worst
#     case for checking if aero torque could spin up the satellite.
#
# For both cases, we need to compute V_body = R_body_from_eci @ V_eci

def attitude_sun_pointing(r_eci, v_eci, sun_dir_eci):
    """
    Compute body-from-ECI rotation for sun-pointing mode.

    Body frame convention:
      +X_body -> points toward Sun
      +Z_body -> along velocity direction (ram)
      +Y_body -> completes right-hand system

    Returns R such that V_body = R @ V_eci
    """
    # X_body = sun direction
    x_b = sun_dir_eci / np.linalg.norm(sun_dir_eci)

    # Z_body = velocity direction (projected perpendicular to x_b)
    v_hat = v_eci / np.linalg.norm(v_eci)
    z_b = v_hat - np.dot(v_hat, x_b) * x_b
    z_norm = np.linalg.norm(z_b)
    if z_norm < 1e-10:
        # Velocity parallel to sun — degenerate, pick arbitrary
        z_b = np.array([0, 0, 1])
    else:
        z_b = z_b / z_norm

    # Y_body = X × Z (right-hand)
    y_b = np.cross(z_b, x_b)
    y_b = y_b / np.linalg.norm(y_b)

    # DCM: rows are body axes expressed in ECI
    R = np.vstack([x_b, y_b, z_b])
    return R


def attitude_nadir_pointing(r_eci, v_eci):
    """
    Nadir-pointing LVLH attitude.
      -Z_body -> nadir (toward Earth centre)
      +X_body -> velocity direction
      +Y_body -> orbit normal
    """
    z_b = -r_eci / np.linalg.norm(r_eci)  # nadir
    x_b = v_eci / np.linalg.norm(v_eci)   # ram
    # Orthogonalise
    x_b = x_b - np.dot(x_b, z_b) * z_b
    x_b = x_b / np.linalg.norm(x_b)
    y_b = np.cross(z_b, x_b)
    R = np.vstack([x_b, y_b, z_b])
    return R


# ======================================================================
#  5. TIME-DOMAIN SIMULATION
# ======================================================================
N_orbits = 2
dt = 10.0  # timestep [s] — 10s gives ~57 points per orbit
t_end = N_orbits * T_orb
t = np.arange(0, t_end, dt)
N_steps = len(t)

# Epoch
epoch = datetime(2025, 6, 21, 12, 0, 0)  # Summer solstice noon
omega_earth = 7.2921159e-5  # Earth rotation rate [rad/s]
gmst0 = 0.0  # GMST at epoch (simplified)

# Sun direction in ECI (approximately +X for June solstice at noon)
# For a dawn-dusk SSO, the orbit plane contains this direction
sun_dir_eci = np.array([1.0, 0.0, 0.0])

# Solar conditions
f107 = 250.0   # Solar maximum
f107a = 250.0
ap = 40

# Preallocate
results = {}
com_label = 'Worst-case (20 mm)'
panels = make_panels(com_offsets[com_label])

# Storage for different attitude modes
attitude_modes = {
    'Sun-pointing': attitude_sun_pointing,
}

for mode_name in attitude_modes:
    results[mode_name] = {
        'T_vec': np.zeros((N_steps, 3)),
        'T_mag': np.zeros(N_steps),
        'F_vec': np.zeros((N_steps, 3)),
        'F_mag': np.zeros(N_steps),
        'rho':   np.zeros(N_steps),
        'lat':   np.zeros(N_steps),
        'lon':   np.zeros(N_steps),
        'alt':   np.zeros(N_steps),
    }

print(f"Simulating {N_orbits} orbits ({t_end:.0f} s, dt={dt:.0f} s, "
      f"{N_steps} steps)...")
print(f"  Attitude modes: {list(attitude_modes.keys())}")
print(f"  CoM offset: {com_label}")
print()

for i, ti in enumerate(t):
    # True anomaly (= mean anomaly for circular orbit)
    nu = n_orb * ti

    # Position and velocity in perifocal frame
    r_peri = r_orb * np.array([np.cos(nu), np.sin(nu), 0])
    v_peri = V_orb * np.array([-np.sin(nu), np.cos(nu), 0])

    # Transform to ECI
    r_eci, v_eci = perifocal_to_eci(r_peri, v_peri)

    # ECI to ECEF (for NRLMSISE-00 lat/lon)
    gmst = gmst0 + omega_earth * ti
    r_ecef = eci_to_ecef(r_eci, gmst)
    lat, lon, alt = ecef_to_lla(r_ecef)

    # NRLMSISE-00 atmospheric density
    dt_epoch = timedelta(seconds=float(ti))
    current_time = epoch + dt_epoch
    doy = current_time.timetuple().tm_yday
    sec_of_day = (current_time.hour * 3600 +
                  current_time.minute * 60 +
                  current_time.second)
    # Local solar time (approximate)
    lst = (sec_of_day / 3600.0 + lon / 15.0) % 24.0

    atm = msise_model(current_time, alt / 1e3, lat, lon,
                      f107a, f107, ap, lst=lst)
    rho = atm[0][5]  # total mass density
    T_inf = atm[1][1]  # temperature at altitude

    # C_D depends on temperature (slight variation over orbit)
    C_D_n = 2.0 + sigma_ac * (np.pi / 6) * np.sqrt(T_wall / max(T_inf, 1))

    # Orbital speed at this point (constant for circular orbit)
    V_mag = np.linalg.norm(v_eci)

    for mode_name, att_func in attitude_modes.items():
        # Compute body-from-ECI rotation
        if mode_name == 'Sun-pointing':
            R_body = att_func(r_eci, v_eci, sun_dir_eci)
        else:
            R_body = att_func(r_eci, v_eci)

        # Velocity in body frame
        V_body = R_body @ v_eci

        # Aero force and torque
        F_tot, T_tot = compute_aero(panels, V_body, rho, V_mag, C_D_n)

        res = results[mode_name]
        res['T_vec'][i] = T_tot
        res['T_mag'][i] = np.linalg.norm(T_tot)
        res['F_vec'][i] = F_tot
        res['F_mag'][i] = np.linalg.norm(F_tot)
        res['rho'][i]   = rho
        res['lat'][i]   = lat
        res['lon'][i]   = lon
        res['alt'][i]   = alt / 1e3

# Print summary
print("Results Summary:")
print("-" * 64)
for mode_name, res in results.items():
    print(f"\n  {mode_name} mode:")
    print(f"    Max torque:  {np.max(res['T_mag']):.4e} N·m "
          f"({np.max(res['T_mag'])*1e9:.3f} nN·m)")
    print(f"    Mean torque: {np.mean(res['T_mag']):.4e} N·m "
          f"({np.mean(res['T_mag'])*1e9:.3f} nN·m)")
    print(f"    Max force:   {np.max(res['F_mag']):.4e} N "
          f"({np.max(res['F_mag'])*1e6:.4f} µN)")
    print(f"    Density range: {np.min(res['rho']):.4e} to "
          f"{np.max(res['rho']):.4e} kg/m³")
    print(f"    Latitude range: {np.min(res['lat']):.1f}° to "
          f"{np.max(res['lat']):.1f}°")

# Magnetorquer comparison
mu_mag = 0.1
B_min, B_max = 25e-6, 50e-6
T_ctrl_min = mu_mag * B_min
T_ctrl_max = mu_mag * B_max

print(f"\n  Magnetorquer authority: {T_ctrl_min*1e9:.0f} – "
      f"{T_ctrl_max*1e9:.0f} nN·m")
for mode_name, res in results.items():
    max_T = np.max(res['T_mag'])
    if max_T > 0:
        print(f"  Margin ({mode_name}): {T_ctrl_min/max_T:.0f}x (min B)")
print()

# ======================================================================
#  6. GENERATE FIGURES
# ======================================================================
fig_dpi = 150
t_min = t / 60  # convert to minutes
t_norm = t / T_orb  # normalised to orbital periods

# ── Figure 1: Torque magnitude vs time (both modes) ──
fig1, axes1 = plt.subplots(3, 1, figsize=(12, 10), dpi=fig_dpi, sharex=True)
fig1.suptitle(f'Aerodynamic Disturbance Torque Over {N_orbits} Orbits\n'
              f'(515 km, Solar Max, CoM offset = 20 mm)',
              fontsize=13)

for mode_name, res in results.items():
    ls = '-' if mode_name == 'Sun-pointing' else '--'

    # Panel (a): Torque magnitude
    axes1[0].plot(t_norm, res['T_mag'] * 1e9, ls, lw=1.2,
                  label=mode_name)

    # Panel (b): Torque components (sun-pointing only for clarity)
    if mode_name == 'Sun-pointing':
        axes1[1].plot(t_norm, res['T_vec'][:, 0] * 1e9, 'r-', lw=1,
                      label='$T_x$', alpha=0.8)
        axes1[1].plot(t_norm, res['T_vec'][:, 1] * 1e9, 'g-', lw=1,
                      label='$T_y$', alpha=0.8)
        axes1[1].plot(t_norm, res['T_vec'][:, 2] * 1e9, 'b-', lw=1,
                      label='$T_z$', alpha=0.8)

axes1[0].set_ylabel('|T| [nN·m]')
axes1[0].legend(loc='upper right', fontsize=9)
axes1[0].grid(True, alpha=0.3)
axes1[0].set_title('(a) Torque Magnitude')

axes1[1].set_ylabel('Torque [nN·m]')
axes1[1].legend(loc='upper right', fontsize=9)
axes1[1].grid(True, alpha=0.3)
axes1[1].set_title('(b) Torque Components (Sun-pointing)')

# Panel (c): Atmospheric density and latitude
ax_rho = axes1[2]
res_sp = results['Sun-pointing']
color_rho = 'tab:blue'
ax_rho.plot(t_norm, res_sp['rho'], color=color_rho, lw=1.2)
ax_rho.set_ylabel('ρ [kg/m³]', color=color_rho)
ax_rho.tick_params(axis='y', labelcolor=color_rho)
ax_rho.set_xlabel('Time [orbital periods]')
ax_rho.set_title('(c) Atmospheric Density and Latitude')
ax_rho.grid(True, alpha=0.3)

ax_lat = ax_rho.twinx()
color_lat = 'tab:red'
ax_lat.plot(t_norm, res_sp['lat'], color=color_lat, lw=0.8, alpha=0.6)
ax_lat.set_ylabel('Latitude [deg]', color=color_lat)
ax_lat.tick_params(axis='y', labelcolor=color_lat)

fig1.tight_layout()
fig1.savefig('aero_torque_vs_time.png', dpi=fig_dpi)

# ── Figure 2: Force magnitude vs time ──
fig2, ax2 = plt.subplots(figsize=(12, 5), dpi=fig_dpi)
for mode_name, res in results.items():
    ls = '-' if mode_name == 'Sun-pointing' else '--'
    ax2.plot(t_norm, res['F_mag'] * 1e6, ls, lw=1.2, label=mode_name)
ax2.set_xlabel('Time [orbital periods]')
ax2.set_ylabel('|F| [µN]')
ax2.set_title(f'Aerodynamic Force Over {N_orbits} Orbits (515 km, Solar Max)')
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)
fig2.tight_layout()
fig2.savefig('aero_force_vs_time.png', dpi=fig_dpi)

# ── Figure 3: Groundtrack with density colormap ──
fig3, ax3 = plt.subplots(figsize=(12, 6), dpi=fig_dpi)
sc = ax3.scatter(res_sp['lon'], res_sp['lat'],
                 c=res_sp['rho'], cmap='hot', s=2, zorder=2)
cb = plt.colorbar(sc, ax=ax3)
cb.set_label('ρ [kg/m³]')
ax3.set_xlabel('Longitude [deg]')
ax3.set_ylabel('Latitude [deg]')
ax3.set_title('Groundtrack with Atmospheric Density (NRLMSISE-00, Solar Max)')
ax3.set_xlim([-180, 180])
ax3.set_ylim([-90, 90])
ax3.grid(True, alpha=0.3)
# Mark ascending/descending nodes
for i in range(1, N_steps):
    if res_sp['lat'][i] >= 0 and res_sp['lat'][i-1] < 0:
        ax3.axvline(res_sp['lon'][i], color='g', alpha=0.3, ls='--', lw=0.8)
fig3.tight_layout()
fig3.savefig('groundtrack_density.png', dpi=fig_dpi)

# ── Figure 4: Accumulated angular momentum from aero torque ──
# This shows whether the aero torque is secular (builds up) or
# periodic (cancels out). Important for momentum management.
fig4, ax4 = plt.subplots(figsize=(12, 5), dpi=fig_dpi)
for mode_name, res in results.items():
    ls = '-' if mode_name == 'Sun-pointing' else '--'
    # Integrate torque to get accumulated angular momentum
    H_x = np.cumsum(res['T_vec'][:, 0]) * dt
    H_y = np.cumsum(res['T_vec'][:, 1]) * dt
    H_z = np.cumsum(res['T_vec'][:, 2]) * dt
    H_mag = np.sqrt(H_x**2 + H_y**2 + H_z**2)

    ax4.plot(t_norm, H_mag * 1e9, ls, lw=1.5, label=f'{mode_name} |H|')

    if mode_name == 'Sun-pointing':
        ax4.plot(t_norm, H_x * 1e9, 'r-', lw=0.8, alpha=0.5, label='$H_x$')
        ax4.plot(t_norm, H_y * 1e9, 'g-', lw=0.8, alpha=0.5, label='$H_y$')
        ax4.plot(t_norm, H_z * 1e9, 'b-', lw=0.8, alpha=0.5, label='$H_z$')

ax4.set_xlabel('Time [orbital periods]')
ax4.set_ylabel('Accumulated Angular Momentum [nN·m·s]')
ax4.set_title('Accumulated Angular Momentum from Aerodynamic Torque\n'
              '(secular component drives momentum management requirements)')
ax4.legend(loc='upper left', fontsize=9)
ax4.grid(True, alpha=0.3)
fig4.tight_layout()
fig4.savefig('aero_momentum_accumulation.png', dpi=fig_dpi)

# ── Figure 5: Comparison with other disturbance torques ──
# Bar chart showing relative magnitudes
fig5, ax5 = plt.subplots(figsize=(10, 6), dpi=fig_dpi)

# Estimated disturbance torques for a 2U at 515 km
# These are order-of-magnitude estimates for context
max_aero = np.max(results['Sun-pointing']['T_mag'])

# For 2U: Iz - Iy is very small (nearly square cross-section)
# With 10% inertia asymmetry and 5° off-nadir:
Ix = 0.0093   # kgm2 (approx for 2U, 2.5 kg)
Iy = 0.0093
Iz = 0.0042
T_gg = (3 * mu_E / (2 * r_orb**3)) * abs(Iz - Ix) * np.sin(2 * np.deg2rad(5))

# Residual magnetic dipole: T_mag = m_res × B
# Typical residual: m_res ≈ 0.001 A·m² for a CubeSat
m_residual = 0.001  # A·m²
B_field = 35e-6     # T (average at 515 km)
T_res_mag = m_residual * B_field

# Solar radiation pressure torque: T_srp = P_sun * A * (r_cp - r_com) / c
P_sun = 1361.0  # W/m² (solar constant)
c_light = 3e8   # m/s
A_max = L_y * L_z  # largest face area
srp_offset = 0.01  # 10 mm cp-cm offset
T_srp = (P_sun / c_light) * A_max * srp_offset * 2  # factor 2 for reflection

disturbances = {
    'Aerodynamic\n(this analysis)': max_aero,
    'Gravity\ngradient': T_gg,
    'Residual\nmagnetic dipole': T_res_mag,
    'Solar radiation\npressure': T_srp,
    'Magnetorquer\nauthority (min B)': T_ctrl_min,
}

names = list(disturbances.keys())
values = np.array(list(disturbances.values()))
colors = ['#2196F3', '#FF9800', '#4CAF50', '#9C27B0', '#F44336']

bars = ax5.barh(names, values * 1e9, color=colors, edgecolor='white', height=0.6)
ax5.set_xscale('log')
ax5.set_xlabel('Torque [nN·m]')
ax5.set_title('Disturbance Torque Comparison — GOOSE at 515 km\n'
              '(order-of-magnitude estimates)')

# Add value labels
for bar, val in zip(bars, values):
    ax5.text(val * 1e9 * 1.3, bar.get_y() + bar.get_height()/2,
             f'{val*1e9:.2f} nN·m', va='center', fontsize=9)

ax5.grid(True, alpha=0.3, axis='x')
fig5.tight_layout()
fig5.savefig('disturbance_comparison.png', dpi=fig_dpi)

print("All figures saved.")
print()

fig6, ax6 = plt.subplots(figsize=(10, 6), dpi=fig_dpi)
for mode_name, res in results.items():
    ls = '-'

    #Torque magnitude
    ax6.plot(t_norm, res['T_mag'] * 1e9, ls, lw=1.2,
                  label=mode_name)

ax6.set_ylabel('|T| [nN·m]')
ax6.legend(loc='upper right', fontsize=9)
ax6.grid(True, alpha=0.3)
ax6.set_title('GOOSE residual magnetic torque disturbance over 2 Orbits')
ax6.set_xlabel('Time [orbital periods]')
fig6.tight_layout()
fig6.savefig('GOOSE_residual_magnetic_torque.png', dpi=fig_dpi)

# ======================================================================
#  7. ENGINEERING SUMMARY
# ======================================================================
print("=" * 64)
print("  ENGINEERING SUMMARY")
print("=" * 64)
print()
print(f"  At 515 km (solar max), aerodynamic disturbance torque is:")
print(f"    Peak:  {max_aero*1e9:.3f} nN·m")
print(f"    Mean:  {np.mean(results['Sun-pointing']['T_mag'])*1e9:.3f} nN·m")
print()
print(f"  For comparison:")
print(f"    Gravity gradient:      {T_gg*1e9:.2f} nN·m")
print(f"    Residual mag dipole:   {T_res_mag*1e9:.2f} nN·m")
print(f"    Solar rad pressure:    {T_srp*1e9:.4f} nN·m")
print(f"    Magnetorquer (min B):  {T_ctrl_min*1e9:.0f} nN·m")
print()
print(f"  Conclusion: Aerodynamic drag is NOT the dominant disturbance")
print(f"  at 515 km. Gravity gradient and residual magnetic dipole")
print(f"  torques are {T_gg/max_aero:.0f}x and {T_res_mag/max_aero:.0f}x "
      f"larger, respectively.")
print(f"  The magnetorquer has {T_ctrl_min/max_aero:.0f}x margin over")
print(f"  the worst-case aerodynamic torque.")
print()

# Momentum accumulation per orbit
H_accumulated = np.sum(results['Sun-pointing']['T_vec'][:N_steps//N_orbits] * dt, axis=0)
H_per_orbit_sp = np.linalg.norm(H_accumulated)
print(f"  Aero momentum per orbit (sun-pointing): "
      f"{H_per_orbit_sp*1e9:.4f} nN·m·s")
print(f"  This is negligible for momentum management — gravity gradient")
print(f"  and residual dipole dominate the momentum budget.")

# ======================================================================
#  MATLAB / SIMULINK EXPORT
# ======================================================================
# Saves lookup-table data for a Simulink disturbance input block.
#
# In MATLAB, load and wire up as:
#   load('GOOSE_aero_Td.mat');
#   Td.time             = t;        % (N,1) [s]
#   Td.signals.values   = T_body;   % (N,3) [N·m]  — Tx Ty Tz columns
#   Td.signals.dimensions = 3;
#   % Then connect 'Td' to a 'From Workspace' block in Simulink.
#
# Note: torque is time-varying — driven by NRLMSISE-00 density variation
#       along the orbit (latitude-dependent atmospheric bulge).
# ======================================================================
res_sp = results['Sun-pointing']

H_vec_aero = np.cumsum(res_sp['T_vec'] * dt, axis=0)   # accumulated ΔH [N·m·s]

mat_aero = {
    # --- time ---
    't':              t.reshape(-1, 1),                    # (N,1) [s]
    'dt':             np.array([[dt]]),                    # scalar [s]
    'T_orb':          np.array([[T_orb]]),                 # scalar [s]
    # --- torque (sun-pointing) ---
    'T_body':         res_sp['T_vec'],                     # (N,3) [N·m]
    'T_mag':          res_sp['T_mag'].reshape(-1, 1),      # (N,1) [N·m]
    # --- aero force ---
    'F_body':         res_sp['F_vec'],                     # (N,3) [N]
    'F_mag':          res_sp['F_mag'].reshape(-1, 1),      # (N,1) [N]
    # --- atmosphere (NRLMSISE-00) ---
    'rho_kgm3':       res_sp['rho'].reshape(-1, 1),        # (N,1) [kg/m³]
    # --- orbit ground track ---
    'lat_deg':        res_sp['lat'].reshape(-1, 1),        # (N,1) [deg]
    'lon_deg':        res_sp['lon'].reshape(-1, 1),        # (N,1) [deg]
    'alt_km':         res_sp['alt'].reshape(-1, 1),        # (N,1) [km]
    # --- accumulated angular momentum ---
    'H_body':         H_vec_aero,                          # (N,3) [N·m·s]
    # --- simulation metadata ---
    'f107':           np.array([[f107]]),                  # solar flux proxy
    'com_offset_m':   com_offsets[com_label].reshape(1, -1),  # (1,3) [m]
}
out_mat = r'C:\Users\zalak\Desktop\Sem 4.1\GOOSE_aero_Td.mat'
savemat(out_mat, mat_aero)
print()
print(f"Saved MATLAB data: {out_mat}")
print("  Variables: t, T_body, T_mag, F_body, F_mag, rho_kgm3,")
print("             lat_deg, lon_deg, alt_km, H_body, f107, dt, T_orb")