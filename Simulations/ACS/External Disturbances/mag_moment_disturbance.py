#!/usr/bin/env python3
"""
GOOSE 2U CubeSat — Residual Magnetic Moment Disturbance Torque
==============================================================
Propagates a circular dawn-to-dusk SSO at 515 km and computes the
magnetic disturbance torque  T = m_res × B  over 2 orbital periods.

Model
-----
  Earth B-field  : IGRF-14 (ppigrf), evaluated at each orbital position
                   via geocentric spherical → ECI conversion.
                   (IGRF-14 is the current generation; coefficients are
                   identical to IGRF-13 for epochs before 2020.)
  Residual moment: permanent hardware bias (constant, harmonics neglected)
  Attitude        : sun-pointing (primary) and nadir-pointing (comparison)

Key physics
-----------
  1. |B| is ~2× stronger at the magnetic poles than the magnetic equator,
     so the torque peaks as the satellite passes high latitudes.
  2. The direction of B in the body frame rotates continuously, giving a
     periodic torque in each body axis that partially cancels over one orbit.
  3. A secular (non-cancelling) residual accumulates due to the 11.5° tilt
     of the geomagnetic dipole relative to Earth's spin axis.
  4. Dawn-dusk SSO ≈ constant sun exposure → small current-loop variation.

Dependencies
------------
  pip install ppigrf numpy scipy matplotlib
"""

import sys
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from scipy.io import savemat
import ppigrf

# ======================================================================
#  1. PHYSICAL CONSTANTS
# ======================================================================
RE       = 6.3781e6           # Earth mean radius [m]
MU_E     = 3.986004418e14     # Gravitational parameter [m³/s²]

# Epoch for IGRF evaluation — set to approximate GOOSE mission date
IGRF_DATE = pd.Timestamp('2026-01-01')

# ======================================================================
#  2. ORBITAL PARAMETERS — GOOSE 2U at 515 km dawn-dusk SSO
# ======================================================================
h_orbit = 515e3                               # altitude [m]
r_orb   = RE + h_orbit                        # orbital radius [m]
V_orb   = np.sqrt(MU_E / r_orb)              # orbital speed [m/s]
T_orb   = 2*np.pi * np.sqrt(r_orb**3 / MU_E) # period [s]
n_orb   = 2*np.pi / T_orb                    # mean motion [rad/s]
inc     = np.radians(97.4)                    # SSO inclination [rad]
RAAN    = np.radians(90.0)                    # ascending node at dawn [rad]
argp    = np.radians(0.0)                     # argument of periapsis

print("=" * 64)
print("  GOOSE 2U — Residual Magnetic Moment Disturbance Torque")
print("           (IGRF-14 geomagnetic field model)")
print("=" * 64)
print(f"\n  Altitude:           {h_orbit/1e3:.0f} km")
print(f"  Orbital period:     {T_orb:.1f} s  ({T_orb/60:.2f} min)")
print(f"  Orbital speed:      {V_orb:.1f} m/s")
print(f"  Inclination:        {np.degrees(inc):.1f}°  (SSO, dawn-dusk)")
print(f"  IGRF epoch:         {IGRF_DATE.date()}")
print()

# ======================================================================
#  3. ORBITAL MECHANICS
# ======================================================================
def perifocal_to_eci(nu, r_mag, v_mag):
    """
    Convert Keplerian state at true anomaly nu to ECI frame vectors.

    Returns
    -------
    r_eci : (3,) ndarray  position  [m]
    v_eci : (3,) ndarray  velocity  [m/s]
    """
    cO, sO = np.cos(RAAN), np.sin(RAAN)
    ci, si = np.cos(inc),  np.sin(inc)
    cw, sw = np.cos(argp), np.sin(argp)

    r_p = r_mag * np.array([ np.cos(nu),  np.sin(nu), 0.0])
    v_p = v_mag * np.array([-np.sin(nu),  np.cos(nu), 0.0])

    # DCM: perifocal → ECI  (R3(-RAAN) R1(-inc) R3(-argp))
    R = np.array([
        [ cO*cw - sO*ci*sw,  -(cO*sw + sO*ci*cw),   sO*si ],
        [ sO*cw + cO*ci*sw,  -(sO*sw - cO*ci*cw),  -cO*si ],
        [ si*sw,               si*cw,                ci    ]
    ])
    return R @ r_p, R @ v_p

# ======================================================================
#  4. EARTH MAGNETIC FIELD — IGRF-14 (real spherical harmonic model)
# ======================================================================

def eci_to_geocentric_spherical(r_eci):
    """
    Convert ECI Cartesian position to geocentric spherical coordinates.

    The ECI frame here is J2000-like with the vernal equinox along +X
    and the north pole along +Z.  For the purposes of IGRF evaluation we
    treat the orbit as fixed in inertial space (i.e. we do not rotate
    through Greenwich Sidereal Time), which is the standard assumption
    when RAAN is already defined in ECI.  Longitude is therefore the
    right ascension of the sub-satellite point, NOT geographic longitude;
    this is acceptable because ppigrf evaluates the full spherical
    harmonic model and the small error from ignoring Earth's rotation
    during 2 orbits (~3°) is negligible for disturbance torque purposes.

    Returns
    -------
    r_km   : geocentric radius [km]
    colat  : geocentric co-latitude [deg]   (0° at N pole, 90° at equator)
    lon    : right-ascension-based longitude [deg]
    """
    x, y, z = r_eci
    r     = np.sqrt(x**2 + y**2 + z**2)
    colat = np.degrees(np.arccos(np.clip(z / r, -1.0, 1.0)))  # [deg]
    lon   = np.degrees(np.arctan2(y, x))                       # [deg]
    return r / 1e3, colat, lon


def geocentric_spherical_to_eci_matrix(colat_rad, lon_rad):
    """
    Build the 3×3 rotation matrix that converts a vector expressed in the
    geocentric spherical basis (r̂, θ̂, φ̂) to the ECI Cartesian basis.

      r̂   = outward radial
      θ̂   = southward  (increasing colatitude)
      φ̂   = eastward   (increasing longitude)

    The columns of R are these three unit vectors expressed in ECI.

    Parameters
    ----------
    colat_rad : geocentric co-latitude [rad]
    lon_rad   : longitude [rad]

    Returns
    -------
    R : (3, 3) ndarray   such that  v_eci = R @ v_sph
    """
    ct, st = np.cos(colat_rad), np.sin(colat_rad)
    cp, sp = np.cos(lon_rad),   np.sin(lon_rad)

    r_hat  = np.array([ st*cp,  st*sp,  ct])   # radial outward
    th_hat = np.array([ ct*cp,  ct*sp, -st])   # southward
    ph_hat = np.array([-sp,     cp,     0.0])  # eastward

    return np.column_stack([r_hat, th_hat, ph_hat])  # (3,3)


def get_B_eci(r_eci):
    """
    Geomagnetic field at ECI position r_eci using the IGRF-14 model.

    Steps
    -----
    1. Convert r_eci to geocentric spherical (r, colat, lon).
    2. Query ppigrf.igrf_gc → (Br, Btheta, Bphi) in nT
       where Br is radially outward, Btheta is southward, Bphi is eastward.
    3. Rotate the spherical-frame vector into ECI.

    Returns
    -------
    B_eci : (3,) ndarray  geomagnetic field in ECI frame [T]
    """
    r_km, colat, lon = eci_to_geocentric_spherical(r_eci)

    Br, Bth, Bph = ppigrf.igrf_gc(
        np.array([r_km]),
        np.array([colat]),
        np.array([lon]),
        IGRF_DATE,
    )
    # igrf_gc returns (N_dates, N_points) arrays; extract scalar [nT] → [T]
    Br_T  = float(Br[0, 0])  * 1e-9
    Bth_T = float(Bth[0, 0]) * 1e-9
    Bph_T = float(Bph[0, 0]) * 1e-9

    # Rotate spherical (r, θ, φ) components into ECI
    colat_rad = np.radians(colat)
    lon_rad   = np.radians(lon)
    R_sph2eci = geocentric_spherical_to_eci_matrix(colat_rad, lon_rad)

    B_eci = R_sph2eci @ np.array([Br_T, Bth_T, Bph_T])
    return B_eci


def magnetic_latitude(r_eci):
    """
    Approximate magnetic latitude using the IGRF dipole axis direction.

    The IGRF dipole (g10, g11, h11 coefficients) points roughly 11.5°
    from the geographic pole.  We project r_eci onto that axis to get
    a useful latitude proxy for plotting (not used in torque calculation).
    """
    # IGRF-14 dipole unit vector (geocentric north magnetic pole ~86.5°N, 162°E)
    pole_colat = np.radians(3.5)    # 90° - 86.5°
    pole_lon   = np.radians(162.0)
    m_hat = np.array([
        np.sin(pole_colat) * np.cos(pole_lon),
        np.sin(pole_colat) * np.sin(pole_lon),
        np.cos(pole_colat),
    ])
    sin_lam = np.dot(r_eci / np.linalg.norm(r_eci), m_hat)
    return np.degrees(np.arcsin(np.clip(sin_lam, -1.0, 1.0)))

# ======================================================================
#  5. RESIDUAL MAGNETIC MOMENT — GOOSE 2U MODEL
# ======================================================================
# Conservative constant dipole moment estimate (~0.07 A·m² total).
# Per-axis values representative of a 2U CubeSat with COTS components.
# Harmonics are neglected; this is the dominant permanent bias term.

M_PERM = np.array([0.0364, 0.0409, 0.0432])  # [A·m²] permanent bias, body frame


def residual_moment_body(_t):
    """
    Residual magnetic moment in body frame [A·m²].

    Constant conservative estimate — harmonics neglected.
    Returns m_res as a (3,) ndarray.
    """
    return M_PERM

# ======================================================================
#  6. ATTITUDE MODELS
# ======================================================================
# Sun direction in ECI (approximately along +X at June solstice).
SUN_ECI = np.array([1.0, 0.0, 0.0])


def dcm_sun_pointing(r_eci, v_eci):
    """
    Body-from-ECI DCM for sun-pointing attitude.
    +X_body → Sun,  +Z_body → velocity direction (ram).
    Rows are body-frame basis vectors expressed in ECI.
    """
    x_b = SUN_ECI.copy()
    v_hat = v_eci / np.linalg.norm(v_eci)
    z_b = v_hat - np.dot(v_hat, x_b) * x_b
    nz = np.linalg.norm(z_b)
    z_b = z_b / nz if nz > 1e-10 else np.array([0.0, 0.0, 1.0])
    y_b = np.cross(z_b, x_b)
    y_b /= np.linalg.norm(y_b)
    return np.vstack([x_b, y_b, z_b])      # R:  v_body = R @ v_eci


def dcm_nadir_pointing(r_eci, v_eci):
    """
    Body-from-ECI DCM for nadir-pointing (LVLH) attitude.
    +Z_body → nadir,  +X_body → velocity (ram).
    """
    z_b = -r_eci / np.linalg.norm(r_eci)   # nadir
    x_b = v_eci / np.linalg.norm(v_eci)
    x_b = x_b - np.dot(x_b, z_b) * z_b
    x_b /= np.linalg.norm(x_b)
    y_b = np.cross(z_b, x_b)
    return np.vstack([x_b, y_b, z_b])

# ======================================================================
#  7. TIME-DOMAIN SIMULATION — 2 ORBITS
# ======================================================================
N_orbits = 2
dt       = 10.0                               # timestep [s]
t_arr    = np.arange(0.0, N_orbits * T_orb, dt)
N        = len(t_arr)

attitude_modes = {
    'Sun-pointing':   dcm_sun_pointing,
    'Nadir-pointing': dcm_nadir_pointing,
}

store = {
    mode: {
        'T_vec':   np.zeros((N, 3)),
        'T_mag':   np.zeros(N),
        'm_vec':   np.zeros((N, 3)),
        'm_mag':   np.zeros(N),
        'B_eci':   np.zeros((N, 3)),
        'B_body':  np.zeros((N, 3)),
        'B_mag':   np.zeros(N),
        'mag_lat': np.zeros(N),
    }
    for mode in attitude_modes
}

print(f"Simulating {N_orbits} orbits  ({N} steps, dt = {dt:.0f} s)  "
      f"using IGRF-14...")
print("  (This takes a few seconds — one IGRF query per timestep.)")

for i, t in enumerate(t_arr):
    if i % 200 == 0:
        print(f"  Step {i:4d}/{N}  t = {t/60:.1f} min")

    nu           = n_orb * t
    r_eci, v_eci = perifocal_to_eci(nu, r_orb, V_orb)
    B_eci        = get_B_eci(r_eci)           # IGRF-14 field in ECI [T]
    lam_m        = magnetic_latitude(r_eci)

    for mode, dcm_func in attitude_modes.items():
        R = dcm_func(r_eci, v_eci)            # body-from-ECI rotation matrix

        m_body = residual_moment_body(t)      # residual moment in body frame [A·m²]
        B_body = R @ B_eci                    # geomagnetic field in body frame [T]

        T_body = np.cross(m_body, B_body)     # magnetic disturbance torque [N·m]

        s = store[mode]
        s['T_vec'][i]   = T_body
        s['T_mag'][i]   = np.linalg.norm(T_body)
        s['m_vec'][i]   = m_body
        s['m_mag'][i]   = np.linalg.norm(m_body)
        s['B_eci'][i]   = B_eci
        s['B_body'][i]  = B_body
        s['B_mag'][i]   = np.linalg.norm(B_eci)
        s['mag_lat'][i] = lam_m

print()

# ======================================================================
#  8. SUMMARY STATISTICS
# ======================================================================
print("\nResults Summary:")
print("-" * 64)
for mode, s in store.items():
    H_all = np.cumsum(s['T_vec'] * dt, axis=0)
    N1    = N // N_orbits
    H_1   = np.linalg.norm(H_all[N1 - 1])
    H_2   = np.linalg.norm(H_all[-1])
    print(f"\n  {mode} mode:")
    print(f"    |B| range:          {np.min(s['B_mag'])*1e6:.2f} – "
          f"{np.max(s['B_mag'])*1e6:.2f} µT")
    print(f"    |m_res| range:      {np.min(s['m_mag'])*1e3:.3f} – "
          f"{np.max(s['m_mag'])*1e3:.3f} mA·m²")
    print(f"    Peak torque:        {np.max(s['T_mag'])*1e9:.4f} nN·m")
    print(f"    Mean torque:        {np.mean(s['T_mag'])*1e9:.4f} nN·m")
    print(f"    dH after 1 orbit:   {H_1*1e6:.4f} µN·m·s")
    print(f"    dH after 2 orbits:  {H_2*1e6:.4f} µN·m·s")

# ======================================================================
#  9. FIGURES
# ======================================================================
fig_dpi = 150
t_norm  = t_arr / T_orb
sp      = store['Sun-pointing']
np_     = store['Nadir-pointing']

# ── Figure 1 : Overview — B field / residual moment / torque ──────────
fig1 = plt.figure(figsize=(13, 12), dpi=fig_dpi)
gs   = GridSpec(4, 1, figure=fig1, hspace=0.50)
ax_B = fig1.add_subplot(gs[0])
ax_m = fig1.add_subplot(gs[1], sharex=ax_B)
ax_T = fig1.add_subplot(gs[2], sharex=ax_B)
ax_L = fig1.add_subplot(gs[3], sharex=ax_B)

fig1.suptitle(
    'GOOSE 2U CubeSat — Residual Magnetic Moment Disturbance Torque\n'
    '515 km Dawn-Dusk SSO  |  Sun-Pointing  |  2 Orbital Periods\n'
    '(Geomagnetic field: IGRF-14)',
    fontsize=13)

# (a) Geomagnetic field magnitude — real IGRF values
ax_B.plot(t_norm, sp['B_mag']*1e6, '#1f77b4', lw=1.5, label='|B| IGRF-14 at satellite')
ax_B.fill_between(t_norm, sp['B_mag']*1e6, alpha=0.15, color='#1f77b4')
ax_B.set_ylabel('|B|  [µT]')
ax_B.set_title('(a)  Geomagnetic Field Magnitude Along Orbit (IGRF-14)', fontsize=10)
ax_B.legend(fontsize=9)
ax_B.grid(True, alpha=0.3)

# (b) Residual magnetic moment
ax_m.plot(t_norm, sp['m_mag']*1e3, 'k-',  lw=1.5, label='|m_res|', zorder=3)
ax_m.plot(t_norm, sp['m_vec'][:, 0]*1e3, 'r--', lw=0.9, alpha=0.7, label='$m_x$')
ax_m.plot(t_norm, sp['m_vec'][:, 1]*1e3, 'g--', lw=0.9, alpha=0.7, label='$m_y$')
ax_m.plot(t_norm, sp['m_vec'][:, 2]*1e3, 'b--', lw=0.9, alpha=0.7, label='$m_z$')
ax_m.axhline(np.linalg.norm(M_PERM)*1e3, color='k', lw=0.7, ls=':',
             alpha=0.5, label='Permanent bias |m_perm|')
ax_m.set_ylabel('m  [mA·m²]')
ax_m.set_title('(b)  Residual Magnetic Moment (body frame, permanent bias)',
               fontsize=10)
ax_m.legend(fontsize=8, ncol=3, loc='upper right')
ax_m.grid(True, alpha=0.3)

# (c) Disturbance torque magnitude — both modes
ax_T.plot(t_norm, sp['T_mag']*1e9,  '#2ca02c', lw=1.5,
          label='Sun-pointing  T = m × B')
ax_T.plot(t_norm, np_['T_mag']*1e9, '#2ca02c', lw=1.0, ls='--', alpha=0.6,
          label='Nadir-pointing  T = m × B')
ax_T.fill_between(t_norm, sp['T_mag']*1e9, alpha=0.12, color='#2ca02c')
ax_T.set_ylabel('|T|  [nN·m]')
ax_T.set_title('(c)  Magnetic Disturbance Torque  T = m_res × B  (IGRF-14)', fontsize=10)
ax_T.legend(fontsize=9)
ax_T.grid(True, alpha=0.3)

# (d) Magnetic latitude
ax_L.plot(t_norm, sp['mag_lat'], '#8c564b', lw=1.2, label='IGRF-14 dipole magnetic latitude')
ax_L.axhline(0, color='k', lw=0.5, ls=':')
ax_L.set_ylabel('Magnetic\nlatitude [°]')
ax_L.set_xlabel('Time  [orbital periods]')
ax_L.set_title('(d)  Magnetic Latitude Along Orbit  '
               '(torque peaks at high |λ_m|)', fontsize=10)
ax_L.legend(fontsize=9)
ax_L.grid(True, alpha=0.3)

# Mark orbit boundary
for ax in (ax_B, ax_m, ax_T, ax_L):
    ax.axvline(1.0, color='gray', lw=1.0, ls=':', alpha=0.7)
    ax.text(1.01, 0.92, '← orbit 1 | orbit 2 →',
            transform=ax.get_xaxis_transform(),
            fontsize=7, color='gray')

out1 = 'mag_disturbance_overview.png'
fig1.savefig(out1, dpi=fig_dpi, bbox_inches='tight')
print(f"\nSaved: {out1}")

# ── Figure 2 : Torque components + accumulated angular momentum ───────
fig2, axes2 = plt.subplots(2, 2, figsize=(14, 9), dpi=fig_dpi)
fig2.suptitle(
    'GOOSE 2U — Body-Frame Torque Components and Accumulated Angular Momentum\n'
    '515 km Dawn-Dusk SSO  |  2 Orbital Periods  |  IGRF-14',
    fontsize=12)

comp_labels = ['$T_x$', '$T_y$', '$T_z$']
comp_colors = ['#d62728', '#2ca02c', '#9467bd']

for k, (ax, lbl, col) in enumerate(zip(axes2.flat[:3], comp_labels, comp_colors)):
    ax.plot(t_norm, sp['T_vec'][:, k]*1e9,  color=col, lw=1.3,
            label=f'{lbl} Sun-pointing')
    ax.plot(t_norm, np_['T_vec'][:, k]*1e9, color=col, lw=1.0, ls='--', alpha=0.6,
            label=f'{lbl} Nadir-pointing')
    ax.axhline(0, color='k', lw=0.5, ls=':')
    ax.axvline(1.0, color='gray', lw=0.8, ls=':', alpha=0.6)
    ax.set_ylabel('Torque [nN·m]')
    ax.set_xlabel('Time [orbital periods]')
    ax.set_title(f'Body-frame {lbl} component', fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

# Accumulated angular momentum
ax_H = axes2[1, 1]
H_sp = np.cumsum(sp['T_vec']  * dt, axis=0)
H_np = np.cumsum(np_['T_vec'] * dt, axis=0)

ax_H.plot(t_norm, np.linalg.norm(H_sp, axis=1)*1e6, 'b-',  lw=1.8,
          label='|H|  Sun-pointing')
ax_H.plot(t_norm, np.linalg.norm(H_np, axis=1)*1e6, 'b--', lw=1.2, alpha=0.6,
          label='|H|  Nadir-pointing')
ax_H.plot(t_norm, H_sp[:, 0]*1e6, 'r-', lw=0.8, alpha=0.5, label='$H_x$')
ax_H.plot(t_norm, H_sp[:, 1]*1e6, 'g-', lw=0.8, alpha=0.5, label='$H_y$')
ax_H.plot(t_norm, H_sp[:, 2]*1e6, 'm-', lw=0.8, alpha=0.5, label='$H_z$')
ax_H.axvline(1.0, color='gray', lw=0.8, ls=':', alpha=0.6)
ax_H.set_ylabel('Accumulated ΔH  [µN·m·s]')
ax_H.set_xlabel('Time [orbital periods]')
ax_H.set_title('Accumulated Angular Momentum  ΔH = ∫T dt\n'
               '(non-zero drift = secular component)', fontsize=10)
ax_H.legend(fontsize=8, ncol=2)
ax_H.grid(True, alpha=0.3)

fig2.tight_layout()
out2 = 'mag_torque_components.png'
fig2.savefig(out2, dpi=fig_dpi, bbox_inches='tight')
print(f"Saved: {out2}")

# ── Figure 3 : Latitude correlation and B-body rotation ──────────────
fig3, axes3 = plt.subplots(1, 3, figsize=(16, 5), dpi=fig_dpi)
fig3.suptitle(
    'GOOSE 2U — Geomagnetic Field Characterisation Along Dawn-Dusk SSO\n'
    '(IGRF-14 — full spherical harmonic model, not tilted dipole)',
    fontsize=11)

# Left: |B| vs magnetic latitude
ax3a = axes3[0]
sc = ax3a.scatter(sp['mag_lat'], sp['B_mag']*1e6,
                  c=t_norm, cmap='plasma', s=5, zorder=3, label='Orbit (IGRF-14)')
plt.colorbar(sc, ax=ax3a, label='Time [orbital periods]')
ax3a.set_xlabel('Magnetic latitude  λ_m  [°]')
ax3a.set_ylabel('|B|  [µT]')
ax3a.set_title('Geomagnetic field strength\nvs magnetic latitude (IGRF-14)', fontsize=10)
ax3a.legend(fontsize=8)
ax3a.grid(True, alpha=0.3)

# Middle: disturbance torque vs |B|, coloured by latitude
ax3b = axes3[1]
sc2 = ax3b.scatter(sp['B_mag']*1e6, sp['T_mag']*1e9,
                   c=sp['mag_lat'], cmap='RdBu_r', s=5, zorder=3)
plt.colorbar(sc2, ax=ax3b, label='Magnetic latitude [°]')
ax3b.set_xlabel('|B|  [µT]')
ax3b.set_ylabel('|T|  [nN·m]')
ax3b.set_title('Disturbance torque\nvs field strength (IGRF-14)', fontsize=10)
ax3b.grid(True, alpha=0.3)

# Right: B-field components in body frame for one orbit
ax3c = axes3[2]
idx1 = N // N_orbits
ax3c.plot(t_norm[:idx1], sp['B_body'][:idx1, 0]*1e6, 'r-', lw=1.0, label='$B_x$ body')
ax3c.plot(t_norm[:idx1], sp['B_body'][:idx1, 1]*1e6, 'g-', lw=1.0, label='$B_y$ body')
ax3c.plot(t_norm[:idx1], sp['B_body'][:idx1, 2]*1e6, 'b-', lw=1.0, label='$B_z$ body')
ax3c.axhline(0, color='k', lw=0.5, ls=':')
ax3c.set_xlabel('Time [orbital periods]')
ax3c.set_ylabel('B  [µT]')
ax3c.set_title('B-field components in body frame\n(sun-pointing, orbit 1, IGRF-14)', fontsize=10)
ax3c.legend(fontsize=8)
ax3c.grid(True, alpha=0.3)

fig3.tight_layout()
out3 = 'mag_field_characterisation.png'
fig3.savefig(out3, dpi=fig_dpi, bbox_inches='tight')
print(f"Saved: {out3}")

# Single-panel torque magnitude (sun-pointing only)
fig5, ax5 = plt.subplots(figsize=(10, 6), dpi=fig_dpi)
ax5.plot(t_norm, sp['T_mag']*1e9, lw=1.2,
          label='Sun-pointing  T = m × B  (IGRF-14)')
ax5.set_ylabel('|T|  [nN·m]')
ax5.set_title('GOOSE residual magnetic disturbance torque  T = m_res × B  (IGRF-14)',
              fontsize=10)
ax5.legend(fontsize=9)
ax5.grid(True, alpha=0.3)
ax5.set_xlabel('Time  [orbital periods]')
fig5.tight_layout()
out5 = 'GOOSE_mag_disturbance_torque.png'
fig5.savefig(out5, dpi=fig_dpi, bbox_inches='tight')
print(f"Saved: {out5}")

# ── Engineering summary ───────────────────────────────────────────────
print()
print("=" * 64)
print("  ENGINEERING SUMMARY  (IGRF-14)")
print("=" * 64)
print()
print("  Residual magnetic moment model (GOOSE 2U body frame):")
print(f"    Permanent:  mx = {M_PERM[0]*1e3:.3f},  my = {M_PERM[1]*1e3:.3f},"
      f"  mz = {M_PERM[2]*1e3:.3f}  mA·m²")
print(f"    |m_perm|  = {np.linalg.norm(M_PERM)*1e3:.3f} mA·m²  (constant)")
print()
print("  Geomagnetic field at 515 km (IGRF-14):")
print(f"    |B| min (orbit):  {np.min(sp['B_mag'])*1e6:.2f} µT")
print(f"    |B| max (orbit):  {np.max(sp['B_mag'])*1e6:.2f} µT")
print(f"    |B| mean (orbit): {np.mean(sp['B_mag'])*1e6:.2f} µT")
print()
sp_peak = np.max(sp['T_mag'])
np_peak = np.max(np_['T_mag'])
print("  Disturbance torque  T = m_res × B  (IGRF-14):")
print(f"    Sun-pointing   — Peak: {sp_peak*1e9:.4f} nN·m  |"
      f"  Mean: {np.mean(sp['T_mag'])*1e9:.4f} nN·m")
print(f"    Nadir-pointing — Peak: {np_peak*1e9:.4f} nN·m  |"
      f"  Mean: {np.mean(np_['T_mag'])*1e9:.4f} nN·m")
print()
H1_sp = np.linalg.norm(H_sp[N // N_orbits - 1])
H2_sp = np.linalg.norm(H_sp[-1])
print("  Angular momentum accumulation (sun-pointing):")
print(f"    After orbit 1 : {H1_sp*1e6:.4f} µN·m·s")
print(f"    After orbit 2 : {H2_sp*1e6:.4f} µN·m·s")
drift = abs(H2_sp - H1_sp)
print(f"    Per-orbit drift: {drift*1e6:.4f} µN·m·s  (secular from dipole tilt)")
print()
mu_mag  = 0.166
T_ctrl  = mu_mag * np.mean(sp['B_mag'])
print(f"  Magnetorquer authority (0.1 A·m², mean B IGRF-14):")
print(f"    {T_ctrl*1e9:.1f} nN·m  →  {T_ctrl/sp_peak:.0f}× margin over peak torque")
print()
print("  Note: IGRF-14 captures regional anomalies (e.g. South Atlantic")
print("  Anomaly) that the tilted dipole model misses.  Expect slightly")
print("  different peak values and asymmetry between orbit 1 and orbit 2")
print("  compared to the tilted-dipole baseline.")
print()

# ======================================================================
#  MATLAB / SIMULINK EXPORT
# ======================================================================
sp  = store['Sun-pointing']
np_ = store['Nadir-pointing']

mat_mag = {
    't':              t_arr.reshape(-1, 1),
    'dt':             np.array([[dt]]),
    'T_orb':          np.array([[T_orb]]),
    'T_body_sp':      sp['T_vec'],
    'T_mag_sp':       sp['T_mag'].reshape(-1, 1),
    'T_body_np':      np_['T_vec'],
    'T_mag_np':       np_['T_mag'].reshape(-1, 1),
    'B_eci':          sp['B_eci'],
    'B_mag':          sp['B_mag'].reshape(-1, 1),
    'm_body':         sp['m_vec'],
    'mag_lat_deg':    sp['mag_lat'].reshape(-1, 1),
}
out_mat = 'GOOSE_mag_Td.mat'
savemat(out_mat, mat_mag)
print(f"Saved MATLAB data: {out_mat}")
print("  Variables: t, T_body_sp, T_mag_sp, T_body_np, T_mag_np,")
print("             B_eci, B_mag, m_body, mag_lat_deg, dt, T_orb")
print()
print("All outputs saved to working directory.")