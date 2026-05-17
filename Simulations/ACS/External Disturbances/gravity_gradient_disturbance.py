#!/usr/bin/env python3
"""
GOOSE 2U CubeSat — Gravity-Gradient Disturbance Torque
=======================================================
Propagates a circular dawn-to-dusk SSO at 515 km and computes the
gravity-gradient disturbance torque over 2 orbital periods.
Attitude: sun-pointing (+X_body → Sun, +Z_body → velocity).

Model
-----
  T_gg = 3(μ/r³) · ê_r × (I · ê_r)

  where ê_r is the unit zenith vector (Earth centre → satellite)
  expressed in the body frame, and I is the body-frame inertia tensor.

Worst case
----------
  T_gg_max = (3/2) · n² · (I_max − I_min)

  Achieved when the minimum-inertia axis is exactly 45° from the
  radial (zenith) direction.  This is a hard analytical upper bound
  for any attitude; it does not assume any particular orientation.

Key physics
-----------
  1. For sun-pointing, +X_body (long axis, minimum inertia Ixx) points at
     the Sun.  The angle between this axis and the radial direction varies
     continuously around the orbit → torque varies at ~2× orbital frequency.
  2. The inertia asymmetry (I_max − I_min) sets the scale of the torque.
  3. Dawn-dusk SSO keeps the Sun roughly in the orbit plane, so the
     misalignment angle θ sweeps broadly and can approach 45°.
  4. GG torque is generally much smaller than the magnetic disturbance
     for a typical unshielded 2U with large residual magnetic moment.
"""

import sys
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from scipy.io import savemat

# ======================================================================
#  1. PHYSICAL CONSTANTS
# ======================================================================
RE   = 6.3781e6           # Earth mean radius [m]
MU_E = 3.986004418e14     # Gravitational parameter [m³/s²]

# ======================================================================
#  2. ORBITAL PARAMETERS — GOOSE 2U at 515 km dawn-dusk SSO
# ======================================================================
h_orbit = 515e3
r_orb   = RE + h_orbit
T_orb   = 2*np.pi * np.sqrt(r_orb**3 / MU_E)
n_orb   = 2*np.pi / T_orb
V_orb   = np.sqrt(MU_E / r_orb)
inc     = np.radians(97.4)
RAAN    = np.radians(90.0)
argp    = np.radians(0.0)

print("=" * 64)
print("  GOOSE 2U — Gravity-Gradient Disturbance Torque")
print("=" * 64)
print(f"\n  Altitude:       {h_orbit/1e3:.0f} km")
print(f"  Period:         {T_orb:.1f} s  ({T_orb/60:.2f} min)")
print(f"  Speed:          {V_orb:.1f} m/s")
print(f"  Inclination:    {np.degrees(inc):.1f}°  (SSO, dawn-dusk)")
print()

# ======================================================================
#  3. SPACECRAFT INERTIA — GOOSE 2U
# ======================================================================
# Geometry: 100 mm × 100 mm × 227 mm (standard 2U envelope), mass 2.0 kg
# +X_body = long (2U stack) axis → Sun in sun-pointing mode
# Off-diagonal terms neglected (principal axes aligned with body axes)

MASS = 2.0          # kg
LX   = 0.227        # m  long axis  (+X_body, 2U stack direction)
LY   = 0.100        # m
LZ   = 0.100        # m

Ixx = (1.0/12.0) * MASS * (LY**2 + LZ**2)   # minimum (along long axis)
Iyy = (1.0/12.0) * MASS * (LX**2 + LZ**2)
Izz = (1.0/12.0) * MASS * (LX**2 + LY**2)

I_body = np.diag([Ixx, Iyy, Izz])

print("  Inertia tensor (body frame, uniform rectangular prism):")
print(f"    Ixx = {Ixx*1e3:.4f} ×10⁻³ kg·m²  (long axis +X, minimum)")
print(f"    Iyy = {Iyy*1e3:.4f} ×10⁻³ kg·m²")
print(f"    Izz = {Izz*1e3:.4f} ×10⁻³ kg·m²")
print(f"    I_max − I_min = {(max(Ixx,Iyy,Izz) - min(Ixx,Iyy,Izz))*1e3:.4f} ×10⁻³ kg·m²")
print()

# ======================================================================
#  4. THEORETICAL WORST CASE
# ======================================================================
I_max   = max(Ixx, Iyy, Izz)
I_min   = min(Ixx, Iyy, Izz)
n2_orb  = n_orb**2
T_worst = 1.5 * n2_orb * (I_max - I_min)

print("  Theoretical worst case  T_gg_max = (3/2) · n² · (I_max − I_min):")
print(f"    n             = {n_orb:.6e} rad/s")
print(f"    n²            = {n2_orb:.6e} rad²/s²")
print(f"    I_max − I_min = {(I_max - I_min)*1e3:.4f} ×10⁻³ kg·m²")
print(f"    T_gg_max      = {T_worst*1e9:.4f} nN·m  (at θ = 45°)")
print()

# ======================================================================
#  5. ORBITAL MECHANICS
# ======================================================================
def perifocal_to_eci(nu, r_mag, v_mag):
    """Keplerian state at true anomaly nu → ECI position and velocity."""
    cO, sO = np.cos(RAAN), np.sin(RAAN)
    ci, si = np.cos(inc),  np.sin(inc)
    cw, sw = np.cos(argp), np.sin(argp)
    r_p = r_mag * np.array([ np.cos(nu),  np.sin(nu), 0.0])
    v_p = v_mag * np.array([-np.sin(nu),  np.cos(nu), 0.0])
    R = np.array([
        [ cO*cw - sO*ci*sw,  -(cO*sw + sO*ci*cw),   sO*si ],
        [ sO*cw + cO*ci*sw,  -(sO*sw - cO*ci*cw),  -cO*si ],
        [ si*sw,               si*cw,                ci    ]
    ])
    return R @ r_p, R @ v_p

# ======================================================================
#  6. ATTITUDE MODEL — SUN-POINTING
# ======================================================================
SUN_ECI = np.array([1.0, 0.0, 0.0])   # Sun direction in ECI (June solstice approx)

def dcm_sun_pointing(r_eci, v_eci):
    """
    Body-from-ECI DCM for sun-pointing attitude.
    +X_body → Sun,  +Z_body → velocity (ram).
    Rows are body-frame basis vectors expressed in ECI.
    """
    x_b   = SUN_ECI.copy()
    v_hat = v_eci / np.linalg.norm(v_eci)
    z_b   = v_hat - np.dot(v_hat, x_b) * x_b
    nz    = np.linalg.norm(z_b)
    z_b   = z_b / nz if nz > 1e-10 else np.array([0.0, 0.0, 1.0])
    y_b   = np.cross(z_b, x_b)
    y_b  /= np.linalg.norm(y_b)
    return np.vstack([x_b, y_b, z_b])    # R: v_body = R @ v_eci

# ======================================================================
#  7. GRAVITY-GRADIENT TORQUE
# ======================================================================
def gravity_gradient_torque(r_eci, R):
    """
    Gravity-gradient torque in body frame [N·m].

      T_gg = 3(μ/r³) · ê_r × (I · ê_r)

    ê_r = unit zenith vector (Earth centre → satellite) in body frame.
    """
    r_mag    = np.linalg.norm(r_eci)
    n2_local = MU_E / r_mag**3           # μ/r³  [rad²/s²]
    e_r_body = R @ (r_eci / r_mag)       # zenith unit vector in body frame
    return 3.0 * n2_local * np.cross(e_r_body, I_body @ e_r_body)

# ======================================================================
#  8. SIMULATION — 2 ORBITS
# ======================================================================
N_orbits = 2
dt       = 10.0
t_arr    = np.arange(0.0, N_orbits * T_orb, dt)
N        = len(t_arr)

T_vec = np.zeros((N, 3))   # body-frame torque vector [N·m]
T_mag = np.zeros(N)        # torque magnitude [N·m]
theta = np.zeros(N)        # angle between +X_body and zenith [deg]

print(f"Simulating {N_orbits} orbits  ({N} steps, dt = {dt:.0f} s)...")

for i, t in enumerate(t_arr):
    nu            = n_orb * t
    r_eci, v_eci  = perifocal_to_eci(nu, r_orb, V_orb)
    R             = dcm_sun_pointing(r_eci, v_eci)

    T_vec[i] = gravity_gradient_torque(r_eci, R)
    T_mag[i] = np.linalg.norm(T_vec[i])

    # Angle between +X_body (long axis) and zenith direction
    # R[0] = first row of body-from-ECI DCM = +X_body expressed in ECI
    e_r_eci   = r_eci / np.linalg.norm(r_eci)
    cos_th    = np.clip(np.dot(R[0], e_r_eci), -1.0, 1.0)
    theta[i]  = np.degrees(np.arccos(np.abs(cos_th)))   # fold to 0–90°

H_vec  = np.cumsum(T_vec * dt, axis=0)   # accumulated ΔH [N·m·s]
t_norm = t_arr / T_orb
N1     = N // N_orbits

# ======================================================================
#  9. SUMMARY STATISTICS
# ======================================================================
print("\nResults Summary:")
print("-" * 64)
print(f"  Worst-case (analytic) : {T_worst*1e9:.4f} nN·m  (θ = 45°)")
print(f"  Simulated peak        : {np.max(T_mag)*1e9:.4f} nN·m")
print(f"  Simulated mean        : {np.mean(T_mag)*1e9:.4f} nN·m")
print(f"  Peak / worst-case     : {np.max(T_mag)/T_worst:.3f}")
print(f"  θ range over orbit    : {np.min(theta):.1f}° – {np.max(theta):.1f}°")
print(f"  |ΔH| after orbit 1    : {np.linalg.norm(H_vec[N1-1])*1e9:.4f} nN·m·s")
print(f"  |ΔH| after orbit 2    : {np.linalg.norm(H_vec[-1])*1e9:.4f} nN·m·s")

# ======================================================================
#  10. FIGURES
# ======================================================================
fig_dpi = 150

# ── Figure 1 : Overview — torque / components / angular momentum ──────
fig1 = plt.figure(figsize=(13, 11), dpi=fig_dpi)
gs   = GridSpec(3, 1, figure=fig1, hspace=0.50)
ax_T = fig1.add_subplot(gs[0])
ax_C = fig1.add_subplot(gs[1], sharex=ax_T)
ax_H = fig1.add_subplot(gs[2], sharex=ax_T)

fig1.suptitle(
    'GOOSE 2U CubeSat — Gravity-Gradient Disturbance Torque\n'
    '515 km Dawn-Dusk SSO  |  Sun-Pointing  |  2 Orbital Periods',
    fontsize=13)

# (a) Torque magnitude + worst-case line
ax_T.plot(t_norm, T_mag*1e9, '#1f77b4', lw=1.5, label='|T_gg|  (sun-pointing)')
ax_T.fill_between(t_norm, T_mag*1e9, alpha=0.15, color='#1f77b4')
ax_T.axhline(T_worst*1e9, color='red', lw=1.3, ls='--',
             label=f'Worst case  {T_worst*1e9:.3f} nN·m  (θ = 45°)')
ax_T.set_ylabel('|T_gg|  [nN·m]')
ax_T.set_title('(a)  Gravity-Gradient Torque Magnitude', fontsize=10)
ax_T.legend(fontsize=9)
ax_T.grid(True, alpha=0.3)

# (b) Body-frame torque components
comp_colors = ['#d62728', '#2ca02c', '#9467bd']
comp_labels = ['$T_x$', '$T_y$', '$T_z$']
for k, (lbl, col) in enumerate(zip(comp_labels, comp_colors)):
    ax_C.plot(t_norm, T_vec[:, k]*1e9, color=col, lw=1.0, label=lbl)
ax_C.axhline(0, color='k', lw=0.5, ls=':')
ax_C.set_ylabel('T  [nN·m]')
ax_C.set_title('(b)  Body-Frame Torque Components', fontsize=10)
ax_C.legend(fontsize=9, ncol=3)
ax_C.grid(True, alpha=0.3)

# (c) Accumulated angular momentum
ax_H.plot(t_norm, np.linalg.norm(H_vec, axis=1)*1e9, 'b-', lw=1.8, label='|ΔH|')
ax_H.plot(t_norm, H_vec[:, 0]*1e9, 'r-', lw=0.8, alpha=0.55, label='$H_x$')
ax_H.plot(t_norm, H_vec[:, 1]*1e9, 'g-', lw=0.8, alpha=0.55, label='$H_y$')
ax_H.plot(t_norm, H_vec[:, 2]*1e9, 'm-', lw=0.8, alpha=0.55, label='$H_z$')
ax_H.set_ylabel('ΔH  [nN·m·s]')
ax_H.set_xlabel('Time  [orbital periods]')
ax_H.set_title('(c)  Accumulated Angular Momentum  ΔH = ∫T_gg dt', fontsize=10)
ax_H.legend(fontsize=9, ncol=2)
ax_H.grid(True, alpha=0.3)

for ax in (ax_T, ax_C, ax_H):
    ax.axvline(1.0, color='gray', lw=0.8, ls=':', alpha=0.7)
    ax.text(1.01, 0.92, '← orbit 1 | orbit 2 →',
            transform=ax.get_xaxis_transform(), fontsize=7, color='gray')

out1 = r'C:\Users\zalak\Desktop\Sem 4.1\gg_disturbance_overview.png'
fig1.savefig(out1, dpi=fig_dpi, bbox_inches='tight')
print(f"\nSaved: {out1}")

# ── Figure 2 : Misalignment angle and torque correlation ──────────────
fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5), dpi=fig_dpi)
fig2.suptitle(
    'GOOSE 2U — Gravity-Gradient Torque Geometry\n'
    '515 km Dawn-Dusk SSO  |  Sun-Pointing',
    fontsize=11)

# Left: misalignment angle vs time
ax2a = axes2[0]
ax2a.plot(t_norm, theta, '#8c564b', lw=1.2, label='θ  (+X_body ∠ zenith)')
ax2a.axhline(45.0, color='red', lw=1.0, ls='--', alpha=0.7, label='θ = 45°  (worst case)')
ax2a.axvline(1.0, color='gray', lw=0.8, ls=':', alpha=0.6)
ax2a.set_xlabel('Time  [orbital periods]')
ax2a.set_ylabel('θ  [degrees]')
ax2a.set_title('Misalignment angle between +X_body (long axis)\nand local zenith (radial outward)', fontsize=10)
ax2a.legend(fontsize=9)
ax2a.grid(True, alpha=0.3)

# Right: scatter of |T_gg| vs θ, coloured by time
# Overlaid with analytical reference T_worst · sin(2θ)
ax2b = axes2[1]
theta_ref = np.linspace(0, 90, 400)
T_theory  = T_worst * np.sin(np.radians(2.0 * theta_ref))
sc = ax2b.scatter(theta, T_mag*1e9, c=t_norm, cmap='plasma', s=5, zorder=3,
                  label='Simulated orbit')
ax2b.plot(theta_ref, T_theory*1e9, 'k--', lw=1.2, alpha=0.6,
          label='T$_{worst}$·sin(2θ)  (theory)')
plt.colorbar(sc, ax=ax2b, label='Time [orbital periods]')
ax2b.axvline(45.0, color='red', lw=0.8, ls=':', alpha=0.6)
ax2b.set_xlabel('θ  [degrees]')
ax2b.set_ylabel('|T_gg|  [nN·m]')
ax2b.set_title('Torque vs misalignment angle\n(scatter = orbit track; dashed = 1-axis theory)', fontsize=10)
ax2b.legend(fontsize=9)
ax2b.grid(True, alpha=0.3)

fig2.tight_layout()
out2 = r'C:\Users\zalak\Desktop\Sem 4.1\gg_geometry.png'
fig2.savefig(out2, dpi=fig_dpi, bbox_inches='tight')
print(f"Saved: {out2}")

# (a) Torque magnitude + worst-case line
fig3, ax3 = plt.subplots(figsize=(10, 6), dpi=fig_dpi)
ax3.plot(t_norm, T_mag*1e9, lw=1.5, label='|T_gg|  (sun-pointing)')
ax3.set_ylabel('|T_gg|  [nN·m]')
ax3.set_ylim(-0.1, np.max(T_mag)*1e9*1.2)
ax3.set_xlabel('Time  [orbital periods]')
ax3.set_title('GOOSE Gravity-Gradient Torque Magnitude', fontsize=10)
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3)
fig3.tight_layout()
out3 = r'C:\Users\zalak\Desktop\Sem 4.1\GOOSE_gg_geometry.png'
fig3.savefig(out3, dpi=fig_dpi, bbox_inches='tight')
print(f"Saved: {out3}")

# ── Engineering summary ───────────────────────────────────────────────
print()
print("=" * 64)
print("  ENGINEERING SUMMARY")
print("=" * 64)
print()
print("  Inertia properties (uniform rectangular prism, GOOSE 2U):")
print(f"    LX={LX*1e3:.0f} mm (+X, long),  LY={LY*1e3:.0f} mm,  LZ={LZ*1e3:.0f} mm,  m={MASS:.1f} kg")
print(f"    Ixx = {Ixx:.6f} kg·m²  (minimum, along long axis)")
print(f"    Iyy = {Iyy:.6f} kg·m²")
print(f"    Izz = {Izz:.6f} kg·m²")
print()
print("  Worst-case  T_gg_max = (3/2) · n² · (I_max − I_min):")
print(f"    = {T_worst*1e9:.4f} nN·m")
print(f"    Condition: minimum-inertia axis at 45° from radial")
print()
print(f"  Simulated sun-pointing results (2 orbits):")
print(f"    Peak torque : {np.max(T_mag)*1e9:.4f} nN·m  "
      f"({100*np.max(T_mag)/T_worst:.1f}% of worst case)")
print(f"    Mean torque : {np.mean(T_mag)*1e9:.4f} nN·m")
print(f"    θ range     : {np.min(theta):.1f}° – {np.max(theta):.1f}°")
print()
print("  Angular momentum accumulation (sun-pointing):")
print(f"    |ΔH| after orbit 1 : {np.linalg.norm(H_vec[N1-1])*1e9:.4f} nN·m·s")
print(f"    |ΔH| after orbit 2 : {np.linalg.norm(H_vec[-1])*1e9:.4f} nN·m·s")
drift = abs(np.linalg.norm(H_vec[-1]) - np.linalg.norm(H_vec[N1-1]))
print(f"    Per-orbit drift    : {drift*1e9:.4f} nN·m·s")
print()
print("All figures saved to Desktop/Sem 4.1/")

# ======================================================================
#  MATLAB / SIMULINK EXPORT
# ======================================================================
# Saves lookup-table data for a Simulink disturbance input block.
#
# In MATLAB, load and wire up as:
#   load('GOOSE_gg_Td.mat');
#   Td.time             = t;        % (N,1) [s]
#   Td.signals.values   = T_body;   % (N,3) [N·m]  — Tx Ty Tz columns
#   Td.signals.dimensions = 3;
#   % Then connect 'Td' to a 'From Workspace' block in Simulink.
# ======================================================================
mat_gg = {
    # --- time ---
    't':              t_arr.reshape(-1, 1),       # (N,1)  [s]
    'dt':             np.array([[dt]]),            # scalar [s]
    'T_orb':          np.array([[T_orb]]),         # scalar [s]
    # --- torque (sun-pointing) ---
    'T_body':         T_vec,                       # (N,3)  [N·m]
    'T_mag':          T_mag.reshape(-1, 1),        # (N,1)  [N·m]
    # --- geometry ---
    'theta_deg':      theta.reshape(-1, 1),        # (N,1)  misalignment angle [deg]
    'H_body':         H_vec,                       # (N,3)  accumulated ΔH [N·m·s]
    # --- worst-case scalar ---
    'T_gg_worst_Nm':  np.array([[T_worst]]),       # scalar [N·m]
    # --- inertia ---
    'I_body_kgm2':    I_body,                      # (3,3)  [kg·m²]
}
out_mat = r'C:\Users\zalak\Desktop\Sem 4.1\GOOSE_gg_Td.mat'
savemat(out_mat, mat_gg)
print(f"Saved MATLAB data: {out_mat}")
print("  Variables: t, T_body, T_mag, theta_deg, H_body,")
print("             T_gg_worst_Nm, I_body_kgm2, dt, T_orb")
