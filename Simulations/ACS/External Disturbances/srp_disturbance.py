#!/usr/bin/env python3
"""
GOOSE 2U CubeSat — Solar Radiation Pressure Disturbance Torque
==============================================================
Propagates a circular dawn-to-dusk SSO at 515 km and computes the
solar radiation pressure (SRP) disturbance torque over 2 orbital periods.
Attitude: sun-pointing (+X_body → Sun, +Z_body → velocity).

Model
-----
  Flat-plate per-face model.  For each illuminated face:

    cos θᵢ  = max(0,  ŝ · n̂ᵢ)            ŝ = unit satellite-to-sun vector
    Fᵢ     = P_srp · Aᵢ · cosθᵢ · [(1−ρᵢ)(−ŝ) − 2ρᵢ cosθᵢ n̂ᵢ]
    Tᵢ     = r_cpᵢ × Fᵢ                  r_cp = CoM-to-face-centroid

  Total torque T_srp = Σ Tᵢ over illuminated faces.

  SRP at 1 AU: P = S / c = 1361 / 2.998×10⁸ = 4.56×10⁻⁶ N/m²

Worst case (SRP)
----------------
  Computed numerically: sweep all sun directions on a fine sphere grid
  and find the maximum |T_srp| over all orientations.

Key physics
-----------
  1. For perfect sun-pointing, the +X face is always illuminated.
     The sun direction in the body frame is exactly [1,0,0] throughout
     the orbit → SRP torque is CONSTANT over the orbit.
  2. The torque is driven by the CoM–CoP offset perpendicular to the
     SRP force direction.  A perfectly symmetric satellite with CoM at
     the geometric centre has zero SRP torque.
  3. Dawn-dusk SSO has essentially zero eclipse time → SRP acts
     continuously; angular momentum accumulates LINEARLY with time.
     This secular build-up is the primary ADCS concern for SRP.
  4. SRP torque is typically the smallest of the four disturbances for
     a 2U at 515 km, but its secular nature makes it important for
     long-duration missions.

Reference for CoM offset
-------------------------
  Conservative pre-flight estimate of 5–15 mm per axis, consistent with
  typical COTS 2U build tolerances (see Wertz et al. 2011, SMAD Ch. 19).
  Actual offset to be characterised from detailed mass budget at CDR.
"""

import sys
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401  (registers 3-D projection)
from scipy.io import savemat

# ======================================================================
#  1. PHYSICAL CONSTANTS
# ======================================================================
RE       = 6.3781e6           # Earth mean radius [m]
MU_E     = 3.986004418e14     # Gravitational parameter [m³/s²]
S_SOL    = 1361.0             # Solar flux at 1 AU [W/m²]
C_LIGHT  = 2.998e8            # Speed of light [m/s]
P_SRP    = S_SOL / C_LIGHT    # SRP at 1 AU  ≈ 4.56×10⁻⁶  [N/m²]

# ======================================================================
#  2. ORBITAL PARAMETERS — GOOSE 2U at 515 km dawn-dusk SSO
# ======================================================================
h_orbit  = 515e3
r_orb    = RE + h_orbit
T_orb    = 2*np.pi * np.sqrt(r_orb**3 / MU_E)
n_orb    = 2*np.pi / T_orb
V_orb    = np.sqrt(MU_E / r_orb)
inc      = np.radians(97.4)
RAAN     = np.radians(90.0)
argp     = np.radians(0.0)

print("=" * 64)
print("  GOOSE 2U — Solar Radiation Pressure Disturbance Torque")
print("=" * 64)
print(f"\n  Altitude:       {h_orbit/1e3:.0f} km")
print(f"  Period:         {T_orb:.1f} s  ({T_orb/60:.2f} min)")
print(f"  Inclination:    {np.degrees(inc):.1f}°  (SSO, dawn-dusk)")
print(f"  P_SRP at 1 AU:  {P_SRP:.4e} N/m²")
print()

# ======================================================================
#  3. SPACECRAFT GEOMETRY & SURFACE PROPERTIES
# ======================================================================
# 2U CubeSat body: 100 mm × 100 mm × 227 mm
# +X_body = long axis (2U stack direction) → Sun in sun-pointing mode
# CoM offset from geometric centre (conservative pre-CDR estimate)

LX = 0.227        # m  long axis (+X_body)
LY = 0.100        # m
LZ = 0.100        # m

# Centre-of-mass offset from geometric centre [m]
# Conservative estimate consistent with COTS 2U build tolerances
COM_OFFSET = np.array([0.005, 0.010, 0.010])

# Face definitions (in geometric-centre coordinate frame)
# rho: reflectivity  0 = fully absorbing  1 = fully specular
# +X face = solar panel (lower reflectivity); all others = painted aluminium
_face_data = [
    #  name           normal            A=area[m²]   rho   centroid (geom centre)
    ('+X solar panel', np.array([ 1., 0., 0.]),  LY*LZ, 0.15, np.array([ LX/2,   0.,   0.])),
    ('-X',             np.array([-1., 0., 0.]),  LY*LZ, 0.60, np.array([-LX/2,   0.,   0.])),
    ('+Y',             np.array([ 0., 1., 0.]),  LX*LZ, 0.50, np.array([  0.,  LY/2,   0.])),
    ('-Y',             np.array([ 0.,-1., 0.]),  LX*LZ, 0.50, np.array([  0., -LY/2,   0.])),
    ('+Z',             np.array([ 0., 0., 1.]),  LX*LY, 0.50, np.array([  0.,   0.,  LZ/2])),
    ('-Z',             np.array([ 0., 0.,-1.]),  LX*LY, 0.50, np.array([  0.,   0., -LZ/2])),
]

FACES = []
for name, normal, area, rho, centroid in _face_data:
    FACES.append({
        'name':  name,
        'n':     normal,
        'A':     area,
        'rho':   rho,
        'r_cp':  centroid - COM_OFFSET,   # CoM-to-centroid vector [m]
    })

print("  Spacecraft surface model (6 flat plates):")
print(f"    Dimensions:  {LX*1e3:.0f} × {LY*1e3:.0f} × {LZ*1e3:.0f} mm")
print(f"    CoM offset from geom centre:  "
      f"[{COM_OFFSET[0]*1e3:.0f}, {COM_OFFSET[1]*1e3:.0f}, {COM_OFFSET[2]*1e3:.0f}] mm  "
      f"(|Δ| = {np.linalg.norm(COM_OFFSET)*1e3:.1f} mm)")
print()
print("  Face properties:")
for f in FACES:
    print(f"    {f['name']:18s}  A={f['A']*1e4:.2f} cm²  ρ={f['rho']:.2f}  "
          f"C_R={1+f['rho']:.2f}  |r_cp|={np.linalg.norm(f['r_cp'])*1e3:.1f} mm")
print()

# ======================================================================
#  4. ECLIPSE CHECK — cylindrical shadow model
# ======================================================================
SUN_ECI  = np.array([1.0, 0.0, 0.0])   # Sun direction in ECI (June solstice)

def in_eclipse(r_eci):
    """
    Returns True if satellite is in Earth's cylindrical shadow.
    For a dawn-dusk SSO the result should be False throughout.
    """
    s = SUN_ECI
    proj   = np.dot(r_eci, s)            # projection along sun direction
    perp   = r_eci - proj * s            # perpendicular component
    return (proj < 0.0) and (np.linalg.norm(perp) < RE)

# ======================================================================
#  5. SRP TORQUE FUNCTION
# ======================================================================
def srp_torque_body(s_hat):
    """
    SRP disturbance torque in body frame [N·m].

    s_hat : unit vector from satellite to Sun, expressed in body frame.

    For each face:
      cos θᵢ  = max(0,  s_hat · n̂ᵢ)
      Fᵢ     = P_srp · Aᵢ · cosθᵢ · [(1−ρᵢ)(−s_hat) − 2ρᵢ cosθᵢ n̂ᵢ]
      Tᵢ     = r_cpᵢ × Fᵢ
    """
    T = np.zeros(3)
    for f in FACES:
        cos_th = np.dot(s_hat, f['n'])
        if cos_th <= 0.0:
            continue                       # face not illuminated
        d     = -s_hat                     # photon direction (sun → satellite)
        F     = P_SRP * f['A'] * cos_th * (
                    (1.0 - f['rho']) * d
                    - 2.0 * f['rho'] * cos_th * f['n'])
        T    += np.cross(f['r_cp'], F)
    return T

# ======================================================================
#  6. WORST-CASE SRP TORQUE — sphere sweep
# ======================================================================
# Numerically sweep all possible sun directions in the body frame
# and find the maximum |T_srp|.  Uses a Fibonacci sphere for uniform coverage.

print("Computing worst-case SRP torque (Fibonacci sphere, 50 000 directions)...")
N_SPHERE  = 50_000
golden    = (1.0 + np.sqrt(5.0)) / 2.0
i_arr     = np.arange(N_SPHERE)
theta_fib = np.arccos(1.0 - 2.0 * (i_arr + 0.5) / N_SPHERE)
phi_fib   = 2.0 * np.pi * i_arr / golden

s_sphere  = np.column_stack([
    np.sin(theta_fib) * np.cos(phi_fib),
    np.sin(theta_fib) * np.sin(phi_fib),
    np.cos(theta_fib),
])

T_sphere = np.array([np.linalg.norm(srp_torque_body(s)) for s in s_sphere])
idx_worst = np.argmax(T_sphere)
T_worst   = T_sphere[idx_worst]
s_worst   = s_sphere[idx_worst]

print(f"  T_srp worst case = {T_worst*1e9:.4f} nN·m")
print(f"  Worst sun dir (body) = [{s_worst[0]:.3f}, {s_worst[1]:.3f}, {s_worst[2]:.3f}]")
print()

# ======================================================================
#  7. ORBITAL MECHANICS
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
#  8. ATTITUDE MODEL — SUN-POINTING
# ======================================================================
def dcm_sun_pointing(r_eci, v_eci):
    """
    Body-from-ECI DCM for sun-pointing attitude.
    +X_body → Sun,  +Z_body → velocity (ram).
    """
    x_b   = SUN_ECI.copy()
    v_hat = v_eci / np.linalg.norm(v_eci)
    z_b   = v_hat - np.dot(v_hat, x_b) * x_b
    nz    = np.linalg.norm(z_b)
    z_b   = z_b / nz if nz > 1e-10 else np.array([0.0, 0.0, 1.0])
    y_b   = np.cross(z_b, x_b)
    y_b  /= np.linalg.norm(y_b)
    return np.vstack([x_b, y_b, z_b])    # v_body = R @ v_eci

# ======================================================================
#  9. SIMULATION — 2 ORBITS
# ======================================================================
N_orbits = 2
dt       = 10.0
t_arr    = np.arange(0.0, N_orbits * T_orb, dt)
N        = len(t_arr)

T_vec     = np.zeros((N, 3))   # body-frame SRP torque [N·m]
T_mag     = np.zeros(N)        # torque magnitude [N·m]
eclipse   = np.zeros(N, dtype=bool)
r_eci_arr = np.zeros((N, 3))   # ECI position history [m]
v_eci_arr = np.zeros((N, 3))   # ECI position history [m]

print(f"Simulating {N_orbits} orbits  ({N} steps, dt = {dt:.0f} s)...")

for i, t in enumerate(t_arr):
    nu           = n_orb * t
    r_eci, v_eci = perifocal_to_eci(nu, r_orb, V_orb)
    R            = dcm_sun_pointing(r_eci, v_eci)

    r_eci_arr[i] = r_eci
    v_eci_arr[i] = v_eci
    eclipse[i]   = in_eclipse(r_eci)
    nu_factor    = 0.0 if eclipse[i] else 1.0

    # Sun direction in body frame — always [1,0,0] for perfect sun-pointing
    s_hat_body   = R @ SUN_ECI      # = [1, 0, 0] by construction

    T_body       = nu_factor * srp_torque_body(s_hat_body)
    T_vec[i]     = T_body
    T_mag[i]     = np.linalg.norm(T_body)

H_vec  = np.cumsum(T_vec * dt, axis=0)   # accumulated ΔH [N·m·s]
t_norm = t_arr / T_orb
N1     = N // N_orbits

n_eclipse = np.sum(eclipse)
print(f"  Eclipse steps: {n_eclipse} / {N}  "
      f"({'none — dawn-dusk SSO fully sunlit' if n_eclipse == 0 else f'{n_eclipse*dt:.0f} s in shadow'})")

# ======================================================================
#  10. GRAVITY-GRADIENT WORST CASE (for disturbance budget comparison)
# ======================================================================
# Inertia values identical to gravity_gradient_disturbance.py
MASS  = 2.0
Ixx   = (1.0/12.0) * MASS * (LY**2 + LZ**2)
Iyy   = (1.0/12.0) * MASS * (LX**2 + LZ**2)
Izz   = (1.0/12.0) * MASS * (LX**2 + LY**2)
T_gg_worst = 1.5 * n_orb**2 * (max(Ixx, Iyy, Izz) - min(Ixx, Iyy, Izz))

# ======================================================================
#  11. SUMMARY STATISTICS
# ======================================================================
print("\nResults Summary:")
print("-" * 64)
print(f"  Worst-case SRP  (any attitude, sphere sweep) : {T_worst*1e9:.4f} nN·m")
print(f"  Simulated SRP   (sun-pointing, constant)     : {np.max(T_mag)*1e9:.4f} nN·m")
print(f"  Worst-case GG   (θ = 45°, any attitude)      : {T_gg_worst*1e9:.4f} nN·m")
print(f"  Sim SRP / worst-case SRP                     : {np.max(T_mag)/T_worst:.3f}")
print(f"  |ΔH_srp| after orbit 1 : "
      f"{np.linalg.norm(H_vec[N1-1])*1e9:.4f} nN·m·s")
print(f"  |ΔH_srp| after orbit 2 : "
      f"{np.linalg.norm(H_vec[-1])*1e9:.4f} nN·m·s")
dH_per_orbit = np.linalg.norm(H_vec[-1]) - np.linalg.norm(H_vec[N1-1])
print(f"  ΔH drift per orbit     : {abs(dH_per_orbit)*1e9:.4f} nN·m·s  "
      f"(secular — no cancellation)")

# ======================================================================
#  12. FIGURES
# ======================================================================
fig_dpi = 150

# ── Figure 1 : Overview ────────────────────────────────────────────────
fig1 = plt.figure(figsize=(13, 11), dpi=fig_dpi)
gs   = GridSpec(3, 1, figure=fig1, hspace=0.50)
ax_T = fig1.add_subplot(gs[0])
ax_C = fig1.add_subplot(gs[1], sharex=ax_T)
ax_H = fig1.add_subplot(gs[2], sharex=ax_T)

fig1.suptitle(
    'GOOSE 2U CubeSat — Solar Radiation Pressure Disturbance Torque\n'
    '515 km Dawn-Dusk SSO  |  Sun-Pointing  |  2 Orbital Periods',
    fontsize=13)

# (a) Torque magnitude
ax_T.plot(t_norm, T_mag*1e9, '#e377c2', lw=1.5, label='|T_srp|  (sun-pointing)')
ax_T.fill_between(t_norm, T_mag*1e9, alpha=0.18, color='#e377c2')
ax_T.axhline(T_worst*1e9, color='red', lw=1.3, ls='--',
             label=f'SRP worst case  {T_worst*1e9:.3f} nN·m  (any attitude)')
ax_T.axhline(T_gg_worst*1e9, color='orange', lw=1.0, ls=':',
             label=f'GG worst case   {T_gg_worst*1e9:.3f} nN·m  (θ = 45°)')
ax_T.set_ylabel('|T_srp|  [nN·m]')
ax_T.set_title('(a)  SRP Torque Magnitude  —  constant for perfect sun-pointing', fontsize=10)
ax_T.legend(fontsize=9)
ax_T.grid(True, alpha=0.3)

# (b) Body-frame torque components
comp_colors = ['#d62728', '#2ca02c', '#9467bd']
comp_labels = ['$T_x$', '$T_y$', '$T_z$']
for k, (lbl, col) in enumerate(zip(comp_labels, comp_colors)):
    ax_C.plot(t_norm, T_vec[:, k]*1e9, color=col, lw=1.0, label=lbl)
ax_C.axhline(0, color='k', lw=0.5, ls=':')
ax_C.set_ylabel('T  [nN·m]')
ax_C.set_title('(b)  Body-Frame Torque Components  '
               '(constant — only CoM offset and +X face contribute)', fontsize=10)
ax_C.legend(fontsize=9, ncol=3)
ax_C.grid(True, alpha=0.3)

# (c) Accumulated angular momentum — linear growth (secular)
ax_H.plot(t_norm, np.linalg.norm(H_vec, axis=1)*1e9, 'b-', lw=1.8, label='|ΔH|  (secular growth)')
ax_H.plot(t_norm, H_vec[:, 0]*1e9, 'r-',  lw=0.8, alpha=0.55, label='$H_x$')
ax_H.plot(t_norm, H_vec[:, 1]*1e9, 'g-',  lw=0.8, alpha=0.55, label='$H_y$')
ax_H.plot(t_norm, H_vec[:, 2]*1e9, 'm-',  lw=0.8, alpha=0.55, label='$H_z$')
ax_H.set_ylabel('ΔH  [nN·m·s]')
ax_H.set_xlabel('Time  [orbital periods]')
ax_H.set_title('(c)  Accumulated Angular Momentum  ΔH = ∫T_srp dt  '
               '(linear — no orbital cancellation)', fontsize=10)
ax_H.legend(fontsize=9, ncol=2)
ax_H.grid(True, alpha=0.3)

for ax in (ax_T, ax_C, ax_H):
    ax.axvline(1.0, color='gray', lw=0.8, ls=':', alpha=0.7)
    ax.text(1.01, 0.92, '← orbit 1 | orbit 2 →',
            transform=ax.get_xaxis_transform(), fontsize=7, color='gray')

out1 = r'C:\Users\zalak\Desktop\Sem 4.1\AERO4701\ACS\External Disturbances\srp_disturbance_overview.png'
fig1.savefig(out1, dpi=fig_dpi, bbox_inches='tight')
print(f"\nSaved: {out1}")

# ── Figure 2 : Torque heatmap — |T_srp| vs all sun directions ─────────
# Azimuth (φ) vs elevation (λ) scan; shows worst-case sun direction
N_AZ = 360
N_EL = 180
az_arr = np.linspace(0,   360, N_AZ)
el_arr = np.linspace(-90,  90, N_EL)
AZ, EL = np.meshgrid(az_arr, el_arr)

T_map = np.zeros_like(AZ)
for j, el in enumerate(el_arr):
    el_r = np.radians(el)
    for k, az in enumerate(az_arr):
        az_r = np.radians(az)
        s = np.array([np.cos(el_r)*np.cos(az_r),
                      np.cos(el_r)*np.sin(az_r),
                      np.sin(el_r)])
        T_map[j, k] = np.linalg.norm(srp_torque_body(s))

# Sun direction for sun-pointing (always [1,0,0] in body frame)
az_sp  = np.degrees(np.arctan2(SUN_ECI[1], SUN_ECI[0]))   # 0°
el_sp  = np.degrees(np.arcsin(SUN_ECI[2]))                 # 0°

fig2, ax2 = plt.subplots(figsize=(10, 6), dpi=fig_dpi)
pm = ax2.pcolormesh(AZ, EL, T_map*1e9, cmap='hot', shading='auto')
plt.colorbar(pm, ax=ax2, label='|T_srp|  [nN·m]')
ax2.contour(AZ, EL, T_map*1e9, levels=8, colors='white', linewidths=0.4, alpha=0.5)

# Mark sun-pointing direction
ax2.plot(az_sp % 360, el_sp, 'c*', ms=14, label=f'Sun-pointing  ({az_sp:.0f}°, {el_sp:.0f}°)',
         markeredgecolor='white', markeredgewidth=0.5)
# Mark worst-case direction
az_w = np.degrees(np.arctan2(s_worst[1], s_worst[0])) % 360
el_w = np.degrees(np.arcsin(np.clip(s_worst[2], -1, 1)))
ax2.plot(az_w, el_w, 'r^', ms=12, label=f'Worst case  ({az_w:.0f}°, {el_w:.0f}°)',
         markeredgecolor='white', markeredgewidth=0.5)

ax2.set_xlabel('Azimuth φ  [degrees]')
ax2.set_ylabel('Elevation λ  [degrees]')
ax2.set_title('GOOSE 2U  — SRP Torque Magnitude vs Sun Direction in Body Frame\n'
              '(identifies worst-case pointing scenario for disturbance budget)',
              fontsize=10)
ax2.legend(fontsize=9, loc='lower right')
ax2.set_xlim(0, 360)
ax2.set_ylim(-90, 90)

fig2.tight_layout()
out2 = r'C:\Users\zalak\Desktop\Sem 4.1\AERO4701\ACS\External Disturbances\srp_heatmap.png'
fig2.savefig(out2, dpi=fig_dpi, bbox_inches='tight')
print(f"Saved: {out2}")

# ── Figure 3 : Single-panel clean output ──────────────────────────────
fig3, ax3 = plt.subplots(figsize=(10, 6), dpi=fig_dpi)
ax3.plot(t_norm, T_mag*1e9, '#e377c2', lw=1.5, label='|T_srp|  (sun-pointing)')
ax3.axhline(T_worst*1e9,  color='red',    lw=1.3, ls='--',
            label=f'SRP worst case  {T_worst*1e9:.3f} nN·m')
ax3.set_xlabel('Time  [orbital periods]')
ax3.set_ylabel('|T_srp|  [nN·m]')
ax3.set_ylim(bottom=0)
ax3.set_title('GOOSE 2U — SRP Disturbance Torque  (Sun-Pointing)', fontsize=11)
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3)
fig3.tight_layout()
out3 = r'C:\Users\zalak\Desktop\Sem 4.1\AERO4701\ACS\External Disturbances\GOOSE_srp_torque.png'
fig3.savefig(out3, dpi=fig_dpi, bbox_inches='tight')
print(f"Saved: {out3}")

# ── Figure 4 : Orbit illumination — 3D view + view along sun ──────────
# Compute beta angle and eclipse geometry
h_hat    = np.array([np.sin(inc)*np.sin(RAAN),
                     -np.sin(inc)*np.cos(RAAN),
                      np.cos(inc)])          # orbit normal unit vector
beta_deg = np.degrees(np.arcsin(np.clip(np.dot(SUN_ECI, h_hat), -1, 1)))
beta_min = np.degrees(np.arcsin(RE / r_orb))   # critical beta for no eclipse

# Minimum perpendicular distance from sun-Earth line over the whole orbit
r_perp      = np.sqrt(r_eci_arr[:, 1]**2 + r_eci_arr[:, 2]**2)   # Y-Z distance
r_perp_min  = np.min(r_perp)

# Worst-case seasonal orbit: beta reduced by max solar declination (23.5°)
beta_worst  = beta_deg - 23.5
r_perp_wc   = np.sin(np.radians(beta_worst)) * r_orb   # semi-minor of Y-Z ellipse

fig4 = plt.figure(figsize=(16, 7), dpi=fig_dpi)
fig4.suptitle(
    'GOOSE 2U — Orbit Illumination Analysis  |  515 km Dawn-Dusk SSO\n'
    'Assumption: sun fixed at [1,0,0] (June solstice epoch)  |  '
    'Seasonal variation not modelled',
    fontsize=11)

# ── Left panel: 3D orbit ─────────────────────────────────────────────
ax4a = fig4.add_subplot(121, projection='3d')

sc = 1.0 / r_orb   # scale factor: plot in units of r_orb

# Earth sphere
u_e = np.linspace(0, 2*np.pi, 40)
v_e = np.linspace(0, np.pi, 20)
Xe  = (RE*sc) * np.outer(np.cos(u_e), np.sin(v_e))
Ye  = (RE*sc) * np.outer(np.sin(u_e), np.sin(v_e))
Ze  = (RE*sc) * np.outer(np.ones(40),  np.cos(v_e))
ax4a.plot_surface(Xe, Ye, Ze, alpha=0.35, color='#1f77b4', linewidth=0, zorder=1)

# Shadow cylinder (extends in -X direction, radius RE, length 3*r_orb)
phi_c  = np.linspace(0, 2*np.pi, 60)
x_c    = np.linspace(-3.0, 0.0, 20)           # in units of r_orb
PHI_c, X_c = np.meshgrid(phi_c, x_c)
Y_cyl  = (RE*sc) * np.cos(PHI_c)
Z_cyl  = (RE*sc) * np.sin(PHI_c)
ax4a.plot_surface(X_c, Y_cyl, Z_cyl, alpha=0.08, color='gray', linewidth=0, zorder=0)
# Shadow cap (circle at x=0)
ax4a.plot((RE*sc)*np.cos(phi_c), (RE*sc)*np.sin(phi_c),
          zs=0, zdir='x', color='gray', lw=0.8, alpha=0.5)

# Orbit tracks (1 orbit shown solid, 2nd dashed)
o1 = r_eci_arr[:N1]
o2 = r_eci_arr[N1:]
ax4a.plot(o1[:, 0]*sc, o1[:, 1]*sc, o1[:, 2]*sc,
          color='#2ca02c', lw=1.5, label='Orbit 1 (sunlit)')
ax4a.plot(o2[:, 0]*sc, o2[:, 1]*sc, o2[:, 2]*sc,
          color='#2ca02c', lw=0.8, ls='--', alpha=0.55, label='Orbit 2 (sunlit)')

# Sun direction arrow (from x=+2 pointing toward origin)
ax4a.quiver(2.2, 0, 0, -0.6, 0, 0, color='#ffdd00', arrow_length_ratio=0.25,
            linewidth=2.5, zorder=5)
ax4a.text(2.35, 0, 0, 'Sun', color='#ffdd00', fontsize=9, fontweight='bold',
          ha='center', va='center')

# Orbit normal arrow
ax4a.quiver(0, 0, 0, h_hat[0]*1.4, h_hat[1]*1.4, h_hat[2]*1.4,
            color='cyan', arrow_length_ratio=0.15, linewidth=1.5)
ax4a.text(h_hat[0]*1.55, h_hat[1]*1.55, h_hat[2]*1.55,
          'h (orbit\nnormal)', color='cyan', fontsize=7, ha='center')

ax4a.set_xlim(-1.3, 1.3)
ax4a.set_ylim(-1.3, 1.3)
ax4a.set_zlim(-1.3, 1.3)
ax4a.set_xlabel('X_ECI  [r_orb]', fontsize=8)
ax4a.set_ylabel('Y_ECI  [r_orb]', fontsize=8)
ax4a.set_zlabel('Z_ECI  [r_orb]', fontsize=8)
ax4a.set_title(f'3D Orbit  (β = {beta_deg:.1f}°, shadow cylinder in grey)',
               fontsize=9)
ax4a.legend(fontsize=8, loc='upper left')
ax4a.view_init(elev=25, azim=-60)

# ── Right panel: view along sun direction (+X → screen) ──────────────
# This is the key plot: shows whether the orbit's Y-Z projection
# stays outside the Earth's shadow disk (circle of radius RE).
ax4b = fig4.add_subplot(122)

theta_circ = np.linspace(0, 2*np.pi, 300)

# Earth filled disk (shadow boundary)
ax4b.fill(RE * np.cos(theta_circ) / 1e3,
          RE * np.sin(theta_circ) / 1e3,
          color='#1f77b4', alpha=0.25, zorder=2)
ax4b.plot(RE * np.cos(theta_circ) / 1e3,
          RE * np.sin(theta_circ) / 1e3,
          '#1f77b4', lw=2.0, label=f'Earth shadow boundary  (r = {RE/1e3:.0f} km)', zorder=3)

# Actual orbit Y-Z projection (colored per orbit)
ax4b.plot(o1[:, 1] / 1e3, o1[:, 2] / 1e3,
          color='#2ca02c', lw=1.8, label='Orbit 1 (simulated)', zorder=4)
ax4b.plot(o2[:, 1] / 1e3, o2[:, 2] / 1e3,
          color='#2ca02c', lw=1.0, ls='--', alpha=0.55, label='Orbit 2 (simulated)', zorder=4)

# Worst-case seasonal orbit ellipse (β reduced by 23.5°)
yz_wc_y = r_orb * np.cos(theta_circ)         # semi-major axis (along Y, unchanged)
yz_wc_z = r_perp_wc * np.sin(theta_circ)     # semi-minor axis (along Z, reduced)
ax4b.plot(yz_wc_y / 1e3, yz_wc_z / 1e3,
          color='orange', lw=1.2, ls=':',
          label=f'Worst-case season  (β = {beta_worst:.1f}°, ±23.5° sun decl.)', zorder=3)

# Minimum distance annotation
idx_min = np.argmin(r_perp)
r_min_pt = r_eci_arr[idx_min]
ax4b.annotate(
    f'min r⊥ = {r_perp_min/1e3:.0f} km\n({r_perp_min/RE:.3f} × R_E)',
    xy=(r_min_pt[1]/1e3, r_min_pt[2]/1e3),
    xytext=(r_min_pt[1]/1e3 - 1500, r_min_pt[2]/1e3 - 2500),
    arrowprops=dict(arrowstyle='->', color='black', lw=1.2),
    fontsize=8, color='black',
    bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.8))

# No-eclipse condition box
margin_km = (r_perp_min - RE) / 1e3
txt = (f'β = {beta_deg:.1f}°  >  β_min = {beta_min:.1f}°\n'
       f'→ No eclipse (fixed sun epoch)\n'
       f'Margin: {margin_km:.0f} km = {r_perp_min/RE:.3f} × R_E\n'
       f'Worst-case seasonal β = {beta_worst:.1f}°\n'
       f'  → r⊥_min = {r_perp_wc/1e3:.0f} km  {"< R_E → ECLIPSE" if r_perp_wc < RE else "> R_E → no eclipse"}')
ax4b.text(0.02, 0.02, txt, transform=ax4b.transAxes, fontsize=8,
          verticalalignment='bottom',
          bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

ax4b.set_aspect('equal')
ax4b.set_xlabel('Y_ECI  [km]', fontsize=9)
ax4b.set_ylabel('Z_ECI  [km]', fontsize=9)
ax4b.set_title('View Along Sun Direction (+X)\n'
               'Orbit must stay outside Earth disk to avoid eclipse',
               fontsize=9)
ax4b.legend(fontsize=8, loc='upper right')
ax4b.grid(True, alpha=0.25)

fig4.tight_layout()
out4 = r'C:\Users\zalak\Desktop\Sem 4.1\AERO4701\ACS\External Disturbances\srp_orbit_illumination.png'
fig4.savefig(out4, dpi=fig_dpi, bbox_inches='tight')
print(f"Saved: {out4}")

# (a) Torque magnitude
fig5, ax5 = plt.subplots(figsize=(10, 6), dpi=fig_dpi)
ax5.plot(t_norm, T_mag*1e9, lw=1.5, label='|T_srp|  (sun-pointing)')
ax5.set_ylabel('|T_srp|  [nN·m]')
ax5.set_xlabel('Time  [orbital periods]')
ax5.set_title('SRP Torque Magnitude  —  constant for perfect sun-pointing', fontsize=10)
ax5.legend(fontsize=9)
ax5.grid(True, alpha=0.3)

fig5.tight_layout()
out5 = r'C:\Users\zalak\Desktop\Sem 4.1\AERO4701\ACS\External Disturbances\GOOSE_srp_disturb.png'
fig5.savefig(out5, dpi=fig_dpi, bbox_inches='tight')
print(f"Saved: {out5}")

# ── Engineering summary ───────────────────────────────────────────────
print()
print("=" * 64)
print("  ENGINEERING SUMMARY")
print("=" * 64)
print()
print("  ASSUMPTION (eclipse / sun direction):")
print("    Sun fixed at SUN_ECI = [1, 0, 0]  (June solstice epoch).")
print("    No seasonal variation of sun direction modelled.")
print(f"    At this epoch: β = {beta_deg:.1f}° > β_min = {beta_min:.1f}°")
print(f"    → No eclipse confirmed (margin = {margin_km:.0f} km = {r_perp_min/RE:.3f}×R_E).")
print(f"    Worst-case seasonal β ≈ {beta_worst:.1f}° (β − 23.5° solar declination).")
if r_perp_wc < RE:
    print(f"    At worst-case season, short eclipses DO occur (r⊥_min = {r_perp_wc/1e3:.0f} km < R_E).")
    print("    SRP torque budget is unaffected (eclipses reduce disturbance, not increase it).")
print()
print("  Surface model (flat-plate, 6 faces):")
print(f"    Body: {LX*1e3:.0f} × {LY*1e3:.0f} × {LZ*1e3:.0f} mm")
print(f"    CoM offset from geom centre: "
      f"[{COM_OFFSET[0]*1e3:.0f}, {COM_OFFSET[1]*1e3:.0f}, {COM_OFFSET[2]*1e3:.0f}] mm")
print(f"    +X (solar panel) ρ = 0.15  →  C_R = 1.15")
print(f"    All other faces  ρ = 0.50  →  C_R = 1.50")
print()
print("  SRP disturbance (sun-pointing, constant throughout orbit):")
print(f"    T_srp (sim)      = {np.max(T_mag)*1e9:.4f} nN·m  [constant — secular]")
print(f"    T_srp worst case = {T_worst*1e9:.4f} nN·m  [any attitude, sphere sweep]")
print(f"    Worst sun dir    = [{s_worst[0]:.3f}, {s_worst[1]:.3f}, {s_worst[2]:.3f}]  (body frame)")
print()
print("  GG worst case (from gravity_gradient_disturbance.py):")
print(f"    T_gg worst case  = {T_gg_worst*1e9:.4f} nN·m  (θ = 45°, any attitude)")
print()
print("  Angular momentum accumulation (secular — no cancellation):")
print(f"    |ΔH| after orbit 1 : {np.linalg.norm(H_vec[N1-1])*1e9:.4f} nN·m·s")
print(f"    |ΔH| after orbit 2 : {np.linalg.norm(H_vec[-1])*1e9:.4f} nN·m·s")
print(f"    Rate of growth     : {np.linalg.norm(H_vec[-1])/( N_orbits*T_orb )*1e9:.6f} nN·m  "
      f"(= average torque, constant)")
print()
print(f"  Eclipse steps: {n_eclipse}/{N}  "
      f"({'fully sunlit throughout — dawn-dusk SSO confirmed' if n_eclipse==0 else 'partial shadow detected'})")
print()
print("All figures saved to Desktop/Sem 4.1/")

# ======================================================================
#  MATLAB / SIMULINK EXPORT
# ======================================================================
# Saves lookup-table data for a Simulink disturbance input block.
#
# In MATLAB, load and wire up as:
#   load('GOOSE_srp_Td.mat');
#   Td.time             = t;        % (N,1) [s]
#   Td.signals.values   = T_body;   % (N,3) [N·m]  — Tx Ty Tz columns
#   Td.signals.dimensions = 3;
#   % Then connect 'Td' to a 'From Workspace' block in Simulink.
#
# Note: T_body is constant for perfect sun-pointing (see script header).
#       Eclipse factor is included (0 during shadow, 1 in sunlight).
#       For this epoch no eclipses occur — all shadow_factor values = 1.
# ======================================================================
mat_srp = {
    # --- time ---
    't':               t_arr.reshape(-1, 1),         # (N,1)  [s]
    'dt':              np.array([[dt]]),              # scalar [s]
    'T_orb':           np.array([[T_orb]]),           # scalar [s]
    # --- torque (sun-pointing) ---
    'T_body':          T_vec,                         # (N,3)  [N·m]
    'T_mag':           T_mag.reshape(-1, 1),          # (N,1)  [N·m]
    # --- eclipse flag ---
    'eclipse':         eclipse.astype(np.uint8).reshape(-1, 1),  # (N,1) 0=sunlit 1=shadow
    # --- orbit positions ---
    'r_eci_m':         r_eci_arr,                     # (N,3)  [m]
    # --- orbit positions ---
    'v_eci_m':         v_eci_arr,                     # (N,3)  [m]
    # --- angular momentum accumulation ---
    'H_body':          H_vec,                         # (N,3)  [N·m·s]
    # --- worst-case scalars ---
    'T_srp_worst_Nm':  np.array([[T_worst]]),         # scalar [N·m]
    'T_gg_worst_Nm':   np.array([[T_gg_worst]]),      # scalar [N·m]
    # --- illumination geometry ---
    'beta_deg':        np.array([[beta_deg]]),        # scalar [deg]
    'beta_min_deg':    np.array([[beta_min]]),        # scalar [deg]
}
out_mat = r'C:\Users\zalak\Desktop\Sem 4.1\AERO4701\ACS\External Disturbances\GOOSE_srp_Td.mat'
savemat(out_mat, mat_srp)
print(f"Saved MATLAB data: {out_mat}")
print("  Variables: t, T_body, T_mag, eclipse, r_eci_m, v_eci_m H_body,")
print("             T_srp_worst_Nm, T_gg_worst_Nm, beta_deg, dt, T_orb")
