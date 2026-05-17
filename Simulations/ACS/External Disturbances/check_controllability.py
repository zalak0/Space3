#!/usr/bin/env python3
"""
Controllability Analysis: Magnetic Moment at 97° SSO
======================================================

Check if the control authority (B-field configuration) degrades or loses
controllability at any point in the orbit. This is critical for TVLQR success.

For magnetorquer-only control: m × B must span 3D space.
If B aligns with m (parallel/antiparallel), torque is zero regardless of m control.
If B is confined to 2D subspace, 1 axis loses controllability.
"""

import numpy as np
import pandas as pd
import ppigrf

# Constants
RE       = 6.3781e6
MU_E     = 3.986004418e14
IGRF_DATE = pd.Timestamp('2026-01-01')

h_orbit = 515e3
r_orb   = RE + h_orbit
V_orb   = np.sqrt(MU_E / r_orb)
T_orb   = 2*np.pi * np.sqrt(r_orb**3 / MU_E)
n_orb   = 2*np.pi / T_orb
inc     = np.radians(97.4)
RAAN    = np.radians(90.0)
argp    = np.radians(0.0)

print("=" * 70)
print("  Controllability Analysis — Magnetorquer-Only Control @ 97° SSO")
print("=" * 70)

def perifocal_to_eci(nu):
    cO, sO = np.cos(RAAN), np.sin(RAAN)
    ci, si = np.cos(inc),  np.sin(inc)
    cw, sw = np.cos(argp), np.sin(argp)
    r_p = r_orb * np.array([ np.cos(nu),  np.sin(nu), 0.0])
    R = np.array([
        [ cO*cw - sO*ci*sw,  -(cO*sw + sO*ci*cw),   sO*si ],
        [ sO*cw + cO*ci*sw,  -(sO*sw - cO*ci*cw),  -cO*si ],
        [ si*sw,               si*cw,                ci    ]
    ])
    return R @ r_p

def eci_to_geocentric_spherical(r_eci):
    x, y, z = r_eci
    r     = np.sqrt(x**2 + y**2 + z**2)
    colat = np.degrees(np.arccos(np.clip(z / r, -1.0, 1.0)))
    lon   = np.degrees(np.arctan2(y, x))
    return r / 1e3, colat, lon

def geocentric_spherical_to_eci_matrix(colat_rad, lon_rad):
    ct, st = np.cos(colat_rad), np.sin(colat_rad)
    cp, sp = np.cos(lon_rad),   np.sin(lon_rad)
    r_hat  = np.array([ st*cp,  st*sp,  ct])
    th_hat = np.array([ ct*cp,  ct*sp, -st])
    ph_hat = np.array([-sp,     cp,     0.0])
    return np.column_stack([r_hat, th_hat, ph_hat])

def get_B_eci(r_eci):
    r_km, colat, lon = eci_to_geocentric_spherical(r_eci)
    Br, Bth, Bph = ppigrf.igrf_gc(
        np.array([r_km]), np.array([colat]), np.array([lon]), IGRF_DATE,
    )
    Br_T  = float(Br[0, 0])  * 1e-9
    Bth_T = float(Bth[0, 0]) * 1e-9
    Bph_T = float(Bph[0, 0]) * 1e-9
    colat_rad = np.radians(colat)
    lon_rad   = np.radians(lon)
    R_sph2eci = geocentric_spherical_to_eci_matrix(colat_rad, lon_rad)
    B_eci = R_sph2eci @ np.array([Br_T, Bth_T, Bph_T])
    return B_eci

# Sample orbit
dt       = 30.0
t_arr    = np.arange(0.0, T_orb, dt)
N        = len(t_arr)

# Controllability metrics
controllability_index = np.zeros(N)  # Ratio of min-to-max singular value of B×
rank_deficiency = np.zeros(N)  # How many axes lose controllability
min_sigma = np.zeros(N)         # Smallest singular value (lower = worse)
max_sigma = np.zeros(N)         # Largest singular value

M_PERM = np.array([0.0364, 0.0409, 0.0432])

print(f"\nAnalyzing {N} timesteps over 1 orbit...\n")

for i, t in enumerate(t_arr):
    nu = n_orb * t
    r_eci = perifocal_to_eci(nu)
    B_eci = get_B_eci(r_eci)
    
    # Build the control effectiveness matrix: τ = m × B
    # We want to check if we can actuate all 3 axes by varying m
    # The control matrix is the cross-product operator [B]_× = skew(B)
    B_mag = np.linalg.norm(B_eci)
    B_hat = B_eci / B_mag if B_mag > 1e-12 else B_eci
    
    # Skew-symmetric matrix for cross product: a × b = [a]_× @ b
    def skew(v):
        return np.array([
            [0,     -v[2],  v[1]],
            [v[2],   0,    -v[0]],
            [-v[1],  v[0],  0]
        ])
    
    # Control input matrix: how m variations affect τ
    # τ = m × B, so dτ = dm × B (approximately, for small dm)
    # We can think of this as: each component of m can induce torque via B
    B_cross_op = skew(B_eci)  # operator such that τ = B_cross_op @ m
    
    # Singular value decomposition
    U, sigma, Vt = np.linalg.svd(B_cross_op)
    
    min_sigma[i] = np.min(sigma)
    max_sigma[i] = np.max(sigma)
    
    # Condition number (lower is better; higher means ill-conditioned)
    cond = np.max(sigma) / (np.min(sigma) + 1e-12)
    controllability_index[i] = cond
    
    # Count how many singular values are "small" (< threshold)
    threshold = 1e-7 * np.max(sigma)  # Small relative to largest
    rank_deficiency[i] = np.sum(sigma < threshold)

# Analysis
print("=" * 70)
print("  CONTROLLABILITY METRICS")
print("=" * 70)

min_cond = np.min(controllability_index)
max_cond = np.max(controllability_index)
mean_cond = np.mean(controllability_index)

print(f"\nCondition number of control matrix (lower is better):")
print(f"  Min:   {min_cond:8.2f}   (best case)")
print(f"  Max:   {max_cond:8.2f}   (worst case) {'  ⚠️ CRITICAL' if max_cond > 100 else ''}")
print(f"  Mean:  {mean_cond:8.2f}")
print(f"  Std:   {np.std(controllability_index):8.2f}")

print(f"\nSingular values of B-field operator √B⊥²:")
print(f"  Max:   {np.max(max_sigma)*1e6:8.4f} µT")
print(f"  Min:   {np.min(min_sigma)*1e6:8.4f} µT  {'  ⚠️ VERY SMALL' if np.min(min_sigma) < 1e-8 else ''}")

n_singular = np.sum(rank_deficiency > 0)
print(f"\nRank deficiency (axes losing control):")
print(f"  Timesteps with rank < 3:  {n_singular}/{N}  ({100*n_singular/N:.1f}%)")
if n_singular > 0:
    print(f"  ⚠️  WARNING: Control authority lost at {n_singular} points in orbit!")

print(f"\n" + "=" * 70)
print("  INTERPRETATION FOR TVLQR")
print("=" * 70)

if max_cond > 100:
    print("""
  ⚠️  HIGH CONDITION NUMBER DETECTED
  
  The control effectiveness matrix is severely ill-conditioned.
  This means:
  - TVLQR gains will be very large and numerically unstable
  - Small errors in B-field measurement → huge control input errors
  - Sensor noise gets amplified dramatically
  - Controller may be inherently UNSTABLE even in simulation
  
  This could explain why your TVLQR doesn't work!
  """)
elif max_cond > 10:
    print("""
  ⚠️  MODERATE CONDITION NUMBER
  
  Control effectiveness varies significantly over the orbit.
  TVLQR might work but:
  - Requires careful gain tuning
  - Sensitive to modeling errors
  - May need saturation limits
  """)
else:
    print("""
  ✓ GOOD CONDITION NUMBER
  
  Control system is well-conditioned.
  If TVLQR isn't working, issue likely elsewhere.
  """)

if n_singular > 0:
    print(f"""
  ⚠️  RANK DEFICIENCY DETECTED
  
  At {n_singular} points in the orbit, you lose full 3-axis control.
  The satellite passes through configurations where it cannot
  actuate all three axes simultaneously.
  
  For TVLQR: The problem becomes nonlinear/singular→ solver may fail!
  """)

print()
