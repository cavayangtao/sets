#!/usr/bin/env python3
"""
Lateral Regression Guardrail Test (Issue #007)
===============================================
Verifies that adding vertical control does not degrade lateral performance
by more than 5%.

Test design:
  - Baseline: lateral control enabled, vertical control disabled
  - Comparison: both lateral and vertical control enabled
  - Same initial state and action sequence replayed for both
  - Lateral error metric: RMS position error in the xy-plane relative to
    the desired lateral position.

Usage:
    python3 tests/test_lateral_regression.py
"""

import sys
import os
import math

# Ensure the bindings module is importable.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'build'))
import bindings
import numpy as np


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
BASELINE_CONFIG = os.path.join(
    os.path.dirname(__file__), 'baseline_lateral_only.yaml')
COMPARISON_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'configs', 'sixdofaircraft',
    'policy_convergence_drone_vbody_yaw.yaml')

NUM_STEPS = 50          # trajectory length
DEGRADATION_LIMIT = 5.0  # percent
SEED = 42               # fixed seed for reproducibility


# ---------------------------------------------------------------------------
# Lateral error metric
# ---------------------------------------------------------------------------
def lateral_error_rmse(traj, xd):
    """RMS lateral position error [m] relative to desired xy position."""
    if len(traj.xs) == 0:
        return float('inf')
    sq_errors = []
    xd_x, xd_y = xd[0], xd[1]
    for x in traj.xs:
        sq_errors.append((x[0] - xd_x) ** 2 + (x[1] - xd_y) ** 2)
    return math.sqrt(sum(sq_errors) / len(sq_errors))


def lateral_error_max(traj, xd):
    """Maximum lateral position error [m] relative to desired xy position."""
    if len(traj.xs) == 0:
        return float('inf')
    max_err = 0.0
    xd_x, xd_y = xd[0], xd[1]
    for x in traj.xs:
        err = math.sqrt((x[0] - xd_x) ** 2 + (x[1] - xd_y) ** 2)
        max_err = max(max_err, err)
    return max_err


# ---------------------------------------------------------------------------
# Generate fixed action sequence for replay
# ---------------------------------------------------------------------------
def generate_action_sequence(num_steps):
    """Generate a deterministic, varied action sequence for replay."""
    actions = []
    for i in range(num_steps):
        # Mix lateral and vertical commands to exercise both channels.
        v_bx = 5.0 * math.sin(0.15 * i)       # oscillating forward
        v_by = 3.0 * math.cos(0.12 * i + 1.0)  # oscillating lateral
        v_bz = 2.0 * math.sin(0.08 * i)        # oscillating vertical
        yaw_rate = 0.5 * math.cos(0.1 * i)     # oscillating yaw
        actions.append(np.array([v_bx, v_by, v_bz, yaw_rate]))
    return actions


# ---------------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------------
def main():
    print('=' * 60)
    print('Issue #007: Lateral Regression Guardrail Test')
    print('=' * 60)
    print()

    # ---- Load MDPs ---------------------------------------------------------
    print('[Setup] Loading MDP instances...')
    mdp_baseline = bindings.get_mdp('SixDOFAircraft', BASELINE_CONFIG)
    mdp_compare = bindings.get_mdp('SixDOFAircraft', COMPARISON_CONFIG)

    x0 = mdp_baseline.initial_state()
    xd = np.array([120.0, 120.0, -120.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Verify initial states match.
    x0c = mdp_compare.initial_state()
    assert np.allclose(x0, x0c, atol=1e-9), "Initial states must match!"

    # ---- Generate replay action sequence -----------------------------------
    actions = generate_action_sequence(NUM_STEPS)
    print(f'[Setup] Generated {len(actions)} actions for replay.')
    print(f'        Action range: v_bx=[{min(a[0] for a in actions):.1f}, '
          f'{max(a[0] for a in actions):.1f}], '
          f'v_by=[{min(a[1] for a in actions):.1f}, '
          f'{max(a[1] for a in actions):.1f}]')

    # ---- Run baseline trajectory (lateral only) ----------------------------
    print()
    print('[Test] Running BASELINE trajectory (lateral only, vertical OFF)...')
    traj_base = bindings.rollout_action_sequence(
        x0, actions, mdp_baseline, False)
    print(f'        Steps: {len(traj_base.xs)}, Valid: {traj_base.is_valid}')

    rmse_base = lateral_error_rmse(traj_base, xd)
    max_base = lateral_error_max(traj_base, xd)
    print(f'        Lateral RMSE: {rmse_base:.4f} m')
    print(f'        Lateral Max:  {max_base:.4f} m')

    # ---- Run comparison trajectory (lateral + vertical) --------------------
    print()
    print('[Test] Running COMPARISON trajectory (lateral + vertical)...')
    traj_comp = bindings.rollout_action_sequence(
        x0, actions, mdp_compare, False)
    print(f'        Steps: {len(traj_comp.xs)}, Valid: {traj_comp.is_valid}')

    rmse_comp = lateral_error_rmse(traj_comp, xd)
    max_comp = lateral_error_max(traj_comp, xd)
    print(f'        Lateral RMSE: {rmse_comp:.4f} m')
    print(f'        Lateral Max:  {max_comp:.4f} m')

    # ---- Compare -----------------------------------------------------------
    print()
    print('[Compare] Lateral error degradation:')
    print(f'          Baseline  RMSE: {rmse_base:.6f} m')
    print(f'          Comparison RMSE: {rmse_comp:.6f} m')

    if rmse_base < 1e-9:
        # Baseline has essentially zero lateral error (pathological case).
        # Use absolute difference instead.
        degradation_pct = abs(rmse_comp - rmse_base) * 100.0
        print(f'          (baseline near zero, using absolute difference)')
    else:
        degradation_pct = (rmse_comp - rmse_base) / rmse_base * 100.0

    print(f'          RMSE degradation: {degradation_pct:+.4f} %')
    print(f'          Max error baseline:  {max_base:.4f} m')
    print(f'          Max error comparison: {max_comp:.4f} m')
    print()

    # ---- Assertion ---------------------------------------------------------
    if degradation_pct > DEGRADATION_LIMIT:
        print(f'FAIL: Lateral RMSE degraded by {degradation_pct:.2f}% '
              f'(limit: {DEGRADATION_LIMIT}%)')
        sys.exit(1)

    if rmse_comp > rmse_base * 1.05 and rmse_base > 0.01:
        print(f'FAIL: Comparison RMSE ({rmse_comp:.4f}) > 105% of baseline '
              f'({rmse_base:.4f})')
        sys.exit(1)

    print(f'PASS: Lateral degradation {degradation_pct:+.2f}% '
          f'within {DEGRADATION_LIMIT}% limit.')
    print()
    print('=' * 60)
    print('Lateral Regression Guardrail Test PASSED')
    print('=' * 60)
    return 0


if __name__ == '__main__':
    sys.exit(main())
