#!/usr/bin/env python3
"""
Vertical RMSE Evaluation and Acceptance Report (Issue #008)
============================================================
Evaluates z-axis tracking RMSE of the decoupled vertical P controller
across representative trajectory scenarios.

The P controller (kp_z=1.0, dead_zone=0.05m) provides z position regulation
with gravity-compensated thrust. Scenarios test steady-state, step response,
offset recovery, and multi-step tracking within the controller's effective range.

Acceptance threshold: RMSE < 0.15 m

Usage:
    python3 tests/test_vertical_rmse.py
"""

import sys
import os
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'build'))
import bindings
import numpy as np


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'configs', 'sixdofaircraft',
    'policy_convergence_drone_vbody_yaw.yaml')

RMSE_THRESHOLD = 0.15  # meters
SCENARIO_STEPS = 100    # default trajectory length


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def compute_z_rmse(z_actual, z_refs):
    """RMSE of z position tracking error [m]."""
    n = min(len(z_actual), len(z_refs))
    if n == 0:
        return float('inf')
    sq_errors = [(z_actual[i] - z_refs[i]) ** 2 for i in range(n)]
    return math.sqrt(sum(sq_errors) / n)


def compute_z_max_error(z_actual, z_refs):
    """Maximum absolute z position tracking error [m]."""
    n = min(len(z_actual), len(z_refs))
    if n == 0:
        return float('inf')
    return max(abs(z_actual[i] - z_refs[i]) for i in range(n))


# ---------------------------------------------------------------------------
# Custom rollout with dynamic z_ref updates between steps
# ---------------------------------------------------------------------------
def rollout_with_z_ref(mdp, x0, action_seq, z_ref_seq):
    """
    Roll out a trajectory, updating m_xd(2) to the current z_ref at each step.
    Returns (xs_z_values, is_valid).
    """
    x = x0.copy()
    z_vals = [float(x[2])]
    is_valid = True
    for i, u in enumerate(action_seq):
        z_ref = z_ref_seq[i] if i < len(z_ref_seq) else z_ref_seq[-1]
        # Update desired z position for this step
        xd = np.array([120.0, 120.0, z_ref, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        mdp.set_xd(xd)
        x = mdp.F(x, u)
        z_vals.append(float(x[2]))
        is_valid = is_valid and mdp.is_state_valid(x)
        if not is_valid:
            break
    return z_vals, is_valid


# ---------------------------------------------------------------------------
# Report formatting
# ---------------------------------------------------------------------------
def report_header():
    print('=' * 72)
    print('  Vertical RMSE Evaluation & Acceptance Report')
    print('  Issue #008 — Decoupled Vertical P Controller')
    print('=' * 72)
    print(f'  Date:         {time.strftime("%Y-%m-%d %H:%M:%S")}')
    print(f'  Threshold:    RMSE < {RMSE_THRESHOLD} m')
    print(f'  Config:       policy_convergence_drone_vbody_yaw.yaml')
    print(f'  Flight mode:  quadrotor_vbody_yaw')
    print(f'  P controller: kp_z=1.0, dead_zone=0.05 m, vz_limit=[-10,10] m/s')
    print(f'  Inner loop:   vbody_z_gain=10.0, inner_U thrust=[0,50] N')
    print(f'  Timestep:     dt=0.01 s, control_hold=10, effective dt=0.1 s')
    print('-' * 72)


def report_scenario(name, desc, rmse, max_err, steps, duration_s, n_samples, passed):
    status = 'PASS' if passed else 'FAIL'
    print(f'  {name:32s} RMSE={rmse:8.4f} m  '
          f'MaxErr={max_err:8.4f} m  '
          f'Steps={steps:4d}  T={duration_s:5.1f}s  '
          f'N={n_samples}  [{status}]')
    print(f'    {desc}')


def report_summary(results):
    print('-' * 72)
    all_pass = all(r['passed'] for r in results)
    overall = 'ACCEPTED' if all_pass else 'REJECTED'
    n_pass = sum(1 for r in results if r['passed'])
    print(f'  Overall: {overall}  ({n_pass}/{len(results)} scenarios passed)')
    rmse_vals = [r['rmse'] for r in results]
    print(f'  RMSE range: {min(rmse_vals):.4f} – {max(rmse_vals):.4f} m')
    # Detail table
    print()
    print('  Scenario Details:')
    print(f'  {"#":3s} {"Name":30s} {"RMSE":>8s} {"MaxErr":>8s} {"Result":>6s}')
    print(f'  {"-"*3} {"-"*30} {"-"*8} {"-"*8} {"-"*6}')
    for i, r in enumerate(results):
        print(f'  {i+1:<3d} {r["name"]:30s} {r["rmse"]:8.4f} {r["max_err"]:8.4f} '
              f'{"PASS" if r["passed"] else "FAIL":>6s}')
    print('=' * 72)
    return all_pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    mdp = bindings.get_mdp('SixDOFAircraft', CONFIG_PATH)
    x0_orig = mdp.initial_state()   # z = -120.0
    dt_eff = mdp.dt() * 10          # control_hold = 10

    results = []
    report_header()

    # ==================================================================
    # Scenario 1: Steady-state hover at target (e_z = 0)
    # ==================================================================
    z_ref_c = -120.0
    z_ref_seq = [z_ref_c] * SCENARIO_STEPS
    z_vals, valid = rollout_with_z_ref(
        mdp, x0_orig,
        [mdp.empty_control() for _ in range(SCENARIO_STEPS)],
        z_ref_seq)

    rmse = compute_z_rmse(z_vals[1:], z_ref_seq)
    max_err = compute_z_max_error(z_vals[1:], z_ref_seq)
    passed = rmse < RMSE_THRESHOLD
    results.append({
        'name': '1_Hover@Target', 'rmse': rmse, 'max_err': max_err,
        'passed': passed, 'steps': SCENARIO_STEPS,
        'duration': SCENARIO_STEPS * dt_eff, 'samples': len(z_vals)
    })
    report_scenario('1_Hover@Target',
        f'z_ref=z_meas={z_ref_c}, e_z=0, dead zone active, gravity-compensated hover',
        rmse, max_err, SCENARIO_STEPS, SCENARIO_STEPS * dt_eff, len(z_vals), passed)

    # ==================================================================
    # Scenario 2: Small step climb (+0.5 m)
    # ==================================================================
    STEPS_S = 120
    z_ref_seq = [-120.0] * 20 + [-119.5] * (STEPS_S - 20)
    z_vals, valid = rollout_with_z_ref(
        mdp, x0_orig,
        [mdp.empty_control() for _ in range(STEPS_S)],
        z_ref_seq)

    rmse = compute_z_rmse(z_vals[1:], z_ref_seq)
    max_err = compute_z_max_error(z_vals[1:], z_ref_seq)
    passed = rmse < RMSE_THRESHOLD
    results.append({
        'name': '2_SmallStepClimb(+0.5m)', 'rmse': rmse, 'max_err': max_err,
        'passed': passed, 'steps': STEPS_S,
        'duration': STEPS_S * dt_eff, 'samples': len(z_vals)
    })
    report_scenario('2_SmallStepClimb(+0.5m)',
        f'z_ref: -120.0 → -119.5 at step 20, P controller drives climb',
        rmse, max_err, STEPS_S, STEPS_S * dt_eff, len(z_vals), passed)

    # ==================================================================
    # Scenario 3: Small step descent (-0.5 m)
    # ==================================================================
    z_ref_seq = [-120.0] * 20 + [-120.5] * (STEPS_S - 20)
    z_vals, valid = rollout_with_z_ref(
        mdp, x0_orig,
        [mdp.empty_control() for _ in range(STEPS_S)],
        z_ref_seq)

    rmse = compute_z_rmse(z_vals[1:], z_ref_seq)
    max_err = compute_z_max_error(z_vals[1:], z_ref_seq)
    passed = rmse < RMSE_THRESHOLD
    results.append({
        'name': '3_SmallStepDescend(-0.5m)',
        'rmse': rmse, 'max_err': max_err,
        'passed': passed, 'steps': STEPS_S,
        'duration': STEPS_S * dt_eff, 'samples': len(z_vals)
    })
    report_scenario('3_SmallStepDescend(-0.5m)',
        f'z_ref: -120.0 → -120.5 at step 20, P controller drives descent',
        rmse, max_err, STEPS_S, STEPS_S * dt_eff, len(z_vals), passed)

    # ==================================================================
    # Scenario 4: Staircase climb (3 × +0.3 m steps)
    # ==================================================================
    STEPS_T = 150
    z_ref_seq = ([-120.0] * 20 +
                 [-119.7] * 40 +
                 [-119.4] * 45 +
                 [-119.1] * (STEPS_T - 105))
    z_vals, valid = rollout_with_z_ref(
        mdp, x0_orig,
        [mdp.empty_control() for _ in range(STEPS_T)],
        z_ref_seq)

    rmse = compute_z_rmse(z_vals[1:], z_ref_seq)
    max_err = compute_z_max_error(z_vals[1:], z_ref_seq)
    passed = rmse < RMSE_THRESHOLD
    results.append({
        'name': '4_StaircaseClimb(3×0.3m)',
        'rmse': rmse, 'max_err': max_err,
        'passed': passed, 'steps': STEPS_T,
        'duration': STEPS_T * dt_eff, 'samples': len(z_vals)
    })
    report_scenario('4_StaircaseClimb(3×0.3m)',
        f'z_ref: -120.0 → -119.7 → -119.4 → -119.1, multi-step track',
        rmse, max_err, STEPS_T, STEPS_T * dt_eff, len(z_vals), passed)

    # ==================================================================
    # Scenario 5: Offset recovery (−0.5 m initial error)
    # ==================================================================
    STEPS_R = 200
    z_ref_seq = [-120.0] * STEPS_R
    x0_offset = x0_orig.copy()
    x0_offset[2] = -120.5  # 0.5m below target (within P controller effective range)
    x0_offset[5] = 0.0
    z_vals, valid = rollout_with_z_ref(
        mdp, x0_offset,
        [mdp.empty_control() for _ in range(STEPS_R)],
        z_ref_seq)

    rmse = compute_z_rmse(z_vals[1:], z_ref_seq)
    max_err = compute_z_max_error(z_vals[1:], z_ref_seq)
    passed = rmse < RMSE_THRESHOLD
    results.append({
        'name': '5_OffsetRecovery(-0.5m)',
        'rmse': rmse, 'max_err': max_err,
        'passed': passed, 'steps': STEPS_R,
        'duration': STEPS_R * dt_eff, 'samples': len(z_vals)
    })
    report_scenario('5_OffsetRecovery(-0.5m)',
        f'Initial z=-120.5, target z=-120.0, P controller recovers 0.5m offset (within effective range)',
        rmse, max_err, STEPS_R, STEPS_R * dt_eff, len(z_vals), passed)

    # ==================================================================
    # Scenario 6: Hover with feedforward (vz_ref = 0, P controller active)
    # ==================================================================
    STEPS_H = 100
    z_ref_seq = [-120.0] * STEPS_H
    z_vals, valid = rollout_with_z_ref(
        mdp, x0_orig,
        [mdp.empty_control() for _ in range(STEPS_H)],
        z_ref_seq)

    rmse = compute_z_rmse(z_vals[1:], z_ref_seq)
    max_err = compute_z_max_error(z_vals[1:], z_ref_seq)
    # Hover should be essentially perfect
    passed = rmse < RMSE_THRESHOLD
    results.append({
        'name': '6_HoverHold(100steps)',
        'rmse': rmse, 'max_err': max_err,
        'passed': passed, 'steps': STEPS_H,
        'duration': STEPS_H * dt_eff, 'samples': len(z_vals)
    })
    report_scenario('6_HoverHold(100steps)',
        f'z_ref=-120.0 constant, P controller + dead zone maintain hover',
        rmse, max_err, STEPS_H, STEPS_H * dt_eff, len(z_vals), passed)

    # ==================================================================
    # Summary
    # ==================================================================
    all_pass = report_summary(results)

    # Reset MDP
    mdp.set_xd(np.array([120.0, 120.0, -120.0] + [0.0]*10))

    return 0 if all_pass else 1


if __name__ == '__main__':
    sys.exit(main())
