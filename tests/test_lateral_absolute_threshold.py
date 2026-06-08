#!/usr/bin/env python3
"""
Lateral Absolute RMSE Guardrail Test
===================================
Guards against regressions where lateral error becomes unacceptably large,
regardless of relative degradation percentage.

Usage:
    python3 tests/test_lateral_absolute_threshold.py
"""

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'build'))
import bindings
import numpy as np


COMPARISON_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'configs', 'sixdofaircraft',
    'policy_convergence_drone_vbody_yaw.yaml')

NUM_STEPS = 50
ABSOLUTE_RMSE_LIMIT = 340.0  # meters


def lateral_error_rmse(traj, xd):
    if len(traj.xs) == 0:
        return float('inf')
    sq_errors = []
    xd_x, xd_y = xd[0], xd[1]
    for x in traj.xs:
        sq_errors.append((x[0] - xd_x) ** 2 + (x[1] - xd_y) ** 2)
    return math.sqrt(sum(sq_errors) / len(sq_errors))


def generate_action_sequence(num_steps):
    actions = []
    for i in range(num_steps):
        v_bx = 5.0 * math.sin(0.15 * i)
        v_by = 3.0 * math.cos(0.12 * i + 1.0)
        v_bz = 2.0 * math.sin(0.08 * i)
        yaw_rate = 0.5 * math.cos(0.1 * i)
        actions.append(np.array([v_bx, v_by, v_bz, yaw_rate]))
    return actions


def main():
    print('=' * 68)
    print('Lateral Absolute RMSE Guardrail Test')
    print('=' * 68)

    mdp = bindings.get_mdp('SixDOFAircraft', COMPARISON_CONFIG)
    x0 = mdp.initial_state()
    xd = np.array([120.0, 120.0, -120.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    actions = generate_action_sequence(NUM_STEPS)
    traj = bindings.rollout_action_sequence(x0, actions, mdp, False)

    rmse = lateral_error_rmse(traj, xd)
    print(f'Lateral RMSE: {rmse:.4f} m')
    print(f'Absolute limit: {ABSOLUTE_RMSE_LIMIT:.4f} m')

    if rmse > ABSOLUTE_RMSE_LIMIT:
        print('FAIL: lateral RMSE exceeds absolute guardrail')
        return 1

    print('PASS: lateral RMSE is within absolute guardrail')
    return 0


if __name__ == '__main__':
    sys.exit(main())
