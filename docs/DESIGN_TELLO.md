# Tello Talent Planning Adaptation Design

## Design Goal
Use minimum-impact changes to adapt planning to DJI Tello Talent scale while preserving the current experiment workflow.

## Minimal-Change Strategy
1. Keep algorithmic flow unchanged in [scripts/rollout.py](scripts/rollout.py).
2. Introduce three staged config files:
   - [configs/sixdofaircraft/policy_convergence_tello_stage1.yaml](configs/sixdofaircraft/policy_convergence_tello_stage1.yaml)
   - [configs/sixdofaircraft/policy_convergence_tello_stage2.yaml](configs/sixdofaircraft/policy_convergence_tello_stage2.yaml)
   - [configs/sixdofaircraft/policy_convergence_tello_stage3.yaml](configs/sixdofaircraft/policy_convergence_tello_stage3.yaml)
3. Keep [scripts/policy_convergence.py](scripts/policy_convergence.py) as entry, and add stage selection via `TELLO_STAGE` environment variable.

## Configuration Decisions
1. Flight mode
   - Use quadrotor vbody-yaw mode for robust command-level control compatibility.
2. Vehicle scale
   - Stage-1 keeps a wide, proven-solvable envelope as baseline.
   - Stage-2 shrinks mass/inertia and control bounds to an intermediate region.
   - Stage-3 further shrinks to near-real Tello scale while preserving solver feasibility.
3. Scenario geometry
   - Start: (-20, 0, -12)
   - Goal: (20, 0, -12)
   - Distance: 40 m
   - Obstacles centered near x=-8 and x=8, both with y=0 and z=-12, thus on the start-goal line.
4. Compatibility
   - Keep config_name prefixed with policy_convergence so existing [scripts/plot_rollout_data.py](scripts/plot_rollout_data.py) file glob still matches rollout outputs.

## Stage Summary
1. Stage-1 (stable baseline)
   - Wide dynamic/control envelope for robust expansion.
2. Stage-2 (intermediate)
   - Reduced mass/inertia and reduced outer/inner control limits.
3. Stage-3 (near-real)
   - Near-real tello mass/inertia and tighter control limits with feasibility margin.

## Verification Plan
1. Parse YAML and check key presence.
2. Compute distance and obstacle collinearity metrics.
3. Execute [scripts/policy_convergence.py](scripts/policy_convergence.py) from [scripts](scripts) for each stage:
   - `TELLO_STAGE=stage1 python policy_convergence.py`
   - `TELLO_STAGE=stage2 python policy_convergence.py`
   - `TELLO_STAGE=stage3 python policy_convergence.py`
4. Generate visualization PDF via [scripts/plot_rollout_data.py](scripts/plot_rollout_data.py).
5. Confirm output artifacts exist in [data](data) and [plots](plots).

## Verification Results
1. Stage-1 runtime validation passed.
2. Stage-2 runtime validation passed.
3. Stage-3 runtime validation passed.
4. Final visualization artifact generated:
   - [plots/tello_rollout_three_stage.pdf](plots/tello_rollout_three_stage.pdf)
