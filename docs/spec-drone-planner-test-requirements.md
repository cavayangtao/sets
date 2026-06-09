# SPEC: Drone Planner Minimal End-to-End Test Requirements

> Technical specification derived from: [tasks/prd-drone-planner-test-requirements.md](tasks/prd-drone-planner-test-requirements.md)
> Generated: 2026-06-08 | Target branch: main | Commit: b799d4d

## 1. Summary

### 1.1 What This SPEC Covers
This SPEC defines how to implement a minimal, one-command, internal ROS test workflow for drone_planner_node + stanley_controller_node. It focuses on two-waypoint sequential mission validation, dynamic target switching, artifact generation (PNG/GIF/JSON), and threshold-based pass/fail checks. External Gazebo mode is explicitly out of scope.

### 1.2 PRD Reference
- Source: [tasks/prd-drone-planner-test-requirements.md](tasks/prd-drone-planner-test-requirements.md)
- User Stories covered: US-001, US-002, US-003, US-004, US-005, US-006
- Functional Requirements covered: FR-1 through FR-13

### 1.3 Design Decisions Summary
| Decision | Choice | Rationale |
|----------|--------|-----------|
| Test script location | Extend existing [test/run_drone_planner_test.py](test/run_drone_planner_test.py) | Reuse current logic and avoid duplicate entry points |
| Scenario template location | tests/scenarios | Clear separation: executable runner in test/, static test data in tests/ |
| Failure exit codes | Segmented exit codes | Better CI diagnosis and faster triage |
| Lateral absolute RMSE threshold | Keep 340.0 m | Align with current baseline and avoid blocking initial rollout |
| SPEC save location | tasks/spec-drone-planner-test-requirements.md | Keep PRD/SPEC colocated for traceability |

---

## 2. Architecture

### 2.1 System Context
The test runner orchestrates planner/controller validation with an internal pose simulation node.

Dataflow:
1. Test runner starts and validates ROS services.
2. Internal pose node publishes /pose.
3. Planner consumes pose and publishes /planner/trajectory and /planner/control_seq (+ optional /planner/vz_cmd topics).
4. Stanley controller consumes trajectory and pose, publishes /cmd_vel.
5. Test runner samples pose/cmd/traj, checks waypoint reach, then writes artifacts.

### 2.2 Component Design
- Component A: MissionRunner (existing runner core in [test/run_drone_planner_test.py](test/run_drone_planner_test.py))
  - Responsibilities: orchestration, service calls, phase transitions, threshold checks, exit code emission.
- Component B: InternalPoseNode (existing DroneRosTestPoseNode)
  - Responsibilities: model-based pose publishing in internal mode.
- Component C: ScenarioTemplateLoader (new module/function block inside runner)
  - Responsibilities: load built-in or external scenario template and resolve final mission params.
- Component D: ArtifactWriter (existing plotting/report block with minor extension)
  - Responsibilities: PNG/GIF/JSON generation and standardized report schema.

### 2.3 Module Interactions
- MissionRunner -> ScenarioTemplateLoader: resolve wp0/wp1/timeouts/reach-dist/thresholds.
- MissionRunner -> /drone_planner/arm service: arm planner.
- MissionRunner -> /drone_planner/update_target service: set target.
- MissionRunner -> ArtifactWriter: serialize run result and metrics.
- MissionRunner -> process exit: emit segmented exit code.

### 2.4 File Structure

```
test/
  run_drone_planner_test.py                [MODIFY]

tests/
  scenarios/
    default_two_waypoints.yaml             [NEW]
    obstacle_crossing.yaml                 [NEW, optional starter template]

  test_lateral_regression.py               [EXISTING]
  test_vertical_rmse.py                    [EXISTING]
  test_lateral_absolute_threshold.py       [EXISTING]

docs/
  RUN_DRONE_PLANNER_TEST_SKILL.md          [MODIFY]
```

---

## 3. Data Model

### 3.1 Schema Changes
No database schema changes.

### 3.2 Entity Definitions

Scenario template schema (YAML):

```yaml
name: default_two_waypoints
wp0: [-20.0, 0.0, -120.0]
wp1: [20.0, 0.0, -120.0]
reach_dist: 2.0
wp_timeout: 90.0
pose_wait_timeout: 20.0
thresholds:
  z_rmse_max: 0.15
  lateral_degradation_pct_max: 5.0
  lateral_absolute_rmse_max: 340.0
```

Run report schema (JSON):

```json
{
  "status": "success|failed",
  "error": "string",
  "failure_type": "none|service_unavailable|pose_timeout|waypoint_timeout|threshold_failed|scenario_invalid|artifact_write_failed",
  "exit_code": 0,
  "scenario": {"name": "default_two_waypoints"},
  "wp0": [0, 0, 0],
  "wp1": [0, 0, 0],
  "reach_dist": 2.0,
  "phase_min_dist": {"wp0": 1.2, "wp1": 1.8},
  "metrics": {
    "final_pos": [0, 0, 0],
    "path_len": 0.0,
    "trajectory_msg_count": 0
  }
}
```

### 3.3 Relationships
- Scenario template feeds runtime mission config.
- Runtime mission config drives service requests and timeout checks.
- Samples drive metrics; metrics + status form report.

### 3.4 Migration Plan
- Backward compatible default: existing CLI behavior remains unchanged if scenario args are not used.
- No data migration needed.
- Rollback: remove template-loading branch and keep current hardcoded/default CLI flow.

---

## 4. API Design

### 4.1 Interfaces

| Interface Type | Name | Description | Auth | Request | Response |
|----------------|------|-------------|------|---------|----------|
| CLI arg | --scenario-template | Select built-in template name | N/A | string | resolved scenario |
| CLI arg | --scenario-file | External YAML template path | N/A | path | resolved scenario |
| ROS Service | /drone_planner/arm | Arm planner | ROS local | SetBool | success/message |
| ROS Service | /drone_planner/update_target | Update target | ROS local | Trigger | success/message |
| ROS Topic Sub | /pose | Pose samples | N/A | PoseStamped | internal state update |
| ROS Topic Sub | /planner/trajectory | Trajectory heartbeat | N/A | Path | traj availability flag |
| ROS Topic Sub | /cmd_vel | Controller output | N/A | Twist | command sample |

### 4.2 Request/Response Schemas
- CLI parsing rules:
  - --scenario-template and explicit --wp0/--wp1 can coexist; explicit waypoint args override template values.
  - --scenario-file has highest priority for template base config.
- ROS service validations:
  - arm failure with non-RUNNING reason -> failure_type=service_unavailable.
  - update_target failure -> failure_type=service_unavailable.

### 4.3 Error Responses
- Non-HTTP style: structured stderr log + JSON report + process exit code.
- All failures must include:
  - failure_type
  - human-readable error message
  - phase context (if mission phase exists)

### 4.4 Breaking Changes
No breaking API changes; all new CLI options are additive.

---

## 5. Business Logic

### 5.1 Core Algorithms
Mission execution algorithm:
1. Parse CLI and load scenario template.
2. Resolve final mission parameters (template <- file <- CLI overrides).
3. Start internal pose node if --use-ros-test-node is enabled.
4. Wait planner services.
5. Wait first pose until pose_wait_timeout.
6. Set wp0, arm, wait reach with timeout.
7. Set wp1, wait reach with timeout.
8. Save artifacts and report.
9. Evaluate thresholds and finalize exit code.

Template resolution algorithm:
1. Initialize with built-in default_two_waypoints.
2. If --scenario-template set, load matching built-in template.
3. If --scenario-file set, merge external file on top.
4. Apply explicit CLI fields as final overrides.
5. Validate required fields and numeric ranges.

### 5.2 Validation Rules
- wp0/wp1 must be length-3 numeric arrays.
- reach_dist > 0.
- wp_timeout > 0.
- pose_wait_timeout > 0.
- threshold values must be non-negative.
- scenario template missing required field -> scenario_invalid.

### 5.3 State Machine
States:
- INIT -> WAIT_POSE -> ARM -> PHASE_WP0 -> PHASE_WP1 -> DONE
- Any state -> FAILED on fatal error

Transitions:
- WAIT_POSE timeout -> FAILED(pose_timeout)
- PHASE_WP0 timeout -> FAILED(waypoint_timeout)
- PHASE_WP1 timeout -> FAILED(waypoint_timeout)
- Artifact write failure after mission -> FAILED(artifact_write_failed)

### 5.4 Edge Cases
- Services available but arm returns already RUNNING: treat as non-fatal warning.
- Partial samples on failure: still generate report/artifacts when possible.
- Empty trajectory message stream: metrics computed with fallback fields and warning.
- Scenario file parse error: hard fail before mission start.

---

## 6. Error Handling

### 6.1 Error Taxonomy
| Error Code | Exit Code | Condition | User Message |
|------------|-----------|-----------|--------------|
| OK | 0 | Full pass | Mission and thresholds passed |
| SERVICE_UNAVAILABLE | 10 | arm/update_target unavailable or failed | Planner service unavailable or rejected request |
| POSE_TIMEOUT | 11 | No pose before timeout | No pose received before pose_wait_timeout |
| WAYPOINT_TIMEOUT | 12 | wp0/wp1 not reached before timeout | Timeout waiting waypoint |
| THRESHOLD_FAILED | 13 | Metrics exceed threshold | Acceptance thresholds not met |
| SCENARIO_INVALID | 14 | Invalid template or missing fields | Scenario template invalid |
| ARTIFACT_WRITE_FAILED | 15 | PNG/GIF/JSON save failed | Failed to save artifacts/report |
| UNEXPECTED_RUNTIME | 16 | Unhandled exception | Unexpected runtime error |

### 6.2 Retry Strategy
- Service wait: one bounded wait window, no infinite retry.
- update_target: no automatic retry in first version.
- Artifact write: single attempt; fail with ARTIFACT_WRITE_FAILED.

### 6.3 Failure Modes
- Planner down: fail fast with SERVICE_UNAVAILABLE.
- Pose stream missing: fail with POSE_TIMEOUT.
- Mission progress stalled: fail with WAYPOINT_TIMEOUT.
- Metrics fail: fail with THRESHOLD_FAILED while preserving artifacts.

---

## 7. Security

### 7.1 Authentication & Authorization
- ROS local network trust model assumed.
- No new auth/role model introduced.

### 7.2 Input Validation
- Validate all CLI numeric inputs and YAML schema before mission start.
- Reject malformed waypoint values and non-finite numbers.
- Reject unsafe file path traversal only if sandbox policy is added (future option).

### 7.3 Data Protection
- No sensitive data expected in artifacts.
- Keep logs/report local filesystem only.

---

## 8. Performance

### 8.1 Expected Load
- Single process mission runner.
- Sample rate around incoming /pose frequency (commonly 20-50 Hz).
- One run artifact set: PNG + GIF + JSON.

### 8.2 Optimization Strategy
- Cap GIF frames (existing pattern max 300).
- Keep sample structure flat for low overhead.
- Avoid per-sample heavy plotting; render once at end.

### 8.3 Runtime/Storage Considerations
- Report and artifact size bounded by sample count and frame cap.
- Scenario templates are tiny static YAML files.

---

## 9. Testing Strategy

### 9.1 Unit Tests
- Template parser and precedence rules.
- Exit code mapping logic.
- Threshold evaluation helper.

### 9.2 Integration Tests
- Internal ROS mode happy path with default template.
- External scenario-file path handling and override behavior.
- Service failure and pose timeout paths.

### 9.3 Edge Case Tests
- Missing template field -> SCENARIO_INVALID.
- wp_timeout reached in wp0 and wp1 separately.
- Artifact save failure path (simulate unwritable directory).

### 9.4 Acceptance Criteria Mapping
| US/FR | Test | Type | Description |
|-------|------|------|-------------|
| US-001, FR-1 | test_runner_entrypoint_executes_full_flow | integration | One-command run completes orchestration |
| US-002, FR-2 | test_internal_pose_node_mode | integration | Internal pose simulation publishes valid pose stream |
| US-003, FR-5/6 | test_wp0_wp1_sequential_progression | integration | Verify ordered target progression and timeout logic |
| US-004, FR-11/12/13 | test_scenario_template_resolution | unit/integration | Validate built-in, file override, and CLI override precedence |
| US-005, FR-8 | test_artifacts_generated | integration | PNG/GIF/JSON output exists and parseable |
| FR-10 | test_threshold_gate_pass_fail | unit/integration | Threshold checks return expected status and exit code |

---

## 10. Implementation Plan

### 10.1 Phases
Phase 1: Scenario template foundation
- Add tests/scenarios/default_two_waypoints.yaml
- Add template loader and validation in runner

Phase 2: Exit code segmentation
- Replace current generic failure code with taxonomy in Section 6.1
- Add failure_type and exit_code to JSON report

Phase 3: Threshold gate formalization
- Centralize threshold evaluation (z_rmse, lateral_degradation_pct, lateral_absolute_rmse)
- Keep absolute RMSE threshold at 340.0

Phase 4: Docs and polish
- Update runbook with template usage and exit code table
- Final acceptance run and artifact verification

### 10.2 Issue Mapping
| Issue | SPEC Sections | Priority | Depends On |
|-------|---------------|----------|------------|
| #1 Template loader + schema | 3.2, 5.1, 5.2 | high | — |
| #2 Scenario precedence and overrides | 4.2, 5.1 | high | #1 |
| #3 Segmented exit code handling | 6.1, 6.3 | high | #1 |
| #4 Threshold gate implementation | 8.1, 9.4 | high | #3 |
| #5 Artifact/report schema extension | 3.2, 4.3 | medium | #3 |
| #6 Documentation update | 2.4, 10.1 | medium | #1-#5 |

### 10.3 Incremental Delivery
- Delivery 1: template + runner compatibility with current defaults.
- Delivery 2: segmented exit codes and failure_type.
- Delivery 3: threshold gate enforced in final status.
- Delivery 4: docs complete and CI integration.

---

## 11. Open Questions & Risks

### 11.1 Unresolved Questions
- Should scenario template directory be hardcoded to tests/scenarios only, or support fallback search paths?
- Should threshold values live only in scenario template, or also as CLI flags for CI flexibility?

### 11.2 Technical Risks
| Risk | Impact | Mitigation |
|------|--------|------------|
| Current lateral baseline is high (near threshold) | Frequent false failures after threshold tightening | Keep 340.0 now, tighten gradually with baseline updates |
| ROS timing jitter in CI | flaky timeout failures | Add timeout margins and consistent CI machine profile |
| Scenario schema drift | runtime failures in custom files | strict validation + clear SCENARIO_INVALID message |

### 11.3 Assumptions
- Existing [test/run_drone_planner_test.py](test/run_drone_planner_test.py) remains primary entrypoint.
- Internal ROS mode is the only required mode for this SPEC.
- Thresholds are checked post-run using generated metrics and companion tests.
- No business-logic changes are required in planner/controller nodes for this SPEC scope.
