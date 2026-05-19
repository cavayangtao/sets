#!/usr/bin/env bash
set -u

# Usage: bash benchmark_stability.sh [seed_start] [seed_end]
SEED_START="${1:-0}"
SEED_END="${2:-9}"

SETS_ROOT="/home/tyang/sets"
CSV_OUT="${SETS_ROOT}/plots/tello_two_wp_s3_seed_stats.csv"

PY_RUN=(/home/tyang/anaconda3/bin/conda run -p /home/tyang/anaconda3 --no-capture-output python)

source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}

cleanup() {
    pkill -f "tello_planner_node.py|run_tello_two_waypoints_auto.py" 2>/dev/null || true
    if [[ -n "${RCPID:-}" ]]; then
        kill "${RCPID}" 2>/dev/null || true
    fi
    pkill -f "roscore|rosmaster|roslaunch" 2>/dev/null || true
}
trap cleanup EXIT

pkill -f "roscore|rosmaster|roslaunch|tello_planner_node.py|run_tello_two_waypoints_auto.py" || true
roscore >/tmp/roscore_bench.log 2>&1 &
RCPID=$!

READY=no
for _ in $(seq 1 80); do
    if rosparam list >/dev/null 2>&1; then
        READY=yes
        break
    fi
    sleep 0.5
done

if [[ "${READY}" != "yes" ]]; then
    echo "roscore not ready"
    exit 1
fi

echo "seed,status,error,final_dist_wp1,min_dist_wp1,duration_s,num_samples,path_efficiency,total_turn_rad,cmd_delta_rms,min_obstacle_clearance" > "${CSV_OUT}"

for s in $(seq "${SEED_START}" "${SEED_END}"); do
    "${PY_RUN[@]}" "${SETS_ROOT}/scripts/tello_planner_node.py" \
        _config_name:=policy_convergence_tello_stage3 \
        _uct_N:=10000 _uct_max_depth:=10 _uct_c:=0.7 _uct_wct:=2200.0 \
        _uct_mpc_depth:=2 _pose_timeout:=8.0 _max_cmd_hold:=12.0 \
        _planner_rate:=1.0 _cmd_pub_rate:=30.0 _seed:="${s}" \
        >/tmp/planner_s3_seed_${s}.log 2>&1 &
    LPID=$!

    SERVICE=no
    for _ in $(seq 1 120); do
        if rosservice list 2>/dev/null | grep -q "/tello_planner/arm"; then
            SERVICE=yes
            break
        fi
        sleep 0.5
    done

    if [[ "${SERVICE}" != "yes" ]]; then
        echo "${s},failed,service_not_ready,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A" >> "${CSV_OUT}"
        kill "${LPID}" 2>/dev/null || true
        wait "${LPID}" 2>/dev/null || true
        continue
    fi

    "${PY_RUN[@]}" "${SETS_ROOT}/scripts/run_tello_two_waypoints_auto.py" \
        --use-ros-test-node --sim-mode rollout --sim-config-name policy_convergence_tello_stage3 \
        --wp-timeout 150 --reach-dist 2.0 \
        --out-prefix "${SETS_ROOT}/plots/tello_two_wp_s3_seed${s}" \
        >/tmp/runner_s3_seed_${s}.log 2>&1

    REPORT="${SETS_ROOT}/plots/tello_two_wp_s3_seed${s}_report.json"
    if [[ -f "${REPORT}" ]]; then
        python3 - <<PY >> "${CSV_OUT}"
import json
seed = ${s}
with open("${REPORT}", "r", encoding="utf-8") as f:
        d = json.load(f)
m = d.get("metrics", {})
err = str(d.get("error", "")).replace(",", ";")
print(
        f"{seed},{d.get('status','N/A')},{err},"
        f"{m.get('final_dist_wp1','N/A')},{m.get('min_dist_wp1','N/A')},"
        f"{m.get('duration_s','N/A')},{m.get('num_samples','N/A')},"
        f"{m.get('path_efficiency','N/A')},{m.get('total_turn_rad','N/A')},"
        f"{m.get('cmd_delta_rms','N/A')},{m.get('min_obstacle_clearance','N/A')}"
)
PY
    else
        echo "${s},failed,no_report,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A" >> "${CSV_OUT}"
    fi

    kill "${LPID}" 2>/dev/null || true
    wait "${LPID}" 2>/dev/null || true
    pkill -f "tello_planner_node.py" 2>/dev/null || true
    sleep 1
done

cat "${CSV_OUT}"

python3 - <<PY
import csv
import numpy as np

csv_path = "${CSV_OUT}"
metrics = [
        "final_dist_wp1",
        "min_dist_wp1",
        "duration_s",
        "path_efficiency",
        "total_turn_rad",
        "cmd_delta_rms",
        "min_obstacle_clearance",
]

rows = []
with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
                rows.append(r)

def to_float(v):
        try:
                return float(v)
        except Exception:
                return np.nan

def stat(block, key):
        vals = np.array([to_float(r[key]) for r in block], dtype=np.float64)
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
                return np.nan, np.nan
        return float(np.mean(vals)), float(np.var(vals, ddof=0))

total = len(rows)
success_rows = [r for r in rows if r.get("status") == "success"]
print("\nSuccess Count: %d/%d" % (len(success_rows), total))
print("Success Rate: %.4f" % (len(success_rows) / max(1, total)))

print("\nStats for Successful Runs:")
for k in metrics:
        mean, var = stat(success_rows, k)
        print("%s: mean=%.6f, var=%.6f" % (k, mean, var))

print("\nStats for All Runs:")
for k in metrics:
        mean, var = stat(rows, k)
        print("%s: mean=%.6f, var=%.6f" % (k, mean, var))

failed = [r for r in rows if r.get("status") != "success"]
if failed:
        print("\nFailed Seeds:")
        for r in failed:
                print("seed=%s status=%s error=%s" % (r.get("seed"), r.get("status"), r.get("error", "")))
PY
