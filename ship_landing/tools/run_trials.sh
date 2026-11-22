#!/usr/bin/env bash
# Measure landing accuracy: several batches of landings, restarting the simulator
# between batches so the batches are independent of each other.
#
#   ship_landing/tools/run_trials.sh              3 batches of 10 landings
#   ship_landing/tools/run_trials.sh 2 5          2 batches of 5
#   GUI=false ship_landing/tools/run_trials.sh    without the Gazebo window (lighter)
#   NAV=flow ship_landing/tools/run_trials.sh     GPS off, optical flow instead
#
# Results go to results/<date_time>/: one CSV per batch, the simulator logs,
# summary.md and touchdowns.png. Stop with Ctrl+C; finished trials are kept.

BATCHES="${1:-3}"
TRIALS="${2:-10}"
GUI="${GUI:-true}"
NAV="${NAV:-gps}"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/env.sh"
OUT="$REPO/results/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

stop_simulator() {
    pkill -INT -f 'ros2 launch ship_landing' 2>/dev/null
    sleep 5
    for name in 'gz sim' arducopter mavproxy marker_detector micro_ros_agent parameter_bridge robot_state_publisher; do
        pkill -f "$name" 2>/dev/null
    done
    sleep 3
}
trap 'echo; echo "Interrupted, stopping the simulator"; stop_simulator; exit 130' INT TERM

echo "Results folder: $OUT"
stop_simulator      # make sure nothing is left over from an earlier session

for batch in $(seq 1 "$BATCHES"); do
    echo
    echo "=== Batch $batch of $BATCHES: starting the simulator ==="
    (cd "$ARDU_WS" && exec ros2 launch ship_landing pad_sim.launch.py gui:="$GUI" nav:="$NAV") > "$OUT/sim_$batch.log" 2>&1 &

    # The trials node waits for the autopilot itself, so it can start straight away.
    ros2 run ship_landing landing_trials --ros-args -p use_sim_time:=true \
        -p trials:="$TRIALS" -p seed:="$batch" -p batch:="batch$batch" -p output:="$OUT/batch_$batch.csv"
    echo "=== Batch $batch finished with exit code $? ==="
    stop_simulator
done

echo
python3 "$REPO/ship_landing/tools/analyze_trials.py" "$OUT"/batch_*.csv --out "$OUT"
