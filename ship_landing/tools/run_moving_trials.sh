#!/usr/bin/env bash
# Measure landing accuracy on the moving pad: several batches of landings, restarting
# the simulator between batches so the batches are independent of each other.
#
#   bash ship_landing/tools/run_moving_trials.sh                 3 batches of 10 landings, line at 0.5 m/s
#   bash ship_landing/tools/run_moving_trials.sh 1 3             1 batch of 3
#   PATTERN=circle bash ship_landing/tools/run_moving_trials.sh  the pad on its circle
#   PAD_SPEED=1.0 bash ship_landing/tools/run_moving_trials.sh   a faster pad
#   GUI=false bash ship_landing/tools/run_moving_trials.sh       without the Gazebo window (lighter)
#   NAV=flow bash ship_landing/tools/run_moving_trials.sh        GPS off, optical flow instead
#
# Results go to results/moving_<date_time>/: one CSV per batch, the simulator logs,
# summary.md and touchdowns.png. Stop with Ctrl+C; finished trials are kept.
# A batch ends early if a trial does not finish with the drone standing on the pad.

BATCHES="${1:-3}"
TRIALS="${2:-10}"
PATTERN="${PATTERN:-line}"
PAD_SPEED="${PAD_SPEED:-0.5}"
GUI="${GUI:-true}"
NAV="${NAV:-gps}"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/env.sh"
OUT="$REPO/results/moving_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

stop_simulator() {
    bash "$REPO/ship_landing/tools/stop_sim.sh" > /dev/null
}
trap 'echo; echo "Interrupted, stopping the simulator"; stop_simulator; exit 130' INT TERM

echo "Results folder: $OUT"
echo "Pad: $PATTERN at $PAD_SPEED m/s, navigation: $NAV"
echo "pattern=$PATTERN pad_speed=$PAD_SPEED nav=$NAV" > "$OUT/run_info.txt"
stop_simulator      # make sure nothing is left over from an earlier session

for batch in $(seq 1 "$BATCHES"); do
    echo
    echo "=== Batch $batch of $BATCHES: starting the simulator ==="
    (cd "$ARDU_WS" && exec ros2 launch ship_landing pad_sim.launch.py world:=moving_pad \
        pattern:="$PATTERN" pad_speed:="$PAD_SPEED" gui:="$GUI" nav:="$NAV") > "$OUT/sim_$batch.log" 2>&1 &

    # The trials node waits for the autopilot itself, so it can start straight away.
    ros2 run ship_landing moving_pad_trials --ros-args -p use_sim_time:=true \
        -p trials:="$TRIALS" -p seed:="$batch" -p batch:="batch$batch" \
        -p pattern:="$PATTERN" -p pad_speed:="$PAD_SPEED" -p output:="$OUT/batch_$batch.csv"
    echo "=== Batch $batch finished with exit code $? ==="
    stop_simulator
done

echo
python3 "$REPO/ship_landing/tools/analyze_moving_trials.py" "$OUT"/batch_*.csv --out "$OUT"
