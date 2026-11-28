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
#   LANDER=lean NAV=flow bash ship_landing/tools/run_moving_trials.sh   land by leaning, camera only (step 5d)
#   WORLD=ship SHIP_SPEED=2.0 bash ship_landing/tools/run_moving_trials.sh   land on the boat (step 6); always
#                                                                by leaning and on optical flow
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
LANDER="${LANDER:-velocity}"      # velocity: GUIDED velocity commands; lean: ALT_HOLD sticks, camera only
WORLD="${WORLD:-moving_pad}"      # moving_pad, or ship: the boat with the pad on its deck
SHIP_SPEED="${SHIP_SPEED:-2.0}"
if [ "$WORLD" = "ship" ]; then
    LANDER=lean; NAV=flow; PATTERN=straight; PAD_SPEED="$SHIP_SPEED"
fi

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/env.sh"
OUT="$REPO/results/moving_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

stop_simulator() {
    bash "$REPO/ship_landing/tools/stop_sim.sh" > /dev/null
}
trap 'echo; echo "Interrupted, stopping the simulator"; stop_simulator; exit 130' INT TERM

echo "Results folder: $OUT"
echo "Pad: $PATTERN at $PAD_SPEED m/s, navigation: $NAV, landing by: $LANDER"
echo "world=$WORLD pattern=$PATTERN pad_speed=$PAD_SPEED nav=$NAV lander=$LANDER" > "$OUT/run_info.txt"
stop_simulator      # make sure nothing is left over from an earlier session

for batch in $(seq 1 "$BATCHES"); do
    echo
    echo "=== Batch $batch of $BATCHES: starting the simulator ==="
    (cd "$ARDU_WS" && exec ros2 launch ship_landing pad_sim.launch.py world:="$WORLD" \
        pattern:="$PATTERN" pad_speed:="$PAD_SPEED" gui:="$GUI" nav:="$NAV") > "$OUT/sim_$batch.log" 2>&1 &

    # The trials node waits for the autopilot itself, so it can start straight away.
    if [ "$WORLD" = "ship" ]; then
        ros2 run ship_landing lean_trials --ros-args -p use_sim_time:=true \
            -p trials:="$TRIALS" -p seed:="$batch" -p batch:="batch$batch" \
            -p pattern:=straight -p ship_speed:="$SHIP_SPEED" -p pad_x:=0.0 -p pad_y:=0.0 -p pad_z:=0.0 \
            -p results:="$OUT/batch_$batch.csv" -p output:="$OUT" -p csv_name:="flight_$batch.csv"
    elif [ "$LANDER" = "lean" ]; then
        ros2 run ship_landing lean_trials --ros-args -p use_sim_time:=true \
            -p trials:="$TRIALS" -p seed:="$batch" -p batch:="batch$batch" \
            -p pattern:="$PATTERN" -p pad_speed:="$PAD_SPEED" -p results:="$OUT/batch_$batch.csv" \
            -p output:="$OUT" -p csv_name:="flight_$batch.csv"
    else
        ros2 run ship_landing moving_pad_trials --ros-args -p use_sim_time:=true \
            -p trials:="$TRIALS" -p seed:="$batch" -p batch:="batch$batch" \
            -p pattern:="$PATTERN" -p pad_speed:="$PAD_SPEED" -p output:="$OUT/batch_$batch.csv"
    fi
    echo "=== Batch $batch finished with exit code $? ==="
    stop_simulator
done

echo
python3 "$REPO/ship_landing/tools/analyze_moving_trials.py" "$OUT"/batch_*.csv --out "$OUT"
