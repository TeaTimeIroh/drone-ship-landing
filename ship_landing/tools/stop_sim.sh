#!/usr/bin/env bash
# Stop the simulator and everything started with it.
#
#   bash ship_landing/tools/stop_sim.sh
#
# This is a script, not a one-line command, on purpose: "pkill -f" matches against whole
# command lines, and a one-liner that contains the names it is looking for kills its own
# shell part-way through.

pkill -INT -f 'ros2 launch ship_landing' 2>/dev/null
sleep 4
for name in 'gz sim' arducopter mavproxy marker_detector pad_mover micro_ros_agent \
            parameter_bridge robot_state_publisher rviz2 rqt_image_view; do
    pkill -f "$name" 2>/dev/null
done
sleep 2
for name in 'gz sim' arducopter; do      # anything that ignored the polite request
    pkill -9 -f "$name" 2>/dev/null
done

left="$(pgrep -a -f 'gz sim|arducopter|mavproxy' || true)"
if [ -n "$left" ]; then
    echo "Still running:"
    echo "$left"
    exit 1
fi
echo "stopped"
