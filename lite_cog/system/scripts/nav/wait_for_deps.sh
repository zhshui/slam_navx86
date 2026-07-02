#!/bin/bash
# ============================================
# Wait for move_base dependencies before launching
# Used as launch-prefix in navigation.launch
# ============================================
source /opt/ros/noetic/setup.bash

MAX_WAIT=30

echo "[wait_deps] Waiting for /map topic and TF (map->base_link), timeout=${MAX_WAIT}s..."
start_ts=$(date +%s)

while true; do
    elapsed=$(($(date +%s) - start_ts))
    if [ $elapsed -ge $MAX_WAIT ]; then
        echo "[wait_deps] Timeout after ${MAX_WAIT}s, launching move_base anyway..."
        break
    fi

    # Check 1: /map topic has a publisher
    map_info=$(timeout 2 rostopic info /map 2>/dev/null)
    if ! echo "$map_info" | grep -q 'Publishers:.*http'; then
        sleep 1
        continue
    fi

    # Check 2: TF from map to base_link exists
    tf_line=$(timeout 2 bash -c 'source /opt/ros/noetic/setup.bash && rosrun tf tf_echo map base_link 2>/dev/null' | head -1)
    if ! echo "$tf_line" | grep -q 'At time'; then
        sleep 1
        continue
    fi

    echo "[wait_deps] All dependencies ready (${elapsed}s)"
    break
done

exec "$@"
