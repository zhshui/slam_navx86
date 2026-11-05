#!/bin/bash
# Wait for a map message and a usable TF before executing the launch command.
set -e
PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)
source /opt/ros/noetic/setup.bash
source "$PROJECT_ROOT/lite_cog/nav/devel/setup.bash"
MAX_WAIT=${NAV_DEPS_TIMEOUT:-30}
if [[ ! "$MAX_WAIT" =~ ^[1-9][0-9]*$ ]] || (( $# == 0 )); then
    echo "[wait_deps] Supply a command and a positive integer NAV_DEPS_TIMEOUT." >&2
    exit 2
fi

echo "[wait_deps] Waiting for /map and TF map->base_link, timeout=${MAX_WAIT}s..."
if timeout --signal=TERM --kill-after=2s "${MAX_WAIT}s" python3 - <<'PYROS'
import threading
import time
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid

started = time.monotonic()
rospy.init_node('wait_for_nav_dependencies', anonymous=True, disable_signals=True)
map_received = threading.Event()

def on_map(message):
    if message.info.width > 0 and message.info.height > 0 and message.data:
        map_received.set()

buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
subscriber = rospy.Subscriber('/map', OccupancyGrid, on_map, queue_size=1)
previous = None
while not rospy.is_shutdown():
    state = (map_received.is_set(), buffer.can_transform('map', 'base_link', rospy.Time(0)))
    if state == (True, True):
        print('[wait_deps] All dependencies ready (%.2fs)' % (time.monotonic() - started), flush=True)
        rospy.signal_shutdown('dependencies ready')
        raise SystemExit(0)
    if state != previous:
        print('[wait_deps] map=%s tf=%s' % state, flush=True)
        previous = state
    time.sleep(0.1)
raise SystemExit(1)
PYROS
then
    exec "$@"
else
    rc=$?
    echo "[wait_deps] Dependency check failed or timed out (exit=$rc); command NOT started." >&2
    exit "$rc"
fi
