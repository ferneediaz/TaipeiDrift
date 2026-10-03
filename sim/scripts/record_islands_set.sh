#!/usr/bin/env bash
# Record a set of flights over the islands world in the Mid-Air format: data/sim/islands/sunny/.
#
# Each flight is a fresh start of the simulator (a new IMU noise draw, as Mid-Air draws one per flight): the drone
# takes off from helipad A, recording starts once it is at height, and stops when the route is flown once.
#
#   sim/scripts/record_islands_set.sh [FLIGHT ...]     on the host
#
# A flight is "route height spacing [first-waypoint]": route pads or survey, height above helipad A in m, survey line
# spacing in m (0 for pads), and optionally the waypoint to start at, to continue an interrupted survey.
# Default set: "survey 40 60" "survey 80 120" "pads 40 0" "pads 60 0" "pads 80 0" "pads 100 0",
# about 2 hours and 8 GB. Progress goes to data/sim/islands/record_set.log.
set -euo pipefail
cd "$(dirname "$0")/.."   # sim/
OUT=../data/sim/islands
mkdir -p "$OUT"
LOG="$OUT/record_set.log"
FLIGHTS=("$@")
[ ${#FLIGHTS[@]} -eq 0 ] && FLIGHTS=("survey 40 60" "survey 80 120" "pads 40 0" "pads 60 0" "pads 80 0" "pads 100 0")

say() { echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
in_sim() { docker compose exec -T sim bash -lc "source /opt/ros/jazzy/setup.bash; $1"; }
wait_for() {  # wait until a pattern shows up in a log file in the container
  local file=$1 pattern=$2 timeout=$3 t=0
  until in_sim "grep -q '$pattern' $file 2>/dev/null"; do
    sleep 5; t=$((t + 5))
    if [ "$t" -ge "$timeout" ]; then say "timed out waiting for '$pattern' in $file"; return 1; fi
  done
}

for flight in "${FLIGHTS[@]}"; do
  read -r route height spacing start <<<"$flight"
  start=${start:-0}
  say "flight: $route at $height m$([ "$spacing" != 0 ] && echo " (lines $spacing m apart)")$([ "$start" != 0 ] && echo ", from waypoint $start")"
  docker compose restart >/dev/null
  sleep 5
  in_sim "rm -f /tmp/demo.log /tmp/rec.log /tmp/rec.pid"  # the container keeps /tmp across restarts
  in_sim "python3 -c 'import h5py' 2>/dev/null || (apt-get update -qq && apt-get install -y -qq --no-install-recommends python3-h5py >/dev/null)"
  docker compose exec -d sim bash -ic "ros2 launch sim/launch/sim.launch.py world:=islands gui:=false > /tmp/sim.log 2>&1"
  sleep 25
  args="--world islands --route $route --height $height --once --from-waypoint $start"
  [ "$spacing" != 0 ] && args="$args --spacing $spacing"
  docker compose exec -d sim bash -ic "python3 sim/nodes/demo_flight.py $args > /tmp/demo.log 2>&1"
  wait_for /tmp/demo.log "at height" 600
  docker compose exec -d sim bash -ic "echo \$\$ > /tmp/rec.pid; exec python3 sim/nodes/record_midair.py --world islands > /tmp/rec.log 2>&1"
  wait_for /tmp/demo.log "route done" 14400
  in_sim 'kill -INT $(cat /tmp/rec.pid)'
  wait_for /tmp/rec.log "wrote trajectory\|no camera frames\|nothing written" 600
  in_sim "grep -E 'wrote|WARN|ERROR' /tmp/rec.log" | tee -a "$LOG"
done
docker compose restart >/dev/null
say "set done: $(du -sh "$OUT/sunny" | cut -f1) in $OUT/sunny"
