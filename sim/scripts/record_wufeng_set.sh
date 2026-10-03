#!/usr/bin/env bash
# Record simulated flights over Wufeng for the camera navigator: recordings/<name>/ in the taipeidrift-replay/1 format.
#
# Each flight is a fresh start of the simulator without its window: the recorder starts, the drone climbs on the
# spot and flies its route (sim/scripts/route_flight.py), and the recorder stops after the given simulation time.
# A flight whose recording already exists is skipped.
#
#   sim/scripts/record_wufeng_set.sh "NAME ROUTE DURATION_S" ...     on the host
#
# Example: sim/scripts/record_wufeng_set.sh "wufeng_north_90m sim/scenarios/wufeng_north_90m.json 720"
# DURATION_S must cover the climb, the route and a little hover (route length / speed times 1.1, plus about a minute).
# Progress goes to recordings/record_set.log.
set -euo pipefail
cd "$(dirname "$0")/.."   # sim/
mkdir -p ../recordings
LOG=../recordings/record_set.log

say() { echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
in_sim() { docker compose exec -T sim bash -lc "source /opt/ros/jazzy/setup.bash; $1"; }
wait_for() {  # wait until a pattern shows up in a log file in the container
  local file=$1 pattern=$2 timeout=$3 t=0
  until in_sim "grep -q '$pattern' $file 2>/dev/null"; do
    sleep 5; t=$((t + 5))
    if [ "$t" -ge "$timeout" ]; then say "timed out waiting for '$pattern' in $file"; return 1; fi
  done
}

for flight in "$@"; do
  read -r name route duration <<<"$flight"
  if [ -e "../recordings/$name" ]; then say "skip $name: recordings/$name exists"; continue; fi
  say "flight $name: $route, recording $duration s of simulation time"
  docker compose restart >/dev/null
  sleep 5
  in_sim "rm -f /tmp/sim.log /tmp/recorder.log /tmp/flight.log"  # the container keeps /tmp across restarts
  docker compose exec -d sim bash -ic "cd /ws/TaipeiDrift && ros2 launch sim/launch/sim.launch.py cam_res:=512 gui:=false world:=terrain > /tmp/sim.log 2>&1"
  sleep 25
  docker compose exec -d sim bash -ic "cd /ws/TaipeiDrift && python3 sim/nodes/recorder.py --name $name --world terrain --image-rate-hz 5 --cam-res 512 --duration-s $duration > /tmp/recorder.log 2>&1"
  sleep 5
  docker compose exec -d sim bash -ic "cd /ws/TaipeiDrift && python3 sim/scripts/route_flight.py --route $route > /tmp/flight.log 2>&1"
  wait_for /tmp/flight.log "^done:" $((duration * 3))
  in_sim "tail -1 /tmp/flight.log" | tee -a "$LOG"
  wait_for /tmp/recorder.log "Recording stopped" $((duration * 3))
  in_sim "grep -o 'Recording stopped.*' /tmp/recorder.log" | tee -a "$LOG"
done
say "set done"
