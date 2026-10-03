#!/usr/bin/env bash
# Fly the video's demo flight again, or under a changed condition, with every estimator logged against the truth
# and without recording the screen. Run on the host:
#
#   scripts/demo_flight_batch.sh NAME ["EXTRA LAUNCH ARGS"] [WALL_LIMIT_S]
#
#   scripts/demo_flight_batch.sh r1                              the demo flight as in the video, new random noise
#   scripts/demo_flight_batch.sh wind "wind:=6,20"               with wind: extra arguments win over the demo's own
#   scripts/demo_flight_batch.sh long "route:=pads land:=false" 600   to island B and back, until the time limit
#   PLAIN_SEA=1 scripts/demo_flight_batch.sh plainsea            the sea without any texture for this flight: real
#                                                                waves give a camera nothing to track (only for a
#                                                                world built before commit 18bbbdd, whose sea has a
#                                                                texture; since then the generator builds it plain)
#   DAN_CHECK_S=230 scripts/demo_flight_batch.sh r1              also run sim/scripts/check_rf_nav.py that long
#
# Logs go to outputs/demo/batch/NAME (estimators.csv, status.jsonl, sim.log); score them with
# scripts/score_demo_flights.py. A flight ends when the drone has landed or after WALL_LIMIT_S seconds (default
# 480). The simulator runs at its normal pace, about 0.4 of real time. A start that crashes (Gazebo's window does
# now and then) is tried once more.
set -uo pipefail
cd "$(dirname "$0")/../sim"
NAME=$1
EXTRA=${2:-}
LIMIT=${3:-480}
OUT=outputs/demo/batch/$NAME      # from the repository root, on the host and in the container
DEMO="gnss_cutoff_s:=26 route:=crossing land:=true metric_flow:=true flow_min_range_m:=10 flow_update_every_n:=2 flow_max_dt_s:=0.5 flow_soft_limit:=9.21 vision_rotation:=false vision_direction:=false ais_start_s:=41"
ORIGIN="-p gps_origin_latitude:=23.65 -p gps_origin_longitude:=119.85 -p gps_origin_elevation:=4.0"
SEA=models/islands/materials/textures/sea.jpg

say() { echo "$(date '+%H:%M:%S') $NAME: $*"; }
sim() { docker compose exec -T sim "$@"; }
restart() {
  sim rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 >/dev/null 2>&1
  docker compose restart >/dev/null 2>&1
  for _ in $(seq 1 40); do sim supervisorctl status xvfb 2>/dev/null | grep -q RUNNING && break; sleep 1; done
  sleep 3
}
restore_sea() { if [ -f "$SEA.normal" ]; then mv -f "$SEA.normal" "$SEA"; fi; }
failed() { sim cat /tmp/sim.log > "../outputs/demo/batch/${NAME}_failed_start.log" 2>/dev/null; return 1; }
trap restore_sea EXIT

fly() {
  restart
  rm -rf "../$OUT"
  mkdir -p "../$OUT"
  sim rm -f /tmp/sim.log
  docker compose exec -d sim bash -ic "python3 sim/nodes/eskf_ros_adapter.py --ros-args $ORIGIN -p publish_tf:=false -p vision_rotation:=false -p vision_direction:=false -p metric_flow:=false -r __node:=eskf_inertial -r /nav/odom:=/nav_inertial/odom -r /nav/estimator_status:=/nav_inertial/estimator_status > /tmp/inertial.log 2>&1"
  docker compose exec -d sim bash -ic "python3 sim/scripts/log_two_estimators.py --out $OUT/estimators.csv --topics camera=/nav/odom ours=/nav_rf/odom inertial=/nav_inertial/odom ships=/rf_nav/odom > /tmp/logger.log 2>&1"
  docker compose exec -d sim bash -ic "python3 sim/scripts/log_estimator_status.py $OUT/status.jsonl camera=/nav/estimator_status ours=/nav_rf/estimator_status inertial=/nav_inertial/estimator_status > /tmp/status.log 2>&1"
  sleep 4
  docker compose exec -d sim bash -ic "ros2 launch sim/launch/sim.launch.py world:=strait demo:=true cam_res:=512 $DEMO $EXTRA > /tmp/sim.log 2>&1"
  if [ -n "${DAN_CHECK_S:-}" ]; then
    docker compose exec -d sim bash -ic "python3 sim/scripts/check_rf_nav.py $DAN_CHECK_S --out $OUT/dan_check > /tmp/check.log 2>&1"
  fi
  local start=$SECONDS
  while [ $((SECONDS - start)) -lt "$LIMIT" ]; do
    sleep 10
    if sim grep -q "landed" /tmp/sim.log 2>/dev/null; then break; fi
    if sim grep -q "Segmentation fault\|Traceback" /tmp/sim.log 2>/dev/null; then failed; return 1; fi
    if [ $((SECONDS - start)) -gt 150 ] && ! sim grep -q "at height" /tmp/sim.log 2>/dev/null; then failed; return 1; fi
  done
  sleep 8
  if [ -n "${DAN_CHECK_S:-}" ]; then
    for _ in $(seq 1 30); do sim pgrep -f "[c]heck_rf_nav.py" >/dev/null 2>&1 || break; sleep 5; done
    sim cat /tmp/check.log > "../$OUT/dan_check.txt" 2>/dev/null
  fi
  sim bash -c 'pkill -INT -f "[l]og_two_estimators.py"; pkill -INT -f "[l]og_estimator_status.py"; sleep 2'
  sim cat /tmp/sim.log > "../$OUT/sim.log" 2>/dev/null
  return 0
}

if [ "${PLAIN_SEA:-0}" = 1 ]; then
  cp "$SEA" "$SEA.normal"
  sim python3 -c "from PIL import Image; p = 'sim/$SEA'; im = Image.open(p); Image.new('RGB', im.size, (14, 56, 108)).save(p, quality=92)"
  say "the sea has no texture for this flight"
fi
say "starting ($DEMO $EXTRA)"
if ! fly; then
  say "the start failed (log: outputs/demo/batch/${NAME}_failed_start.log), trying once more"
  fly || say "FAILED twice"
fi
restart
say "done: $(wc -l < "../$OUT/estimators.csv" 2>/dev/null | tr -d ' ') log rows, $(grep -o 'landed, [0-9.]* m from the pad centre' "../$OUT/sim.log" 2>/dev/null || echo 'no landing')"
