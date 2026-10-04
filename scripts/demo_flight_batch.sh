#!/usr/bin/env bash
# Fly the video's demo flight again, or under a changed condition, with every estimator logged against the truth
# and without recording the screen. Run on the host:
#
#   scripts/demo_flight_batch.sh NAME ["EXTRA LAUNCH ARGS"] [WALL_LIMIT_S]
#
#   scripts/demo_flight_batch.sh r1                              the demo flight as in the video, new random noise
#                                                                (ships transmitting from 20 s, the RF display from
#                                                                41 s; the flights of the night of 3 to 4 October
#                                                                used "ais_start_s:=41 rf_display_after_s:=0")
#   scripts/demo_flight_batch.sh wind "wind:=6,20"               with wind: extra arguments win over the demo's own
#   scripts/demo_flight_batch.sh long "route:=pads land:=false" 600   to island B and back, until the time limit
#   PLAIN_SEA=1 scripts/demo_flight_batch.sh plainsea            the sea without any texture for this flight: real
#                                                                waves give a camera nothing to track (only for a
#                                                                world built before commit 18bbbdd, whose sea has a
#                                                                texture; since then the generator builds it plain)
#   DAN_CHECK_S=230 scripts/demo_flight_batch.sh r1              also run sim/scripts/check_rf_nav.py that long
#   RECORD=1 scripts/demo_flight_batch.sh take3                  a take for the video: the simulator is slowed to
#                                                                0.12 of real time (else the 3D view stutters) and
#                                                                the desktop is captured into NAME/frames; about
#                                                                16 minutes. Watch it at http://localhost:6080.
#                                                                Then: python scripts/make_demo_video.py
#                                                                outputs/demo/batch/take3 --out VIDEO.mp4 ...
#
# Logs go to outputs/demo/batch/NAME (estimators.csv, status.jsonl, sim.log); a folder of that name is replaced.
# Score them with scripts/score_demo_flights.py. A flight ends when the drone has landed or after WALL_LIMIT_S
# seconds (default 480; 1500 with RECORD=1). The simulator runs at its normal pace, about 0.4 of real time. A start that crashes or hangs (Gazebo
# does both now and then) is tried once more.
set -uo pipefail
cd "$(dirname "$0")/../sim"
NAME=$1
EXTRA=${2:-}
RECORD=${RECORD:-0}
if [ "$RECORD" = 1 ]; then LIMIT=${3:-1500}; CLIMB_BY=420; else LIMIT=${3:-480}; CLIMB_BY=150; fi
OUT=outputs/demo/batch/$NAME      # from the repository root, on the host and in the container
DEMO="gnss_cutoff_s:=26 route:=crossing land:=true metric_flow:=true flow_min_range_m:=10 flow_update_every_n:=2 flow_max_dt_s:=0.5 flow_soft_limit:=9.21 vision_rotation:=false vision_direction:=false ais_start_s:=20 rf_display_after_s:=41"
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
  # the world is up when the drone has been placed in it; Gazebo's server sometimes hangs before that
  for _ in $(seq 1 60); do sim grep -q "Entity creation successful" /tmp/sim.log 2>/dev/null && break; sleep 1; done
  if ! sim grep -q "Entity creation successful" /tmp/sim.log 2>/dev/null; then failed; return 1; fi
  if [ "$RECORD" = 1 ]; then
    sim bash -ic 'gz service -s /world/strait/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 3000 --req "max_step_size: 0.001, real_time_factor: 0.12"' >/dev/null 2>&1
    docker compose exec -d sim bash -ic "python3 sim/scripts/capture_desktop.py --out $OUT/frames --fps 60 --max-s 1500 > /tmp/capture.log 2>&1"
    say "recording: the simulator runs at 0.12 of real time, the desktop is captured into $OUT/frames"
  fi
  local start=$SECONDS
  while [ $((SECONDS - start)) -lt "$LIMIT" ]; do
    sleep 10
    if sim grep -q "landed" /tmp/sim.log 2>/dev/null; then break; fi
    if sim grep -q "Segmentation fault\|Traceback" /tmp/sim.log 2>/dev/null; then failed; return 1; fi
    if [ $((SECONDS - start)) -gt "$CLIMB_BY" ] && ! sim grep -q "at height" /tmp/sim.log 2>/dev/null; then failed; return 1; fi
  done
  if [ "$RECORD" = 1 ]; then sleep 30; else sleep 8; fi      # a few seconds on the ground at the end
  if [ -n "${DAN_CHECK_S:-}" ]; then
    for _ in $(seq 1 30); do sim pgrep -f "[c]heck_rf_nav.py" >/dev/null 2>&1 || break; sleep 5; done
    sim cat /tmp/check.log > "../$OUT/dan_check.txt" 2>/dev/null
  fi
  sim bash -c 'pkill -INT -f "[c]apture_desktop.py"; pkill -INT -f "[l]og_two_estimators.py"; pkill -INT -f "[l]og_estimator_status.py"; sleep 2'
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
say "done: $(wc -l < "../$OUT/estimators.csv" 2>/dev/null | tr -d ' ') log rows, $(grep -o 'landed, [0-9.]* m from the pad centre' "../$OUT/sim.log" 2>/dev/null || echo 'no landing')$([ "$RECORD" = 1 ] && echo ", $(ls "../$OUT/frames" 2>/dev/null | wc -l | tr -d ' ') frames")"
