#!/usr/bin/env bash
# Start the simulator in Docker and open its desktop in the browser. Run on the host (macOS or Linux):
#
#   sim/run.sh [WORLD] [LAUNCH_ARG ...]
#
#   sim/run.sh                          the strait world (warships, AIS), no GPS, demo flight: the default
#   sim/run.sh islands                  the islands world, demo flight, with GPS
#   sim/run.sh strait gps:=true         any extra arguments go to sim.launch.py
#   PORT=6081 sim/run.sh                the browser desktop on another port, if 6080 is taken
#
# Builds the container the first time (about 10 minutes; after changing sim/docker/, rebuild with
# docker compose build), stops any simulation already running in it, starts the
# new one, waits for the desktop, and opens http://localhost:PORT. The log is /tmp/sim.log in the container:
#   docker compose exec sim tail -50 /tmp/sim.log
# Stop with: docker compose restart (fresh container) or docker compose down (from sim/).
set -euo pipefail
cd "$(dirname "$0")"   # sim/

WORLD=${1:-strait}
[ $# -gt 0 ] && shift
export PORT=${PORT:-6080}
ARGS=("world:=$WORLD" "demo:=true" "cam_res:=512")
[ "$WORLD" = strait ] && ARGS+=("gps:=false")   # the strait run navigates on the ships, without GNSS
ARGS+=("$@")                                    # later arguments win: sim/run.sh strait gps:=true
URL="http://localhost:$PORT"

say() { echo "$(date '+%H:%M:%S') $*"; }

if ! docker info >/dev/null 2>&1; then
  echo "Docker is not running. Start Docker Desktop, wait until it says it is running, then try again." >&2
  exit 1
fi
if [ ! -f "worlds/$WORLD.sdf" ]; then
  echo "No world '$WORLD'. Choose one of: $(ls worlds | sed 's/\.sdf$//' | tr '\n' ' ')" >&2
  exit 1
fi

if docker image inspect taipeidrift-sim >/dev/null 2>&1; then
  say "starting the container"
  docker compose up -d
else
  say "building the container image (the first time takes about 10 minutes)"
  docker compose up -d --build
fi
# The container keeps /tmp across restarts, and a display lock left there stops the virtual display (Xvfb) from
# starting again: clear it before restarting, and repair the display services if they are down.
clear_display_lock() { docker compose exec -T sim rm -f /tmp/.X1-lock /tmp/.X11-unix/X1; }
if docker compose exec -T sim pgrep -f "^gz sim server" >/dev/null 2>&1; then
  say "a simulation is already running: restarting the container"
  clear_display_lock
  docker compose restart >/dev/null
  sleep 3
fi
if ! docker compose exec -T sim supervisorctl status xvfb | grep -q RUNNING; then
  say "the virtual display is down: restarting it"
  clear_display_lock
  docker compose exec -T sim supervisorctl start xvfb >/dev/null
  sleep 2
  docker compose exec -T sim supervisorctl start fluxbox x11vnc >/dev/null
fi
docker compose exec -T sim rm -f /tmp/sim.log   # the container keeps /tmp across restarts

say "launching: ros2 launch sim/launch/sim.launch.py ${ARGS[*]}"
docker compose exec -d sim bash -ic "ros2 launch sim/launch/sim.launch.py ${ARGS[*]} > /tmp/sim.log 2>&1"

say "waiting for the browser desktop on port $PORT"
for _ in $(seq 1 60); do
  curl -fs -o /dev/null "$URL" && break
  sleep 2
done
if ! curl -fs -o /dev/null "$URL"; then
  echo "Nothing answers on $URL. Is the port taken? Try: PORT=6081 sim/run.sh" >&2
  exit 1
fi

say "opening $URL (the Gazebo windows appear after one to two minutes)"
if command -v open >/dev/null; then open "$URL"
elif command -v xdg-open >/dev/null; then xdg-open "$URL" >/dev/null 2>&1
else echo "Open $URL in your browser."
fi

# Report when the simulation is up, or show the log if it failed
for _ in $(seq 1 90); do
  if docker compose exec -T sim grep -q "Traceback\|already running\|No world\|No display" /tmp/sim.log 2>/dev/null; then
    echo "The simulation did not start. Last lines of /tmp/sim.log:" >&2
    docker compose exec -T sim tail -20 /tmp/sim.log >&2
    exit 1
  fi
  if docker compose exec -T sim grep -q "Entity creation successful" /tmp/sim.log 2>/dev/null; then
    say "simulation running: $WORLD (${ARGS[*]})"
    [ "$WORLD" = strait ] && say "score the GNSS-free navigation with: docker compose exec sim bash -ic \"python3 sim/scripts/check_rf_nav.py 300\""
    exit 0
  fi
  sleep 2
done
say "still starting; follow it with: docker compose exec sim tail -f /tmp/sim.log"
