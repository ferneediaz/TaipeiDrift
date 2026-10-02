#!/usr/bin/env bash
# Download the team's Mid-Air subset: about 10 GB instead of the full 100 GB.
#
#   scripts/fetch_midair.sh path/to/links.txt
#
# links.txt is the file the Mid-Air download page sends after the form
# (https://midair.ulg.ac.be/download.html, image type "Down RGB", everything else as it is).
# It contains a personal download ID, so keep it out of git.
#
# What is fetched:
#   - the sensor records of every flight in every condition (0.8 GB)
#   - downward camera, flights 0000-0005 in sun and the same six in fog (2000-2005)
#   - downward camera, three flights each in spring, fall and winter
#
# The server cannot resume a broken download. Files that are already complete are skipped,
# so run the script again after an interruption and delete the one half-written file first.
set -euo pipefail

links="${1:?usage: scripts/fetch_midair.sh path/to/links.txt}"
dest="$(cd "$(dirname "$0")/.." && pwd)/data/raw/midair"
mkdir -p "$dest"

todo="$(mktemp)"
{
  grep 'sensor_records.zip' "$links"
  grep -E 'Kite_training/(sunny|foggy)/color_down/trajectory_[02]00[0-5]/' "$links"
  grep -E 'PLE_training/(spring|fall|winter)/color_down/trajectory_[456]00[0-2]/' "$links"
} | while read -r url; do
  path="$(echo "$url" | sed -E 's#^https?://[^/]+/##; s/\?id=.*//')"
  [ -f "$dest/$path" ] || echo "$url"
done > "$todo"

echo "$(wc -l < "$todo" | tr -d ' ') files to fetch into $dest"
cd "$dest"
wget --content-disposition -x -nH --tries=3 --timeout=60 -nv -i "$todo"
rm -f "$todo"
