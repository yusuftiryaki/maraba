#!/usr/bin/env bash
# Install deploy/robot.service on the Pi (docs/deployment.md, step 8).
# Run ON the Pi, from the repo:   ./scripts/install_service.sh [--now]
#   --now   also (re)start it — the robot starts moving its stack up.

set -euo pipefail
cd "$(dirname "$0")/.."

unit=deploy/robot.service
site=$(grep -o 'site_file:=[^ ]*' "$unit" | cut -d= -f2)
if [ -z "$site" ]; then
  echo "!! $unit has no site_file:= — the robot would run on an empty site" >&2
  exit 1
fi
if [ ! -d "$(dirname "$site")" ]; then
  echo "!! $(dirname "$site") does not exist (site_file:=$site)" >&2
  exit 1
fi

sudo install -m 644 "$unit" /etc/systemd/system/robot.service
sudo systemctl daemon-reload
sudo systemctl enable robot.service
if [ "${1:-}" = "--now" ]; then
  sudo systemctl restart robot.service
  echo ">> robot.service restarted — journalctl -u robot.service -f"
else
  echo ">> installed + enabled; starts at boot (or: sudo systemctl restart robot.service)"
fi
