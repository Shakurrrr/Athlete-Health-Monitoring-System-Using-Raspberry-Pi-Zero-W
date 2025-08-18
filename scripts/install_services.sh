#!/usr/bin/env bash
set -euo pipefail
REPO_DIR="/home/pi/athlete-monitor-pi"

sudo install -m 644 "$REPO_DIR/systemd/athlete-monitor.service" /etc/systemd/system/
sudo install -m 644 "$REPO_DIR/systemd/nodered.service" /etc/systemd/system/
sudo install -m 644 "$REPO_DIR/systemd/wifi-nosleep.service" /etc/systemd/system/
sudo install -m 755 "$REPO_DIR/scripts/wifi_fallback.sh" /usr/local/bin/wifi_fallback.sh || true
if [ -f "$REPO_DIR/systemd/wifi-fallback.service" ]; then
  sudo install -m 644 "$REPO_DIR/systemd/wifi-fallback.service" /etc/systemd/system/
fi

sudo systemctl daemon-reload
sudo systemctl enable --now wifi-nosleep.service
sudo systemctl enable --now nodered.service
sudo systemctl enable --now athlete-monitor.service
[ -f /etc/systemd/system/wifi-fallback.service ] && sudo systemctl enable --now wifi-fallback.service || true

echo "Services enabled."
