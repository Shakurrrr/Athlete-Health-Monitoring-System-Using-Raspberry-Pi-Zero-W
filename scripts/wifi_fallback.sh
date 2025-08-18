#!/usr/bin/env bash
set -e
for i in {1..45}; do
  if nmcli -t -f STATE g | grep -q connected; then
    exit 0
  fi
  sleep 1
done
nmcli con up AthleteSetup || true
