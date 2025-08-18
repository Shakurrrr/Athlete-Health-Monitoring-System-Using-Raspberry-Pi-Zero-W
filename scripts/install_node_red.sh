#!/usr/bin/env bash
set -euo pipefail
mkdir -p /home/pi/.node-red
cd /home/pi/.node-red
npm init -y >/dev/null 2>&1 || true
npm install --no-audit --no-fund node-red@3.1.10 node-red-dashboard
echo "Node-RED installed locally in ~/.node-red"
