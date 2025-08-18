#!/usr/bin/env bash
set -euo pipefail

# Base tools
sudo apt update
sudo apt install -y python3-venv python3-pip git i2c-tools mosquitto mosquitto-clients

# Enable interfaces
sudo raspi-config nonint do_i2c 0 || true
sudo raspi-config nonint do_onewire 0 || true

# Python env
bash "$(dirname "$0")/install_python.sh"

# Node-RED local install
bash "$(dirname "$0")/install_node_red.sh"

# Services
bash "$(dirname "$0")/install_services.sh"

echo "Setup complete. Node-RED at http://<pi-ip>:1880/ui"
