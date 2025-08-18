# Athlete Monitor (Raspberry Pi Zero W)

Turn a Pi Zero W into a near–real-time vital signs node:
- **MAX30102**: Heart rate & SpO₂ (4 s sliding window, 0.5 s updates)
- **DS18B20**: Temperature
- **MQTT** topic: `athlete/data`
- **Node-RED Dashboard**: Gauges + trends
- Wi-Fi status LED on **GPIO27**

## Hardware
- Raspberry Pi Zero W (Rev 1.1)
- MAX30102 on I²C (address `0x57`)
- DS18B20 on 1-Wire
- Green LED to GPIO27 (series resistor)

## OS
Raspberry Pi OS Bookworm (armhf). Enable **I²C** and **1-Wire**.

## Quick Start
```bash
git clone https://github.com/<you>/athlete-monitor-pi.git
cd athlete-monitor-pi
bash scripts/setup_all.sh
```
Open Node-RED: `http://<pi-ip>:1880/` → **Import** `node-red/flows_athlete.json`. Dashboard at `/ui`.

## Python service
- File: `src/athlete_monitor.py`
- Virtualenv: `./.venv`
- Systemd: `athlete-monitor.service` (enabled by setup)

## MQTT payload
```json
{
  "temperature_c": 28.1,
  "heart_rate_bpm": 78.0,
  "spo2_percent": 97.3,
  "wifi_connected": true,
  "timestamp": 1755445596,
  "timestamp_ms": 1755445596224,
  "timestamp_iso_utc": "2025-08-17T15:46:36.224Z",
  "timestamp_iso": "2025-08-17T16:46:36+0100",
  "signal_quality": 82
}
```

## Multi-Wi-Fi (NetworkManager)
Add multiple SSIDs (priority decides which wins):
```bash
sudo nmcli dev wifi connect "HomeSSID" password "HomePass" ifname wlan0
sudo nmcli con modify "HomeSSID" connection.autoconnect yes connection.autoconnect-priority 100
```
Optional **fallback AP**: create a profile `Athlete-Setup` and enable `wifi-fallback.service` (scripts provided).

## Troubleshooting
- Sensor check: `python3 src/max30102_probe.py`
- Service logs: `sudo journalctl -u athlete-monitor -n 100 -f`
- Node-RED logs: `sudo journalctl -u nodered -n 100 -f`
- I²C detect: `i2cdetect -y 1` → look for `0x57`

## License
MIT
