# -*- coding: utf-8 -*-


import time
import json
import socket
import threading
from collections import deque

from gpiozero import LED
from w1thermsensor import W1ThermSensor, W1ThermSensorError
from paho.mqtt import client as mqtt
from spo2calc import estimate_spo2  # local module

# ---------- CONFIG ----------
MQTT_BROKER = "localhost"
MQTT_PORT   = 1883
MQTT_TOPIC  = "athlete/data"
MQTT_CLIENT_ID = "athlete-pi"

WIFI_LED_PIN = 27  # simple LED; if inverted, swap wiring

SAMPLE_RATE_HZ   = 100
WINDOW_SEC       = 4.0
READ_CHUNK_SEC   = 0.25
PUBLISH_PERIOD_S = 0.5

QUALITY_MIN_SPO2 = 75
QUALITY_MIN_HR   = 70
FINGER_MIN_AC    = 90.0
FINGER_MIN_PEAKS = 2
STALE_TIMEOUT_S  = 15.0

MAX_HR_STEP   = 20.0
MAX_SPO2_STEP = 2.0
TEMP_PERIOD_S = 2.0

WINDOW_N     = int(SAMPLE_RATE_HZ * WINDOW_SEC)
READ_CHUNK_N = int(SAMPLE_RATE_HZ * READ_CHUNK_SEC)

# ---------- Wi-Fi LED ----------
wifi_led = LED(WIFI_LED_PIN)

def wifi_connected(host="8.8.8.8", port=53, timeout=2.0) -> bool:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.close()
        return True
    except Exception:
        return False

def wifi_led_task():
    while True:
        try:
            wifi_led.on() if wifi_connected() else wifi_led.off()
        except Exception:
            wifi_led.off()
        time.sleep(2)

# ---------- Temperature ----------
try:
    temp_sensor = W1ThermSensor()
except Exception:
    temp_sensor = None

_last_temp_t = 0.0
_temp_cache  = None

def read_temperature_c():
    global _last_temp_t, _temp_cache
    if not temp_sensor:
        return None
    now = time.time()
    if (now - _last_temp_t) < TEMP_PERIOD_S and _temp_cache is not None:
        return _temp_cache
    try:
        _temp_cache = round(temp_sensor.get_temperature(), 2)
        _last_temp_t = now
        return _temp_cache
    except (W1ThermSensorError, Exception):
        return _temp_cache

# ---------- MAX30102 ----------
sensor = None
have_max = False

def init_max30102():
    global sensor, have_max
    try:
        import max30102 as max30102_local
        sensor = max30102_local.MAX30102(gpio_pin=None)  # polling mode (no INT)
        have_max = True
        return True
    except Exception as e:
        print("MAX30102 init failed:", e)
        sensor = None
        have_max = False
        return False

def estimate_hr_bpm(ir_samples, fs=SAMPLE_RATE_HZ):
    n = len(ir_samples)
    if n < int(fs * 3):
        return None
    win = max(1, int(0.8 * fs))  # 0.8 s MA DC removal
    acc = 0.0
    ma = []
    for i, v in enumerate(ir_samples):
        acc += v
        if i >= win:
            acc -= ir_samples[i - win]
        ma.append(acc / float(min(i + 1, win)))
    ac = [ir_samples[i] - ma[i] for i in range(n)]
    rng = max(ac) - min(ac)
    thr = max(500, int(0.15 * rng))
    refractory = int(0.30 * fs)
    peaks, last = 0, -refractory
    for i in range(1, n - 1):
        if ac[i] > thr and ac[i] > ac[i - 1] and ac[i] >= ac[i + 1] and (i - last) > refractory:
            peaks += 1
            last = i
    if peaks < 2:
        return None
    hr = (peaks / (n / float(fs))) * 60.0
    return round(hr, 1) if 35 <= hr <= 200 else None

# Ring buffers + concurrency
red_buf = deque(maxlen=WINDOW_N)
ir_buf  = deque(maxlen=WINDOW_N)
buffers_lock = threading.Lock()
warmed_up = False
stop_evt = threading.Event()

def sampler_thread():
    global warmed_up
    while not stop_evt.is_set() and sensor is not None:
        try:
            red, ir = sensor.read_sequential(READ_CHUNK_N)  # ~READ_CHUNK_SEC
            if red and ir:
                with buffers_lock:
                    red_buf.extend(red)
                    ir_buf.extend(ir)
                    if len(red_buf) == WINDOW_N and len(ir_buf) == WINDOW_N:
                        warmed_up = True
        except Exception:
            time.sleep(0.05)

def compute_from_window():
    if not warmed_up:
        return None, None, {}
    with buffers_lock:
        red = list(red_buf)
        ir  = list(ir_buf)
    spo2, diag = estimate_spo2(red, ir, fs=SAMPLE_RATE_HZ)
    hr = estimate_hr_bpm(ir, fs=SAMPLE_RATE_HZ)
    return hr, spo2, diag

# ---------- Smoothing ----------
def ema(prev, new, alpha):
    if new is None:
        return prev
    return new if prev is None else (1 - alpha) * prev + alpha * new

def step_clamp(prev, curr, max_step):
    if prev is None or curr is None:
        return curr
    delta = curr - prev
    if abs(delta) > max_step:
        return prev + max_step * (1 if delta > 0 else -1)
    return curr

# ---------- MQTT ----------
client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id=MQTT_CLIENT_ID,
    protocol=mqtt.MQTTv311
)

def on_connect(client, userdata, flags, reason_code, properties=None):
    print("MQTT connected" if reason_code == 0 else f"MQTT connect failed: {reason_code}")

client.on_connect = on_connect
client.loop_start()

def mqtt_connect():
    while True:
        try:
            client.connect(MQTT_BROKER, MQTT_PORT, keepalive=30)
            return
        except Exception:
            time.sleep(2)

# ---------- MAIN ----------
def main():
    threading.Thread(target=wifi_led_task, daemon=True).start()

    if not init_max30102():
        print("Proceeding without MAX30102; HR/SpO2 will be None.")
    if have_max and sensor is not None:
        threading.Thread(target=sampler_thread, daemon=True).start()

    mqtt_connect()

    hr_smoothed = None
    spo2_smoothed = None
    last_valid_ts = 0.0
    finger_was_present = False

    while True:
        t0 = time.monotonic()

        hr = spo2 = None
        q = 0
        ac_ir = 0.0
        peaks = 0
        diag = {}

        if have_max and warmed_up:
            try:
                hr, spo2, diag = compute_from_window()
                q = diag.get("quality_score", 0)
                ac_ir = diag.get("ac_rms_ir", 0.0)
                peaks = diag.get("peaks", 0)

                finger_present = (ac_ir >= FINGER_MIN_AC) and (peaks >= FINGER_MIN_PEAKS) and (q >= 50)

                if q < QUALITY_MIN_HR:
                    hr = None
                if q < QUALITY_MIN_SPO2:
                    spo2 = None

                if finger_was_present and not finger_present:
                    hr = spo2 = None
                    hr_smoothed = spo2_smoothed = None
                    try:
                        sensor.flush_fifo()
                    except Exception:
                        pass
                finger_was_present = finger_present

            except Exception:
                pass

        temp_c = read_temperature_c()

        now = time.time()
        if (hr is not None) or (spo2 is not None):
            last_valid_ts = now
        if (now - last_valid_ts) > STALE_TIMEOUT_S:
            hr = spo2 = None
            hr_smoothed = spo2_smoothed = None

        prev_hr, prev_spo2 = hr_smoothed, spo2_smoothed
        hr_smoothed   = ema(hr_smoothed,   hr,   0.30)
        spo2_smoothed = ema(spo2_smoothed, spo2, 0.20)
        hr_out   = step_clamp(prev_hr,   hr_smoothed,   MAX_HR_STEP)   if hr_smoothed   is not None else None
        spo2_out = step_clamp(prev_spo2, spo2_smoothed, MAX_SPO2_STEP) if spo2_smoothed is not None else None

        ts_ms = int(now * 1000)
        payload = {
            "temperature_c": temp_c,
            "heart_rate_bpm": None if hr_out   is None else round(hr_out,   1),
            "spo2_percent":  None if spo2_out is None else round(spo2_out, 1),
            "wifi_connected": wifi_connected(),
            "timestamp": int(now),
            "timestamp_ms": ts_ms,
            "timestamp_iso_utc": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(now)) + f".{ts_ms%1000:03d}Z",
            "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(now)),
            "signal_quality": q,
        }

        try:
            client.publish(MQTT_TOPIC, json.dumps(payload), qos=0, retain=False)
            print("Published:", payload)
        except Exception as e:
            print("MQTT publish error:", e)
            mqtt_connect()

        dt = time.monotonic() - t0
        if (PUBLISH_PERIOD_S - dt) > 0:
            time.sleep(PUBLISH_PERIOD_S - dt)

if __name__ == "__main__":
    try:
        main()
    finally:
        stop_evt.set()
