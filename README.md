# PocketNC Sensor & Data Acquisition

Firmware and softwarre fo monitoring a **PocketNC** CNC machine. Collects vibration, IMU, RPM, motor current, audio, video, and machine state data.

---

## Repository Structure

```
PocketNC/
├── mega_sensor_fw/          # Arduino Mega firmware
├── pc_data_code/            # PC-side data logger (serial + audio)
└── pi_camera_code/
    ├── videoStream/         # Live camera stream (runs on Pi host, see Setup)
    └── PocketNc/webSoc/     # PocketNC WebSocket → MQTT bridge
```

---

## Components

### Arduino Mega (`mega_sensor_fw`)
Reads all sensors and can publish to MQTT via an **Ethernet shield** at 50 Hz. Live sensor CSV output and MQTT publishing can be enabled or disabled at runtime over serial.

**Sensors:**
- Vibration — analog (A0)
- RPM — IR sensor (pin 2, interrupt-driven)
- IMU — MPU6050 (accel XYZ, gyro XYZ, temp)
- Motor current — SCT-013 30A/1V clamp (A1)

**MQTT topic:** `pocketnc/sensors`
**Serial output:** `vib_adc,rpm,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,current_A`

### Runtime serial commands

Open the serial monitor at **115200 baud**, select a line ending, and send one command per line. The settings apply immediately and reset to their defaults after a power cycle or reset.

| Command | Effect |
| --- | --- |
| `help` | Show the available commands. |
| `status` | Show current runtime settings and calibration-buffer progress. |
| `debug on` / `debug off` | Select raw-current output / calibrated-current output. This affects both CSV and MQTT readings. |
| `serial on` / `serial all` | Start the full live 50 Hz CSV sensor stream. |
| `serial single <reading>` | Display only one selected reading at 50 Hz, one numeric value per line. For example, `serial single rpm`. Valid readings: `vib_adc`, `rpm`, `accel_x`, `accel_y`, `accel_z`, `gyro_x`, `gyro_y`, `gyro_z`, `temp_c`, and `current_a`. |
| `serial off` | Stop live sensor output. Command replies still print. |
| `mqtt on` / `mqtt off` | Start or stop MQTT publishing. Turning it off disconnects the MQTT client; turning it on reconnects without blocking sensor sampling. |
| `calibrate <actual_amps>` | Uses the median of the most recent 100 raw-current samples to print a suggested calibration value. Use `calibrate 0` with the machine off to obtain a suggested `I_ZERO_RAW`; otherwise it prints a `CAL_TABLE` row for the supplied measured current. |
| `machinecal` (or `accelcal machine`) | With the machine homed and motionless, capture 100 samples as a machine reference. It sets X/Y/Z acceleration and gyro X/Y/Z to zero at that pose. Gravity is included in the Z home reference so all measurements are relative to machine home. Repeat whenever you re-home the machine. |
| `accelcal +x`, `accelcal -x`, etc. | Capture 100 motionless samples with the specified MPU6050 axis pointing straight up. Capture all six faces: `+x`, `-x`, `+y`, `-y`, `+z`, and `-z`. The final capture applies per-axis offset and scale corrections immediately. |
| `accelcal status` / `accelcal reset` | Show six-face calibration progress / discard captured faces and restore the default accelerometer correction. |

Command and calibration replies begin with `#`, so they are distinguishable from CSV sensor records. Calibration suggestions are intentionally not written to the sketch automatically: review the result, update `I_ZERO_RAW` or `CAL_TABLE` in `mega_sensor_fw.ino`, then upload it.

Accelerometer calibration prints `ACCEL_OFFSET_DEFAULT` and `ACCEL_SCALE_DEFAULT` after the sixth face, and machine calibration prints `GYRO_OFFSET_DEFAULT`. Copy those values into the sketch to retain the sensor corrections after reset or power loss. The machine-home acceleration reference itself is intentionally runtime-only, so run `machinecal` after each home.

### PC Data Logger (`pc_data_code`)
Logs sensor data from serial + records audio from a ReSpeaker 6-channel USB mic. Run `combined.py` to capture both simultaneously.

### Pi Camera Stream (`pi_camera_code/videoStream`)
Flask app that serves a live camera feed at `http://<pi-ip>:8000/pocketNc/stream`. Runs directly on
the Pi host — not Dockerized. On a Raspberry Pi 5 / CM5, camera access through libcamera
(`rpicam-vid`) and device passthrough into a container both proved unreliable in practice for a USB
webcam, so this just runs as a plain venv'd Python process with OpenCV talking straight to the V4L2
device node. `docker-compose.yaml`/`dockerfile` in that folder are left over from an earlier attempt
and are currently unused.

### Pi WebSocket Bridge (`pi_camera_code/PocketNc/webSoc`)
Connects to the PocketNC machine's WebSocket API and publishes machine state (position, spindle, estop, feedrate, etc.) to MQTT.

---

## Setup

### Arduino
1. Install libraries via Arduino Library Manager:
   - `Adafruit MPU6050`
   - `Ethernet`
   - `PubSubClient`
2. Set your MQTT broker IP in `mega_sensor_fw.ino` (`mqtt_server`)
3. Upload to an **Arduino Mega** with an Ethernet shield attached

### PC Data Logger
```bash
pip install pyserial pyaudio
python pc_data_code/combined.py
```
Edit `COM6` port in `combined.py` to match your system. Stop with `Ctrl+C`.

### Pi — Camera Stream
Runs directly on the Pi (not Docker — see note above).

```bash
cd pi_camera_code/videoStream/code
sudo apt-get install -y python3-venv v4l-utils
python3 -m venv venv
source venv/bin/activate
pip install flask opencv-python numpy
```

**Find the camera's real V4L2 capture node.** USB webcams aren't always `/dev/video0` — that index
is often already taken by the board's onboard CSI/ISP nodes:
```bash
v4l2-ctl --list-devices
for d in /dev/video*; do echo "== $d =="; v4l2-ctl -d "$d" --all 2>/dev/null | grep -A3 "Device Caps"; done
```
Find the node that reports `Video Capture` in its capability list, and set that path (as a string,
e.g. `"/dev/video19"`) in `app.py`'s `cv2.VideoCapture(...)` line.

If opening the device fails with a permission error, add your user to the `video` group and start a
new session:
```bash
sudo usermod -aG video $USER
newgrp video
```

**Run:**
```bash
python app.py
```

View from another machine on the same network at `http://<pi-ip>:8000/pocketNc/stream`.

### Pi — WebSocket Bridge
```bash
cd pi_camera_code/PocketNc/webSoc
python3 -m venv venv
source venv/bin/activate
pip install websockets paho-mqtt

python pnc_wSock.py
```
Edit `broker` and `url` at the top of `pnc_wSock.py` for your network.
