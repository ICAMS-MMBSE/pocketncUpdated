import csv
import time
import threading
from datetime import datetime
import os
import sys

import serial
import pyaudio
import wave

# =========================
# CONFIG (EDIT IF NEEDED)
# =========================
PORT_ACM0 = "COM6"   # MPU6050 + vib + rpm (50 Hz)
PORT_ACM1 = "COM7"   # current (50 Hz)
BAUD = 115200

TARGET_HZ = 50
PERIOD_S = 1.0 / TARGET_HZ
ALLOW_PARTIAL = False

RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 6
RESPEAKER_WIDTH = 2
CHUNK = 1024
DEVICE_NAME_KEYWORD = "ReSpeaker"

# Shared base name for both output files (same timestamp)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_NAME = datetime.now().strftime("%Y%m%d_%H%M%S")
OUT_CSV   = os.path.join(SCRIPT_DIR, f"run_{BASE_NAME}.csv")
OUT_WAV   = os.path.join(SCRIPT_DIR, f"run_{BASE_NAME}.wav")


# =========================
# STOP FLAG (shared)
# =========================
stop_event = threading.Event()


# =========================
# SERIAL READER THREADS
# =========================
class ACM0Reader(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.latest = None
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(1.5)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def run(self):
        while not stop_event.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line or line.lower().startswith("vib_adc"):
                    continue
                parts = [p.strip() for p in line.split(",")]
                if len(parts) != 9:
                    continue
                sample = [
                    int(float(parts[0])),
                    float(parts[1]),
                    float(parts[2]), float(parts[3]), float(parts[4]),
                    float(parts[5]), float(parts[6]), float(parts[7]),
                    float(parts[8]),
                ]
                with self.lock:
                    self.latest = (time.monotonic(), sample)
            except Exception:
                time.sleep(0.01)

    def get_latest(self):
        with self.lock:
            return self.latest

    def stop(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


class ACM1Reader(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.latest = None
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(1.5)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def run(self):
        while not stop_event.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                val = float(line)
                with self.lock:
                    self.latest = (time.monotonic(), val)
            except Exception:
                time.sleep(0.005)

    def get_latest(self):
        with self.lock:
            return self.latest

    def stop(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


# =========================
# AUDIO RECORDER THREAD
# =========================
class AudioRecorder(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.error = None

    def _find_device(self, pa):
        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            if (int(info.get("maxInputChannels", 0)) >= RESPEAKER_CHANNELS and
                    DEVICE_NAME_KEYWORD.lower() in info.get("name", "").lower()):
                return i, info
        raise RuntimeError(f"Could not find '{DEVICE_NAME_KEYWORD}' device with {RESPEAKER_CHANNELS} channels.")

    def run(self):
        p = pyaudio.PyAudio()
        stream = None
        wf = None
        try:
            device_index, device_info = self._find_device(p)
            print(f"[Audio] Using device {device_index}: {device_info['name']}")

            audio_format = p.get_format_from_width(RESPEAKER_WIDTH)
            wf = wave.open(OUT_WAV, "wb")
            wf.setnchannels(RESPEAKER_CHANNELS)
            wf.setsampwidth(p.get_sample_size(audio_format))
            wf.setframerate(RESPEAKER_RATE)

            stream = p.open(
                rate=RESPEAKER_RATE,
                format=audio_format,
                channels=RESPEAKER_CHANNELS,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=CHUNK,
            )
            print("[Audio] Recording started.")

            chunks_recorded = 0
            last_report = time.time()
            while not stop_event.is_set():
                data = stream.read(CHUNK, exception_on_overflow=False)
                wf.writeframes(data)
                chunks_recorded += 1
                if time.time() - last_report >= 60:
                    seconds_done = chunks_recorded * CHUNK / RESPEAKER_RATE
                    print(f"[Audio] Recorded ~{seconds_done:.1f} s so far...")
                    last_report = time.time()

        except Exception as e:
            self.error = e
            print(f"[Audio] Error: {e}", file=sys.stderr)
        finally:
            try:
                if stream:
                    stream.stop_stream()
                    stream.close()
            except Exception:
                pass
            try:
                if wf:
                    wf.close()
                    print(f"[Audio] Saved: {OUT_WAV}")
            except Exception as e:
                print(f"[Audio] Error saving WAV: {e}", file=sys.stderr)
            p.terminate()


# =========================
# MAIN
# =========================
def main():
    print("Opening serial ports...")
    r0 = ACM0Reader(PORT_ACM0, BAUD)
    r1 = ACM1Reader(PORT_ACM1, BAUD)
    r0.open()
    r1.open()
    r0.start()
    r1.start()

    audio = AudioRecorder()
    audio.start()

    header = [
        "pc_time_iso",
        "current_A",
        "vib_adc", "rpm",
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z",
        "temp_c"
    ]

    last_acm0 = None
    last_current = None

    # Warm-up
    t0 = time.time()
    while time.time() - t0 < 3.0:
        a0 = r0.get_latest()
        a1 = r1.get_latest()
        if a0 is not None:
            last_acm0 = a0[1]
        if a1 is not None:
            last_current = a1[1]
        if last_current is not None and (last_acm0 is not None or ALLOW_PARTIAL):
            break
        time.sleep(0.05)

    print(f"Logging to {OUT_CSV} at {TARGET_HZ} Hz. Press Ctrl+C to stop.\n")

    next_tick = time.monotonic()
    last_flush_monot = time.monotonic()

    try:
        with open(OUT_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)

            while not stop_event.is_set():
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(min(0.002, next_tick - now))
                    continue

                next_tick += PERIOD_S

                a0 = r0.get_latest()
                if a0 is not None:
                    last_acm0 = a0[1]

                a1 = r1.get_latest()
                if a1 is not None:
                    last_current = a1[1]

                if last_current is None and not ALLOW_PARTIAL:
                    continue

                pc_time_iso = datetime.now().astimezone().isoformat(timespec="milliseconds")
                acm0_fields = last_acm0 if last_acm0 is not None else ([""] * 9 if ALLOW_PARTIAL else None)
                if acm0_fields is None:
                    continue

                row = [pc_time_iso, last_current if last_current is not None else ""] + acm0_fields
                w.writerow(row)

                if (time.monotonic() - last_flush_monot) >= 1.0:
                    f.flush()
                    last_flush_monot = time.monotonic()

    except KeyboardInterrupt:
        print("\nCtrl+C received. Stopping...")

    finally:
        stop_event.set()
        r0.stop()
        r1.stop()
        audio.join(timeout=5)
        print("Done.")


main()