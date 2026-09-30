import csv
import time
import threading
from collections import deque
from datetime import datetime

import serial

# =========================
# CONFIG (EDIT IF NEEDED)
# =========================
PORT_ACM0 = "COM6"   # MPU6050 + vib + rpm (50 Hz)
PORT_ACM1 = "COM7"   # current (50 Hz)

BAUD = 115200
OUT_CSV = "combined_50hz.csv"

TARGET_HZ = 50
PERIOD_S = 1.0 / TARGET_HZ

# If True, still write rows even before ACM0 first sample (fills blanks)
ALLOW_PARTIAL = False


# =========================
# READER THREADS
# =========================
class ACM0Reader(threading.Thread):
    """
    Reads ACM0:
      header: vib_adc,rpm,accel_x,...,temp_c
      data:   9 comma-separated fields
    """
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.stop_flag = threading.Event()
        self.lock = threading.Lock()
        self.latest = None  # (t_monotonic, [9 fields])
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(1.5)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def run(self):
        while not self.stop_flag.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.lower().startswith("vib_adc"):
                    continue

                parts = [p.strip() for p in line.split(",")]
                if len(parts) != 9:
                    continue

                vib = int(float(parts[0]))
                rpm = float(parts[1])
                ax  = float(parts[2])
                ay  = float(parts[3])
                az  = float(parts[4])
                gx  = float(parts[5])
                gy  = float(parts[6])
                gz  = float(parts[7])
                tc  = float(parts[8])

                t = time.monotonic()
                sample = [vib, rpm, ax, ay, az, gx, gy, gz, tc]

                with self.lock:
                    self.latest = (t, sample)

            except Exception:
                time.sleep(0.01)

    def get_latest(self):
        with self.lock:
            return self.latest

    def stop(self):
        self.stop_flag.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


class ACM1Reader(threading.Thread):
    """
    Reads ACM1: one float per line (current) @ ~50 Hz
    """
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.stop_flag = threading.Event()
        self.lock = threading.Lock()
        self.latest = None  # (t_monotonic, current_A)
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(1.5)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def run(self):
        while not self.stop_flag.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                val = float(line)
                t = time.monotonic()

                with self.lock:
                    self.latest = (t, val)

            except Exception:
                time.sleep(0.005)

    def get_latest(self):
        with self.lock:
            return self.latest

    def stop(self):
        self.stop_flag.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


# =========================
# MAIN LOGGER
# =========================
def main():
    print("Opening serial ports...")
    r0 = ACM0Reader(PORT_ACM0, BAUD)
    r1 = ACM1Reader(PORT_ACM1, BAUD)

    r0.open()
    r1.open()
    r0.start()
    r1.start()

    header = [
        "pc_time_iso",
        "current_A",
        "vib_adc", "rpm",
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z",
        "temp_c"
    ]

    last_acm0 = None      # 9 fields
    last_current = None   # float

    # Warm-up (optional)
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

    print(f"Logging to {OUT_CSV} at {TARGET_HZ} Hz. Stop with Stop button.\n")

    next_tick = time.monotonic()
    last_flush_monot = time.monotonic()

    try:
        with open(OUT_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)

            while True:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(min(0.002, next_tick - now))
                    continue

                next_tick += PERIOD_S

                # Update hold-last values
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

                # Flush about once per second
                if (time.monotonic() - last_flush_monot) >= 1.0:
                    f.flush()
                    last_flush_monot = time.monotonic()

    except KeyboardInterrupt:
        print("\nStopping...")

    r0.stop()
    r1.stop()
    print("Done.")


main()
