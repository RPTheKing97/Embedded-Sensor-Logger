#!/usr/bin/env python3
"""
Live serial dashboard for the STM32 Embedded Sensor Logger.

Usage examples:
  python dashboard/serial_dashboard.py --port COM5
  python dashboard/serial_dashboard.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import collections
import re
import threading
import time
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial


FIELD_RE = re.compile(r"([A-Za-z0-9_]+):([^,]+)")


def parse_telemetry(line: str) -> Optional[Dict[str, float]]:
    """Parse one telemetry line into engineering units."""
    fields: Dict[str, float] = {}
    for key, value in FIELD_RE.findall(line.strip()):
        try:
            fields[key] = float(value)
        except ValueError:
            return None

    if "TS" not in fields or "V" not in fields:
        return None

    # Convert integer-scaled firmware units into human units.
    converted = {
        "t_s": fields["TS"] / 1000.0,
        "valid": int(fields.get("V", 0)),
        "ax_g": fields.get("AX_mg", 0.0) / 1000.0,
        "ay_g": fields.get("AY_mg", 0.0) / 1000.0,
        "az_g": fields.get("AZ_mg", 0.0) / 1000.0,
        "gx_dps": fields.get("GX_cdps", 0.0) / 100.0,
        "gy_dps": fields.get("GY_cdps", 0.0) / 100.0,
        "gz_dps": fields.get("GZ_cdps", 0.0) / 100.0,
        "temp_c": fields.get("T_cC", 0.0) / 100.0,
        "pressure_hpa": fields.get("P_chPa", 0.0) / 100.0,
        "errors": fields.get("ERR", 0.0),
        "retries": fields.get("RET", 0.0),
        "samples": fields.get("S", 0.0),
    }
    return converted


@dataclass
class TelemetryBuffer:
    maxlen: int = 600

    def __post_init__(self) -> None:
        self.lock = threading.Lock()
        self.rows: Deque[Dict[str, float]] = collections.deque(maxlen=self.maxlen)
        self.latest_line: str = ""
        self.last_error: str = ""

    def append(self, row: Dict[str, float], raw_line: str) -> None:
        with self.lock:
            self.rows.append(row)
            self.latest_line = raw_line

    def snapshot(self) -> List[Dict[str, float]]:
        with self.lock:
            return list(self.rows)

    def set_error(self, message: str) -> None:
        with self.lock:
            self.last_error = message

    def status(self) -> tuple[str, str]:
        with self.lock:
            return self.latest_line, self.last_error


def serial_reader(port: str, baud: int, buf: TelemetryBuffer, stop: threading.Event) -> None:
    try:
        with serial.Serial(port, baudrate=baud, timeout=1) as ser:
            ser.reset_input_buffer()
            while not stop.is_set():
                raw = ser.readline().decode("utf-8", errors="replace").strip()
                if not raw:
                    continue
                parsed = parse_telemetry(raw)
                if parsed is not None:
                    buf.append(parsed, raw)
    except serial.SerialException as exc:
        buf.set_error(f"Serial error: {exc}")


def series(rows: List[Dict[str, float]], key: str) -> List[float]:
    return [r[key] for r in rows]


def main() -> None:
    parser = argparse.ArgumentParser(description="Live plotter for STM32 sensor logger telemetry.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--window", type=int, default=600, help="Number of samples to keep on screen")
    args = parser.parse_args()

    buf = TelemetryBuffer(maxlen=args.window)
    stop = threading.Event()

    thread = threading.Thread(target=serial_reader, args=(args.port, args.baud, buf, stop), daemon=True)
    thread.start()

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 8))
    fig.suptitle("STM32 Sensor Logger Live Dashboard")

    ax_accel, ax_gyro, ax_env, ax_status = axes

    accel_lines = {
        "AX": ax_accel.plot([], [], label="AX (g)")[0],
        "AY": ax_accel.plot([], [], label="AY (g)")[0],
        "AZ": ax_accel.plot([], [], label="AZ (g)")[0],
    }
    gyro_lines = {
        "GX": ax_gyro.plot([], [], label="GX (dps)")[0],
        "GY": ax_gyro.plot([], [], label="GY (dps)")[0],
        "GZ": ax_gyro.plot([], [], label="GZ (dps)")[0],
    }
    temp_line = ax_env.plot([], [], label="Temp (C)")[0]
    pressure_line = ax_env.plot([], [], label="Pressure (hPa)")[0]
    err_line = ax_status.plot([], [], label="Errors")[0]
    ret_line = ax_status.plot([], [], label="Retries")[0]

    for axis in axes:
        axis.grid(True)
        axis.legend(loc="upper left")

    ax_status.set_xlabel("Time (s)")
    ax_accel.set_ylabel("Accel")
    ax_gyro.set_ylabel("Gyro")
    ax_env.set_ylabel("Env")
    ax_status.set_ylabel("Faults")

    status_text = fig.text(0.01, 0.01, "", fontsize=9)

    def update(_frame: int):
        rows = buf.snapshot()
        if not rows:
            latest, err = buf.status()
            status_text.set_text(err or f"Waiting for telemetry on {args.port}...")
            return []

        t0 = rows[0]["t_s"]
        t = [r["t_s"] - t0 for r in rows]

        accel_lines["AX"].set_data(t, series(rows, "ax_g"))
        accel_lines["AY"].set_data(t, series(rows, "ay_g"))
        accel_lines["AZ"].set_data(t, series(rows, "az_g"))

        gyro_lines["GX"].set_data(t, series(rows, "gx_dps"))
        gyro_lines["GY"].set_data(t, series(rows, "gy_dps"))
        gyro_lines["GZ"].set_data(t, series(rows, "gz_dps"))

        temp_line.set_data(t, series(rows, "temp_c"))
        pressure_line.set_data(t, series(rows, "pressure_hpa"))

        err_line.set_data(t, series(rows, "errors"))
        ret_line.set_data(t, series(rows, "retries"))

        xmin = max(0.0, t[-1] - 30.0)
        xmax = max(10.0, t[-1])
        for axis in axes:
            axis.set_xlim(xmin, xmax)
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

        latest, err = buf.status()
        valid = int(rows[-1]["valid"])
        sensor_state = []
        sensor_state.append("MPU6050 OK" if (valid & 0x01) else "MPU6050 FAULT")
        sensor_state.append("BMP280 OK" if (valid & 0x02) else "BMP280 FAULT")
        status_text.set_text(" | ".join(sensor_state) + f" | Latest: {latest}" + (f" | {err}" if err else ""))

        return list(accel_lines.values()) + list(gyro_lines.values()) + [temp_line, pressure_line, err_line, ret_line]

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)

    try:
        plt.show()
    finally:
        stop.set()
        thread.join(timeout=2)


if __name__ == "__main__":
    main()
