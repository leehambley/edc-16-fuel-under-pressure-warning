#!/usr/bin/env python3
"""
EDC16 M9R Trafic II - KWP2000 Diagnostic Server
Reads Trame A0 (RPM/speed), A1 (fuel pressure), A5 (fuelling), A6 (rail pressure/MPROP)
Serves live data as JSON via HTTP for the dashboard HTML frontend.

Usage:
  pip install pyserial
  python edc16_server.py --port COM3       # Windows
  python edc16_server.py --port /dev/ttyUSB0  # Linux

Then open edc16_dashboard.html in your browser.
"""

import argparse
import json
import logging
import serial
import struct
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("edc16")

# ─── KWP2000 frame definitions from EDC16_C3_VCD-X70 XML ─────────────────────
# Each trame: (send_bytes_hex, expected_min_bytes, {param_name: (first_byte, n_bits, signed, step, unit)})
# first_byte is 1-indexed per DDT convention; we use 0-indexed in decode (subtract 1)
# All multi-byte values are big-endian signed int16 unless noted

TRAMES = {
    "A0": {
        "cmd": bytes.fromhex("21A0"),
        "min_bytes": 16,
        "params": {
            "RPM":           (13, 16, True,  1.0,    "rpm"),
            "Vehicle speed": (15, 16, True,  0.01,   "km/h"),
        },
    },
    "A1": {
        "cmd": bytes.fromhex("21A1"),
        "min_bytes": 63,
        "params": {
            "Fuel temp":     (29, 16, True,  0.1,    "°C"),
            "Water temp":    (25, 16, True,  0.1,    "°C"),
            "Fuel pressure (LP)": (37, 16, True, 100.0, "hPa"),
            "Batt voltage":  (41, 16, True,  0.001,  "V"),
            "Accel pedal %": (58, 16, True,  0.01,   "%"),
        },
    },
    "A5": {
        "cmd": bytes.fromhex("21A5"),
        "min_bytes": 63,
        "params": {
            "Demanded qty":   (5,  16, True,  0.01, "mg/cyc"),
            "Actual qty":     (7,  16, True,  0.01, "mg/cyc"),
            "Engine qty":     (9,  16, True,  0.01, "mg/cyc"),
            "Max qty limit":  (13, 16, True,  0.01, "mg/cyc"),
            "Cyl1 qty":       (37, 16, True,  0.01, "mg/cyc"),
            "Cyl2 qty":       (39, 16, True,  0.01, "mg/cyc"),
            "Cyl3 qty":       (43, 16, True,  0.01, "mg/cyc"),
            "Cyl4 qty":       (45, 16, True,  0.01, "mg/cyc"),
        },
    },
    "A6": {
        "cmd": bytes.fromhex("21A6"),
        "min_bytes": 63,
        "params": {
            "Rail pressure actual":   (35, 16, True,  100.0, "hPa"),
            "Rail pressure setpoint": (37, 16, True,  100.0, "hPa"),
            "Rail sensor voltage":    (33, 16, True,  4.88758553, "mV"),
            "MPROP current setpoint": (39, 16, True,  1.0,  "mA"),
            "MPROP current actual":   (41, 16, True,  1.0,  "mA"),
            "MPROP duty cycle":       (43, 16, True,  0.01, "%"),
            "Engine torque actual":   (3,  16, True,  0.1,  "Nm"),
            "Torque internal":        (23, 16, True,  0.1,  "Nm"),
        },
    },
}

# Human-readable display names and grouping for the UI
PARAM_GROUPS = {
    "Rail pressure": ["Rail pressure actual", "Rail pressure setpoint"],
    "MPROP regulator": ["MPROP duty cycle", "MPROP current actual", "MPROP current setpoint"],
    "Fuelling": ["Demanded qty", "Actual qty", "Max qty limit"],
    "Per-cylinder qty": ["Cyl1 qty", "Cyl2 qty", "Cyl3 qty", "Cyl4 qty"],
    "Engine": ["RPM", "Vehicle speed", "Engine torque actual"],
    "Sensors": ["Batt voltage", "Water temp", "Fuel temp", "Fuel pressure (LP)", "Rail sensor voltage"],
}

def decode_value(data: bytes, first_byte_1idx: int, n_bits: int, signed: bool, step: float) -> float:
    """Decode a value from a KWP2000 response frame."""
    idx = first_byte_1idx - 1  # convert to 0-indexed
    if n_bits == 16:
        raw = struct.unpack_from(">h" if signed else ">H", data, idx)[0]
    elif n_bits == 8:
        raw = struct.unpack_from(">b" if signed else ">B", data, idx)[0]
    else:
        raw = 0
    return raw * step


class ELM327:
    """Minimal ELM327 driver for KWP2000 slow-init via K-line."""

    def __init__(self, port: str, baud: int = 38400):
        self.ser = serial.Serial(port, baud, timeout=2)
        self._lock = threading.Lock()

    def _send_raw(self, cmd: str) -> str:
        self.ser.write((cmd + "\r").encode())
        time.sleep(0.05)
        resp = b""
        deadline = time.time() + 2.0
        while time.time() < deadline:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            resp += chunk
            if b">" in resp:
                break
        return resp.decode(errors="replace").replace("\r", "\n").strip()

    def init_elm(self):
        """Reset and configure ELM327 for KWP2000 K-line."""
        log.info("Resetting ELM327...")
        self._send_raw("ATZ")
        time.sleep(1.5)
        for cmd in [
            "ATE0",       # echo off
            "ATL0",       # linefeed off
            "ATS0",       # spaces off
            "ATH0",       # headers off
            "ATSP5",      # ISO 14230-4 KWP2000 (slow init, 10.4kbaud)
            "ATKW0",      # don't show key words
            "ATIB10",     # inter-byte delay 10ms
            "ATAT1",      # adaptive timing
        ]:
            r = self._send_raw(cmd)
            log.debug(f"{cmd} -> {r!r}")
        log.info("ELM327 configured.")

    def kwp_init(self):
        """Trigger KWP2000 session init (slow-init 0x7A address for ECU)."""
        log.info("Starting KWP2000 session (address 7A)...")
        r = self._send_raw("ATSH7A")
        log.debug(f"ATSH7A -> {r!r}")
        # Send StartDiagnosticSession
        r = self._send_raw("1081")
        log.info(f"StartDiagnosticSession -> {r!r}")
        time.sleep(0.1)

    def send_kwp(self, payload_hex: str):
        """Send a KWP2000 payload and return response bytes (no header/checksum)."""
        with self._lock:
            resp_str = self._send_raw(payload_hex)
        resp_str = resp_str.strip().replace(" ", "").replace("\n", "")
        # Filter out prompts and non-hex
        lines = [l for l in resp_str.split(">") if l.strip()]
        hex_resp = ""
        for line in lines:
            clean = "".join(c for c in line if c in "0123456789ABCDEFabcdef")
            if len(clean) >= 4:
                hex_resp = clean
                break
        if not hex_resp:
            return None
        try:
            return bytes.fromhex(hex_resp)
        except ValueError:
            return None


class DiagSession:
    """Polls all trames and maintains a current-values dict."""

    def __init__(self, elm: ELM327):
        self.elm = elm
        self.data: dict = {}
        self.errors: list = []
        self.running = False
        self._thread = None
        self.last_update = 0.0

    def _poll_trame(self, name: str, trame: dict):
        cmd_hex = trame["cmd"].hex().upper()
        resp = self.elm.send_kwp(cmd_hex)
        if resp is None or len(resp) < trame["min_bytes"]:
            log.warning(f"Trame {name}: short/no response ({len(resp) if resp else 0} bytes)")
            return
        for param, (fb, nb, signed, step, unit) in trame["params"].items():
            try:
                val = decode_value(resp, fb, nb, signed, step)
                # Convert rail pressure hPa -> bar for readability
                if unit == "hPa" and "pressure" in param.lower():
                    val = val / 1000.0
                    unit = "bar"
                # Convert LP fuel pressure
                if param == "Fuel pressure (LP)":
                    val = val / 1000.0
                    unit = "bar"
                # Rail sensor voltage: mV -> V
                if unit == "mV" and "voltage" in param.lower():
                    val = val / 1000.0
                    unit = "V"
                self.data[param] = {"value": round(val, 3), "unit": unit}
            except Exception as e:
                log.debug(f"Decode error {param}: {e}")

    def poll_once(self):
        for name, trame in TRAMES.items():
            self._poll_trame(name, trame)
        self.last_update = time.time()

    def _loop(self):
        while self.running:
            try:
                self.poll_once()
            except Exception as e:
                log.error(f"Poll error: {e}")
            time.sleep(0.3)

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False


# ─── HTTP server ──────────────────────────────────────────────────────────────

session = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # suppress per-request logging

    def do_GET(self):
        if self.path == "/data":
            self._serve_data()
        elif self.path == "/groups":
            self._serve_groups()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_data(self):
        payload = {
            "ts": time.time(),
            "data": session.data if session else {},
        }
        body = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_groups(self):
        body = json.dumps(PARAM_GROUPS).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)


def main():
    global session

    parser = argparse.ArgumentParser(description="EDC16 M9R KWP2000 diagnostic server")
    parser.add_argument("--port", default="COM3", help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=38400, help="ELM327 serial baud rate")
    parser.add_argument("--http-port", type=int, default=8765, help="HTTP server port")
    parser.add_argument("--mock", action="store_true", help="Run with mock data (no hardware needed)")
    args = parser.parse_args()

    if args.mock:
        log.info("Running in MOCK mode - no hardware required")
        session = _MockSession()
        session.start()
    else:
        log.info(f"Connecting to ELM327 on {args.port} at {args.baud} baud...")
        elm = ELM327(args.port, args.baud)
        elm.init_elm()
        elm.kwp_init()
        session = DiagSession(elm)
        session.start()
        log.info("Polling started.")

    log.info(f"HTTP server on http://localhost:{args.http_port}")
    log.info("Open edc16_dashboard.html in your browser.")
    log.info("Press Ctrl+C to stop.")

    srv = HTTPServer(("localhost", args.http_port), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        log.info("Shutting down.")
    finally:
        if session:
            session.stop()


class _MockSession:
    """Simulates a degraded pump scenario for UI testing without hardware."""
    def __init__(self):
        self.data = {}
        self.last_update = 0.0
        self.running = False
        self._t = 0.0

    def start(self):
        self.running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def stop(self):
        self.running = False

    def _loop(self):
        import math
        while self.running:
            self._t += 0.3
            rpm = 800 + 2200 * abs(math.sin(self._t / 10))
            # Simulate pressure falling short above 2500 rpm
            pressure_limit = 1600 if rpm < 2500 else 1100 + 100 * math.sin(self._t)
            setpoint = min(1600, 800 + rpm * 0.35)
            actual = min(pressure_limit, setpoint - max(0, rpm - 2500) * 0.15)
            duty = min(99.9, 50 + (setpoint - actual) * 0.1)
            qty = 5 + 30 * abs(math.sin(self._t / 8))

            self.data = {
                "RPM":                      {"value": round(rpm), "unit": "rpm"},
                "Vehicle speed":            {"value": round(rpm * 0.04, 1), "unit": "km/h"},
                "Rail pressure actual":     {"value": round(actual / 10, 1), "unit": "bar"},
                "Rail pressure setpoint":   {"value": round(setpoint / 10, 1), "unit": "bar"},
                "MPROP duty cycle":         {"value": round(duty, 1), "unit": "%"},
                "MPROP current actual":     {"value": round(duty * 8), "unit": "mA"},
                "MPROP current setpoint":   {"value": round(setpoint * 0.5), "unit": "mA"},
                "Demanded qty":             {"value": round(qty, 1), "unit": "mg/cyc"},
                "Actual qty":               {"value": round(qty * 0.95, 1), "unit": "mg/cyc"},
                "Max qty limit":            {"value": 100.0, "unit": "mg/cyc"},
                "Cyl1 qty":                 {"value": round(qty * 0.24, 2), "unit": "mg/cyc"},
                "Cyl2 qty":                 {"value": round(qty * 0.25, 2), "unit": "mg/cyc"},
                "Cyl3 qty":                 {"value": round(qty * 0.26, 2), "unit": "mg/cyc"},
                "Cyl4 qty":                 {"value": round(qty * 0.25, 2), "unit": "mg/cyc"},
                "Engine torque actual":     {"value": round(qty * 5.2, 1), "unit": "Nm"},
                "Torque internal":          {"value": round(qty * 5.5, 1), "unit": "Nm"},
                "Water temp":               {"value": 87.0, "unit": "°C"},
                "Fuel temp":                {"value": 42.0, "unit": "°C"},
                "Batt voltage":             {"value": 14.2, "unit": "V"},
                "Fuel pressure (LP)":       {"value": round(4.2 + 0.3 * math.sin(self._t), 2), "unit": "bar"},
                "Rail sensor voltage":      {"value": round(1.17 + actual / 50000, 3), "unit": "V"},
                "Accel pedal %":            {"value": round(50 * abs(math.sin(self._t / 8)), 1), "unit": "%"},
            }
            self.last_update = time.time()
            time.sleep(0.3)


if __name__ == "__main__":
    main()
