#!/usr/bin/env python3
"""
EDC16 M9R Trafic II - KWP2000 Diagnostic Server
Reads Trame A0 (RPM/speed), A1 (fuel pressure), A5 (fuelling), A6 (rail pressure/MPROP)
Serves live data as JSON via HTTP for the dashboard HTML frontend, and logs
every trip (ignition-on period) to its own CSV file under --log-dir.

Ignition is assumed OFF once the K-line stops answering for a few consecutive
poll cycles (IGNITION_OFF_CYCLES) — at that point the current trip log is
closed, and a new one is opened automatically next time data starts flowing.

Usage (with uv):
  uv run edc16_server.py --port COM3          # Windows
  uv run edc16_server.py --port /dev/ttyUSB0  # Linux
  uv run edc16_server.py --mock               # no hardware needed

Then open edc16_dashboard.html in your browser.
"""

import argparse
import csv
import json
import logging
import serial
import struct
import time
import threading
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path

from edc16_trames import TRAMES

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("edc16")

# ─── KWP2000 frame definitions ────────────────────────────────────────────────
# TRAMES (imported from edc16_trames.py) is generated straight from Renault's
# own DDT4ALL ECU definition — see tools/gen_trames.py. Covers trames A0-A8,
# 263 named analog signals total, in their original French names.

# Flat, stable list of every param name across all trames — used as the CSV header
# so column order never changes between trips even if a poll misses some trames.
# Every trame also gets a "<name>_raw" column holding the full hex response, so
# any PID we haven't decoded by name yet is still captured verbatim.
ALL_PARAMS = []
for _name, _trame in TRAMES.items():
    ALL_PARAMS.extend(_trame["params"].keys())
    ALL_PARAMS.append(f"{_name}_raw")
ALL_PARAMS.append("Active DTCs")

# How many consecutive fully-empty poll cycles (all trames unanswered) before we
# decide the K-line has gone quiet because the ignition was switched off.
IGNITION_OFF_CYCLES = 5

# Curated groups for the most driving-relevant signals; everything else in
# ALL_PARAMS (263 signals total, plus raw hex per trame) still gets logged to
# CSV and shown in the dashboard's "Other" catch-all — nothing is hidden.
PARAM_GROUPS = {
    "Rail pressure": ["Pression rail actuelle", "Consigne de pression rail", "Tension capteur pression rail"],
    "MPROP regulator": ["RCO MPROP", "Courant de la MPROP mesuré", "Consigne de courant de la MPROP"],
    "Fuelling": ["Débit injecté", "Débit désiré avant limitation système et sans LiGov", "Débit maximal", "Débit moteur (avec ASD)"],
    "Per-injection qty": ["Débit poste à poste 1", "Débit poste à poste 2", "Débit poste à poste 3", "Débit poste à poste 4"],
    "Engine": ["Régime moteur", "Vitesse véhicule", "Couple Moteur effectif", "Ratio pédale accélérateur"],
    "Turbo / EGR": ["PAVT", "Consigne pression suralimentation", "RCO turbo", "RCO vanne EGR", "Ecart pression turbo"],
    "DPF (FàP)": [
        "PFlt_mSot - Masse de suie dans le FàP",
        "PFlt_pDiff - Pression différentielle dans le FàP",
        "Pression avant FàP",
        "PFltCD_tPre - Température avant FàP",
        "PFltCD_tPst - Température après FàP",
    ],
    "Sensors": ["Tension batterie", "Température eau", "Température carburant", "Température air", "Température huile", "Pression carburant"],
}
_grouped = {p for names in PARAM_GROUPS.values() for p in names}
PARAM_GROUPS["Other"] = [p for p in ALL_PARAMS if p not in _grouped and not p.endswith("_raw")]


def decode_value(data: bytes, first_byte_1idx: int, n_bits: int, signed: bool, scale: float, offset: float = 0.0) -> float:
    """Decode a value from a KWP2000 response frame.

    n_bits is 16 for a normal word, or 31 for a "large" counter (odometer,
    engine-run-time, ...) that DDT4ALL stores as a full 4-byte big-endian word.
    """
    idx = first_byte_1idx - 1  # convert to 0-indexed
    if n_bits == 16:
        raw = struct.unpack_from(">h" if signed else ">H", data, idx)[0]
    elif n_bits == 31:
        raw = struct.unpack_from(">i" if signed else ">I", data, idx)[0]
    elif n_bits == 8:
        raw = struct.unpack_from(">b" if signed else ">B", data, idx)[0]
    else:
        raw = 0
    return raw * scale + offset


class TripLogger:
    """Writes one CSV file per trip (ignition-on period) into log_dir.

    A new file is opened the moment data starts flowing again after a gap,
    and closed as soon as the ignition-off condition is detected, so each
    file corresponds to one drive rather than one continuous never-ending log.
    """

    def __init__(self, log_dir: str):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._file = None
        self._writer = None
        self.path = None

    @property
    def active(self) -> bool:
        return self._file is not None

    def start_trip(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self.log_dir / f"trip_{ts}.csv"
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["timestamp"] + ALL_PARAMS)
        log.info(f"Ignition ON — logging trip to {self.path}")

    def log_row(self, data: dict):
        if not self._writer:
            return
        row = [datetime.now().isoformat(timespec="milliseconds")]
        for name in ALL_PARAMS:
            entry = data.get(name)
            row.append(entry["value"] if entry else "")
        self._writer.writerow(row)
        self._file.flush()

    def end_trip(self):
        if self._file:
            log.info(f"Ignition OFF — closed trip log {self.path}")
            self._file.close()
        self._file = None
        self._writer = None
        self.path = None


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
        """Address the EDC16 ECU (KWP2000 fast-init, target 0x7A) for diagnostic requests.

        ATSH needs the full 3-byte header (format=0x81, target=0x7A, source=0xF1);
        a bare "ATSH7A" is invalid ELM327 syntax (replies "?") and silently leaves
        the header unset, so every request then goes to the wrong/default address
        and comes back as a KWP negative response (7F ...). No explicit
        StartDiagnosticSession is needed — the EDC16_C3_VCD-X70 definition doesn't
        send one either; the fast-init handshake alone is enough.
        """
        log.info("Setting KWP2000 header for target 0x7A...")
        r = self._send_raw("ATSH817AF1")
        log.debug(f"ATSH817AF1 -> {r!r}")

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

    def __init__(self, elm: ELM327, logger: "TripLogger | None" = None):
        self.elm = elm
        self.data: dict = {}
        self.errors: list = []
        self.running = False
        self._thread = None
        self.last_update = 0.0
        self.logger = logger
        self.ignition = "unknown"  # "on" | "off" | "unknown"
        self._consecutive_empty = 0
        self.dtcs: list = []
        self.dtc_events: list = []
        self._dtc_poll_counter = 0

    def _poll_dtcs(self):
        """Read active DTCs (KWP2000 SID 0x17, group 0xFF00 = all groups)."""
        resp = self.elm.send_kwp("17FF00")
        if resp is None or len(resp) < 2:
            return
        ndtc = resp[1]
        new_dtcs = []
        idx = 2
        for _ in range(ndtc):
            if idx + 3 > len(resp):
                break
            hi, lo, status = resp[idx], resp[idx + 1], resp[idx + 2]
            new_dtcs.append({
                "code": f"{hi:02X}{lo:02X}",
                "current": bool(status & 0x04),
                "historical": bool(status & 0x02),
            })
            idx += 3
        old_codes = {d["code"] for d in self.dtcs}
        new_codes = {d["code"] for d in new_dtcs}
        now = datetime.now().isoformat(timespec="seconds")
        for code in sorted(new_codes - old_codes):
            log.warning(f"DTC appeared: {code}")
            self.dtc_events.append({"ts": now, "code": code, "event": "appeared"})
        for code in sorted(old_codes - new_codes):
            log.info(f"DTC no longer active: {code}")
            self.dtc_events.append({"ts": now, "code": code, "event": "cleared"})
        self.dtc_events = self.dtc_events[-200:]
        self.dtcs = new_dtcs

    def clear_dtcs(self) -> bool:
        """Clear all diagnostic information (KWP2000 SID 0x14, group 0xFF00)."""
        resp = self.elm.send_kwp("14FF00")
        ok = resp is not None and len(resp) >= 1 and resp[0] == 0x54
        log.info(f"Clear DTCs -> {'OK' if ok else 'failed'} ({resp.hex().upper() if resp else 'no response'})")
        if ok:
            now = datetime.now().isoformat(timespec="seconds")
            for d in self.dtcs:
                self.dtc_events.append({"ts": now, "code": d["code"], "event": "cleared_by_user"})
            self.dtc_events = self.dtc_events[-200:]
            self.dtcs = []
        return ok

    def _poll_trame(self, name: str, trame: dict) -> bool:
        cmd_hex = trame["cmd"].hex().upper()
        resp = self.elm.send_kwp(cmd_hex)
        if resp is None:
            log.warning(f"Trame {name}: no response")
            return False
        # Always capture the raw hex, even a negative response (7F ...) or a
        # too-short frame, so nothing polled is ever silently discarded.
        self.data[f"{name}_raw"] = {"value": resp.hex().upper(), "unit": "hex"}
        if len(resp) < trame["min_bytes"]:
            log.warning(f"Trame {name}: short response ({len(resp)} bytes): {resp.hex().upper()}")
            return False
        for param, (fb, nb, signed, scale, offset, unit) in trame["params"].items():
            try:
                val = decode_value(resp, fb, nb, signed, scale, offset)
                # hPa -> bar is far more readable for every pressure signal
                if unit == "hPa":
                    val = val / 1000.0
                    unit = "bar"
                self.data[param] = {"value": round(val, 3), "unit": unit}
            except Exception as e:
                log.debug(f"Decode error {param}: {e}")
        return True

    def poll_once(self):
        any_success = False
        for name, trame in TRAMES.items():
            if self._poll_trame(name, trame):
                any_success = True
        self.last_update = time.time()
        if any_success:
            self._dtc_poll_counter += 1
            if self._dtc_poll_counter >= 5:  # ~every 5th cycle, so pressure polling stays responsive
                self._dtc_poll_counter = 0
                try:
                    self._poll_dtcs()
                except Exception as e:
                    log.debug(f"DTC poll error: {e}")
        self._mark_cycle(any_success)

    def _mark_cycle(self, any_success: bool):
        if any_success:
            self._consecutive_empty = 0
            if self.ignition != "on":
                self.ignition = "on"
                if self.logger:
                    self.logger.start_trip()
            if self.logger and self.logger.active:
                self.data["Active DTCs"] = {"value": ";".join(d["code"] for d in self.dtcs), "unit": ""}
                self.logger.log_row(self.data)
        else:
            self._consecutive_empty += 1
            if self._consecutive_empty >= IGNITION_OFF_CYCLES and self.ignition != "off":
                self.ignition = "off"
                log.warning("No response on any trame — assuming ignition OFF.")
                self.data = {}
                if self.logger:
                    self.logger.end_trip()

    def _loop(self):
        while self.running:
            try:
                self.poll_once()
            except Exception as e:
                log.error(f"Poll error: {e}")
                self.last_update = time.time()
                self._mark_cycle(False)
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

    def do_POST(self):
        if self.path == "/dtc/clear":
            self._clear_dtcs()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_data(self):
        payload = {
            "ts": time.time(),
            "data": session.data if session else {},
            "ignition": session.ignition if session else "unknown",
            "logging": bool(session.logger and session.logger.active) if session else False,
            "log_file": str(session.logger.path) if session and session.logger and session.logger.active else None,
            "dtcs": session.dtcs if session else [],
            "dtc_events": session.dtc_events[-20:] if session else [],
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

    def _clear_dtcs(self):
        ok = False
        if session is not None:
            try:
                ok = session.clear_dtcs()
            except Exception as e:
                log.error(f"Clear DTCs failed: {e}")
        body = json.dumps({"ok": ok}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    global session

    parser = argparse.ArgumentParser(description="EDC16 M9R KWP2000 diagnostic server")
    parser.add_argument("--port", default="COM3", help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=38400, help="ELM327 serial baud rate")
    parser.add_argument("--http-port", type=int, default=8765, help="HTTP server port")
    parser.add_argument("--mock", action="store_true", help="Run with mock data (no hardware needed)")
    parser.add_argument("--log-dir", default="logs", help="Directory to write per-trip CSV logs into")
    parser.add_argument("--no-log", action="store_true", help="Disable CSV trip logging")
    args = parser.parse_args()

    trip_logger = None if args.no_log else TripLogger(args.log_dir)

    if args.mock:
        log.info("Running in MOCK mode - no hardware required")
        session = _MockSession(trip_logger)
        session.start()
    else:
        log.info(f"Connecting to ELM327 on {args.port} at {args.baud} baud...")
        elm = ELM327(args.port, args.baud)
        elm.init_elm()
        elm.kwp_init()
        session = DiagSession(elm, trip_logger)
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
    """Simulates a degraded pump scenario for UI testing without hardware.

    Also cycles ignition on/off every couple of minutes so the trip-logging
    and ignition-off UI states can be exercised without real hardware.
    """
    def __init__(self, logger: "TripLogger | None" = None):
        self.data = {}
        self.last_update = 0.0
        self.running = False
        self.logger = logger
        self.ignition = "on"
        self._t = 0.0
        self.dtcs: list = []
        self.dtc_events: list = []

    def clear_dtcs(self) -> bool:
        now = datetime.now().isoformat(timespec="seconds")
        for d in self.dtcs:
            self.dtc_events.append({"ts": now, "code": d["code"], "event": "cleared_by_user"})
        self.dtcs = []
        return True

    def start(self):
        self.running = True
        if self.logger:
            self.logger.start_trip()
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def stop(self):
        self.running = False
        if self.logger:
            self.logger.end_trip()

    def _loop(self):
        import math
        while self.running:
            self._t += 0.3

            # Simulate the engine being switched off for ~10s every ~70s of mock time.
            cycle = self._t % 70
            if cycle > 60:
                if self.ignition != "off":
                    self.ignition = "off"
                    self.data = {}
                    if self.logger:
                        self.logger.end_trip()
                self.last_update = time.time()
                time.sleep(0.3)
                continue
            elif self.ignition != "on":
                self.ignition = "on"
                if self.logger:
                    self.logger.start_trip()

            rpm = 800 + 2200 * abs(math.sin(self._t / 10))
            # Simulate pressure falling short above 2500 rpm
            pressure_limit = 1600 if rpm < 2500 else 1100 + 100 * math.sin(self._t)
            setpoint = min(1600, 800 + rpm * 0.35)
            actual = min(pressure_limit, setpoint - max(0, rpm - 2500) * 0.15)
            duty = min(99.9, 50 + (setpoint - actual) * 0.1)
            qty = 5 + 30 * abs(math.sin(self._t / 8))

            # Simulate a rail-pressure DTC setting when duty saturates hard, so
            # the DTC panel/marker have something to show without hardware.
            saturated = duty > 90 and (setpoint - actual) > 50
            sim_codes = {d["code"] for d in self.dtcs}
            if saturated and "0087" not in sim_codes:
                self.dtcs.append({"code": "0087", "current": True, "historical": False})
                self.dtc_events.append({"ts": datetime.now().isoformat(timespec="seconds"), "code": "0087", "event": "appeared"})
            elif not saturated and "0087" in sim_codes:
                self.dtcs = [d for d in self.dtcs if d["code"] != "0087"]
                self.dtc_events.append({"ts": datetime.now().isoformat(timespec="seconds"), "code": "0087", "event": "cleared"})

            self.data = {
                "Régime moteur":                                       {"value": round(rpm), "unit": "tr/min"},
                "Vitesse véhicule":                                    {"value": round(rpm * 0.04, 1), "unit": "km / h"},
                "Pression rail actuelle":                              {"value": round(actual / 10, 1), "unit": "bar"},
                "Consigne de pression rail":                           {"value": round(setpoint / 10, 1), "unit": "bar"},
                "RCO MPROP":                                           {"value": round(duty, 1), "unit": "%"},
                "Courant de la MPROP mesuré":                          {"value": round(duty * 8), "unit": "mA"},
                "Consigne de courant de la MPROP":                     {"value": round(setpoint * 0.5), "unit": "mA"},
                "Débit désiré avant limitation système et sans LiGov": {"value": round(qty, 1), "unit": "mg/cyc"},
                "Débit injecté":                                       {"value": round(qty * 0.95, 1), "unit": "mg/cyc"},
                "Débit maximal":                                       {"value": 100.0, "unit": "mg/cyc"},
                "Débit poste à poste 1":                               {"value": round(qty * 0.24, 2), "unit": "mg/hub"},
                "Débit poste à poste 2":                               {"value": round(qty * 0.25, 2), "unit": "mg/hub"},
                "Débit poste à poste 3":                               {"value": round(qty * 0.26, 2), "unit": "mg/hub"},
                "Débit poste à poste 4":                               {"value": round(qty * 0.25, 2), "unit": "mg/hub"},
                "Couple Moteur effectif":                              {"value": round(qty * 5.2, 1), "unit": "Nm"},
                "Température eau":                                     {"value": 87.0, "unit": "°C"},
                "Température carburant":                               {"value": 42.0, "unit": "°C"},
                "Tension batterie":                                    {"value": 14.2, "unit": "V"},
                "Pression carburant":                                  {"value": round(4.2 + 0.3 * math.sin(self._t), 2), "unit": "bar"},
                "Tension capteur pression rail":                       {"value": round(1.17 + actual / 50000, 3), "unit": "V"},
                "Ratio pédale accélérateur":                           {"value": round(50 * abs(math.sin(self._t / 8)), 1), "unit": "%"},
            }
            self.last_update = time.time()
            if self.logger:
                self.data["Active DTCs"] = {"value": ";".join(d["code"] for d in self.dtcs), "unit": ""}
                self.logger.log_row(self.data)
            time.sleep(0.3)


if __name__ == "__main__":
    main()
