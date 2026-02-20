#!/usr/bin/env python3

"""
comms_test.py — Serial JSON-RPC comms runner + TSV logger

What it does
- Connects to the display over a serial port (default 115200 baud).
- Sends:
  1) get_device_info  (to read MAC/serial from the device)
  2) set_variant      (after device_info)
  3) set_state loop for ALERT values  (each alert held for 5 seconds)
  4) set_state loop for SYSERR values (each syserr held for 5 seconds)
  5) set_state loop for MODE values   (each mode held for 5 seconds)
- During each 5-second “hold”, the script keeps re-sending set_state every --freq_ms.
- Writes a TSV log file with one row per request:
  req_ts, method, params, mac_id, result, elapsed_time_ms

How to run
--- Packages Needed:
   1)Install python3:sudo apt update
                     sudo apt install -y python3
   2)Install pip(python package manager):sudo apt install -y python3-pip
   3)Install PySerial:pip3 install pyserial
   4)Install CH341 Drivers for USB0 port: Follow these steps :
                                                  1)sudo apt install -y linux-modules-extra-$(uname -r)
                                                  2)sudo modprobe ch341
                                                  3)sudo apt remove --purge -y brltty
                                                  4)sudo reboot
--- Verify whether the port exists with this -- ls /dev/ttyUSB*
 Run with your serial port & frequency (ms):
   python3 comms_test.py --port /dev/ttyUSB0 --freq_ms 300 --time n(sec) or --count N(times) 
NOTE: Both time and count cannot be passes in same command 

Output
- By default, creates:
  comms_stability_<unix_timestamp>.tsv
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import serial  # pip install pyserial

# -----------------------------
# TSV config
# -----------------------------
SEP = "\t"


def _tsv_clean(text: object) -> str:
    if text is None:
        return ""
    s = str(text)
    return s.replace("\t", " ").replace("\r", " ").replace("\n", " ").strip()


def format_log_line(
    req_time_stamp: str,
    req_method: str,
    params: str,
    dev_mac_id: str,
    result: str,
    elapsed_time_ms: int,
) -> str:
    return SEP.join(
        [
            _tsv_clean(req_time_stamp),
            _tsv_clean(req_method),
            _tsv_clean(params),
            _tsv_clean(dev_mac_id),
            _tsv_clean(result),
            _tsv_clean(elapsed_time_ms),
        ]
    )


LOG_HEADER = format_log_line(
    "req_ts",
    "method",
    "params",
    "mac_id",
    "result",
    "elapsed_time_ms",
)

# -----------------------------
# Config
# -----------------------------
@dataclass(frozen=True)
class SerialConfig:
    port: str
    baud: int = 115200
    timeout_ms: int = 0  # placeholder, computed from freq_ms - 10


@dataclass(frozen=True)
class RunConfig:
    set_variant_id: int = 1
    device_info_id: int = 2
    log_file: str = "AUTO"
    freq_ms: int = 0  # required via CLI
    timeout_ms: int = 0  # placeholder, computed from freq_ms - 10
    time_s: Optional[int] = None  # None => infinite
    count: Optional[int] = None  # None => infinite


class _StopNow(Exception):
    pass


# -----------------------------
# Time helpers
# -----------------------------
def unix_timestamp_ms() -> str:
    return str(int(time.time() * 1000))


def elapsed_ms_between(t_start: float, t_end: float) -> int:
    return int((t_end - t_start) * 1000)


def _time_up(end_time: Optional[float]) -> bool:
    return end_time is not None and time.monotonic() >= end_time


def _must_stop(end_time: Optional[float]) -> None:
    if _time_up(end_time):
        raise _StopNow()


def sleep_to_next_tick(next_tick: float, end_time: Optional[float]) -> None:
    _must_stop(end_time)
    now = time.monotonic()
    if next_tick > now:
        if end_time is not None:
            sleep_until = min(next_tick, end_time)
        else:
            sleep_until = next_tick
        delay = sleep_until - now
        if delay > 0:
            time.sleep(delay)
    _must_stop(end_time)


# -----------------------------
# Serial helpers
# -----------------------------
def open_serial_port(cfg: SerialConfig) -> serial.Serial:
    timeout_s = max(0.001, cfg.timeout_ms / 1000.0)

    ser = serial.Serial(
        port=cfg.port,
        baudrate=cfg.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout_s,
        write_timeout=timeout_s,
    )

    time.sleep(0.2)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    return ser


def write_line_lf(ser: serial.Serial, payload_line: str, end_time: Optional[float]) -> float:
    _must_stop(end_time)
    if not payload_line.endswith("\n"):
        payload_line += "\n"
    ser.write(payload_line.encode("utf-8", errors="replace"))
    ser.flush()
    _must_stop(end_time)
    return time.monotonic()


def read_jsonrpc_response_by_id(
    ser: serial.Serial,
    expected_id: int,
    timeout_ms: int,
    end_time: Optional[float],
) -> Tuple[Optional[dict], str, Optional[float]]:
    _must_stop(end_time)

    local_deadline = time.monotonic() + (timeout_ms / 1000.0)
    if end_time is not None:
        deadline = min(local_deadline, end_time)
    else:
        deadline = local_deadline

    while time.monotonic() < deadline:
        _must_stop(end_time)

        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        print(f"[RX] {line}")

        if not (line.startswith("{") and line.endswith("}")):
            continue

        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue

        if msg.get("id") == expected_id:
            _must_stop(end_time)
            return msg, "null", time.monotonic()

    _must_stop(end_time)
    return None, "timeout", None


# -----------------------------
# JSON-RPC builders
# -----------------------------
def build_set_variant(req_id: int) -> dict:
    return {
        "jsonrpc": "2.0",
        "method": "set_variant",
        "params": {"variant": 1, "fts_avail": 127},
        "id": req_id,
    }


def build_get_device_info(req_id: int) -> dict:
    return {"jsonrpc": "2.0", "method": "get_device_info", "id": req_id}


def build_set_state(req_id: int, *, alert: int, syserr: int = 0, mode: int = 1) -> dict:
    return {
        "jsonrpc": "2.0",
        "method": "set_state",
        "params": {
            "screen": 2,
            "fts_avail": 127,
            "fts_state": 7,
            "bed_pos": 1,
            "bed_wid": 60,
            "occ_size": 2,
            "mon_start": "2130",
            "mon_end": "0700",
            "bms": 1,
            "vol": 2,
            "audio": 1,
            "lang": "English",
            "mode": mode,
            "pause_tmr": 0,
            "alert": alert,
            "alert_title": "",
            "syserr": syserr,
            "syserr_title": "",
            "abc": 0,
            "cal": 0,
            "room_number": "1001-a",
        },
        "id": req_id,
    }


# -----------------------------
# Param formatting (for logs)
# -----------------------------
def params_to_compact_str(params: object) -> str:
    if not isinstance(params, dict) or not params:
        return "-"

    parts = []
    for k in sorted(params.keys()):
        v = params[k]
        if v is None:
            v_str = "-"
        elif isinstance(v, bool):
            v_str = "1" if v else "0"
        else:
            v_str = str(v).replace("\n", " ").replace("\r", " ").replace("\t", " ").strip()
            if v_str == "":
                v_str = "-"
        parts.append(f"{k}={v_str}")

    return ",".join(parts)


# -----------------------------
# Parsing helpers
# -----------------------------
def extract_mac_from_device_info(resp: dict) -> Tuple[Optional[str], str]:
    if "error" in resp and resp["error"] is not None:
        err = resp["error"]
        if isinstance(err, dict) and "code" in err:
            return None, str(err.get("code"))
        return None, "error"

    result = resp.get("result")
    if not isinstance(result, dict):
        return None, "error"

    mac = result.get("serial")
    if not isinstance(mac, str) or not mac.strip():
        return None, "error"

    return mac.strip(), "null"


def result_field_from_response(resp: Optional[dict], status: str) -> str:
    if status == "timeout" or resp is None:
        return "-"
    err = resp.get("error")
    if err is None:
        return "null"
    if isinstance(err, dict) and "code" in err:
        return str(err.get("code"))
    return "error"


# -----------------------------
# Logging helpers (TSV)
# -----------------------------
def ensure_log_header(path: str) -> None:
    needs_header = (not os.path.exists(path)) or (os.path.getsize(path) == 0)
    if needs_header:
        with open(path, "a", encoding="utf-8", newline="\n") as f:
            f.write(LOG_HEADER + "\n")
            f.flush()


def append_log_row(
    path: str,
    req_time_stamp: str,
    req_method: str,
    params: str,
    dev_mac_id: str,
    result: str,
    elapsed_time_ms: int,
) -> None:
    ensure_log_header(path)
    line = format_log_line(req_time_stamp, req_method, params, dev_mac_id, result, elapsed_time_ms)
    with open(path, "a", encoding="utf-8", newline="\n") as f:
        f.write(line + "\n")
        f.flush()


# -----------------------------
# Helper: send + log
# -----------------------------
def _send_and_log(
    *,
    ser: serial.Serial,
    log_path: str,
    req_obj: dict,
    expected_id: int,
    mac_for_log: str,
    timeout_ms: int,
    end_time: Optional[float],
) -> Tuple[Optional[dict], str, int]:
    _must_stop(end_time)

    req_time = unix_timestamp_ms()
    req_method = str(req_obj.get("method", "-"))
    params_field = params_to_compact_str(req_obj.get("params"))

    req_line = json.dumps(req_obj, separators=(",", ":"))
    print(f"[TX] {req_line}")

    tx_t = write_line_lf(ser, req_line, end_time=end_time)
    resp, status, rx_t = read_jsonrpc_response_by_id(
        ser, expected_id=expected_id, timeout_ms=timeout_ms, end_time=end_time
    )

    if status == "timeout" or rx_t is None:
        elapsed = timeout_ms
        result = "-"
    else:
        elapsed = elapsed_ms_between(tx_t, rx_t)
        result = result_field_from_response(resp, status)

    append_log_row(
        path=log_path,
        req_time_stamp=req_time,
        req_method=req_method,
        params=params_field,
        dev_mac_id=mac_for_log,
        result=result,
        elapsed_time_ms=elapsed,
    )

    _must_stop(end_time)
    return resp, status, elapsed


# -----------------------------
# CLI + main
# -----------------------------
def parse_args() -> Tuple[SerialConfig, RunConfig]:
    p = argparse.ArgumentParser(description="comms_test.py - TSV log + alert+syserr+mode hold runner.")

    p.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyACM0, /dev/ttyUSB0).")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200).")

    p.add_argument("--set_variant_id", type=int, default=1, help="JSON-RPC id for set_variant (default: 1).")
    p.add_argument("--device_info_id", type=int, default=2, help="JSON-RPC id for get_device_info (default: 2).")

    p.add_argument(
        "--log_file",
        default="AUTO",
        help="Log file path. Use AUTO to create comms_stability_<unix_timestamp_ms>.tsv (default: AUTO).",
    )

    # REQUIRED now (no default)
    p.add_argument("--freq_ms", type=int, required=True, help="Send frequency for set_state inside each 5s hold.")

    p.add_argument(
        "--count",
        type=int,
        default=None,
        help="How many times to run the full program flow. If omitted, runs forever.",
    )

    p.add_argument(
        "--time",
        type=int,
        default=None,
        help="How long to run the full program in seconds. If omitted, runs forever.",
    )

    a = p.parse_args()

    if a.freq_ms <= 0:
        print("--freq_ms must be > 0", file=sys.stderr)
        raise SystemExit(2)

    if a.count is not None and a.count <= 0:
        a.count = None

    if a.time is not None and a.time <= 0:
        a.time = None

    if a.count is not None and a.time is not None:
        print("Warning: time and count shouldn't passed same time", file=sys.stderr)
        raise SystemExit(2)

    computed_timeout_ms = max(1, int(a.freq_ms) - 10)

    if a.log_file == "AUTO":
        a.log_file = f"comms_stability_{unix_timestamp_ms()}.tsv"

    return (
        SerialConfig(port=a.port, baud=a.baud, timeout_ms=computed_timeout_ms),
        RunConfig(
            set_variant_id=a.set_variant_id,
            device_info_id=a.device_info_id,
            log_file=a.log_file,
            freq_ms=a.freq_ms,
            timeout_ms=computed_timeout_ms,
            time_s=a.time,
            count=a.count,
        ),
    )


def run_once(ser: serial.Serial, r_cfg: RunConfig, end_time: Optional[float]) -> int:
    _must_stop(end_time)

    dev_mac = "-"

    req_time = unix_timestamp_ms()
    gdi_obj = build_get_device_info(r_cfg.device_info_id)
    gdi_line = json.dumps(gdi_obj, separators=(",", ":"))
    print(f"[TX] {gdi_line}")

    tx_t = write_line_lf(ser, gdi_line, end_time=end_time)
    resp, status, rx_t = read_jsonrpc_response_by_id(
        ser, expected_id=r_cfg.device_info_id, timeout_ms=r_cfg.timeout_ms, end_time=end_time
    )

    if status == "timeout" or rx_t is None or resp is None:
        append_log_row(
            path=r_cfg.log_file,
            req_time_stamp=req_time,
            req_method="get_device_info",
            params=params_to_compact_str(gdi_obj.get("params")),
            dev_mac_id="-",
            result="-",
            elapsed_time_ms=r_cfg.timeout_ms,
        )
    else:
        elapsed = elapsed_ms_between(tx_t, rx_t)
        mac, mac_status = extract_mac_from_device_info(resp)
        dev_mac = mac if mac else "-"

        append_log_row(
            path=r_cfg.log_file,
            req_time_stamp=req_time,
            req_method="get_device_info",
            params=params_to_compact_str(gdi_obj.get("params")),
            dev_mac_id=dev_mac,
            result=mac_status if mac_status else "-",
            elapsed_time_ms=elapsed,
        )

        if mac:
            print(f"firmware identified: {mac}")

    _must_stop(end_time)

    # Step 2: set_variant
    sv_obj = build_set_variant(r_cfg.set_variant_id)
    _send_and_log(
        ser=ser,
        log_path=r_cfg.log_file,
        req_obj=sv_obj,
        expected_id=r_cfg.set_variant_id,
        mac_for_log=dev_mac,
        timeout_ms=r_cfg.timeout_ms,
        end_time=end_time,
    )

    alert_states = [{"alert": 1}, {"alert": 2}, {"alert": 101}, {"alert": 102}]
    syserr_states = [{"syserr": 11}, {"syserr": 12}, {"syserr": 13}, {"syserr": 41}, {"syserr": 42}, {"syserr": 61}]
    mode_states = [{"mode": 1}, {"mode": 2}, {"mode": 3}, {"mode": 4}, {"mode": 5}]

    HOLD_S = 5.0
    freq_s = max(0.01, r_cfg.freq_ms / 1000.0)
    next_id = r_cfg.device_info_id + 1

    for st in alert_states:
        _must_stop(end_time)
        hold_deadline = time.monotonic() + HOLD_S
        next_tick = time.monotonic()
        while time.monotonic() < hold_deadline:
            _must_stop(end_time)
            sleep_to_next_tick(next_tick, end_time=end_time)
            next_tick += freq_s
            req_obj = build_set_state(next_id, alert=st["alert"], syserr=0, mode=1)
            _send_and_log(
                ser=ser,
                log_path=r_cfg.log_file,
                req_obj=req_obj,
                expected_id=next_id,
                mac_for_log=dev_mac,
                timeout_ms=r_cfg.timeout_ms,
                end_time=end_time,
            )
            next_id += 1

    for st in syserr_states:
        _must_stop(end_time)
        hold_deadline = time.monotonic() + HOLD_S
        next_tick = time.monotonic()
        while time.monotonic() < hold_deadline:
            _must_stop(end_time)
            sleep_to_next_tick(next_tick, end_time=end_time)
            next_tick += freq_s
            req_obj = build_set_state(next_id, alert=0, syserr=st["syserr"], mode=1)
            _send_and_log(
                ser=ser,
                log_path=r_cfg.log_file,
                req_obj=req_obj,
                expected_id=next_id,
                mac_for_log=dev_mac,
                timeout_ms=r_cfg.timeout_ms,
                end_time=end_time,
            )
            next_id += 1

    for st in mode_states:
        _must_stop(end_time)
        hold_deadline = time.monotonic() + HOLD_S
        next_tick = time.monotonic()
        while time.monotonic() < hold_deadline:
            _must_stop(end_time)
            sleep_to_next_tick(next_tick, end_time=end_time)
            next_tick += freq_s
            req_obj = build_set_state(next_id, alert=0, syserr=0, mode=st["mode"])
            _send_and_log(
                ser=ser,
                log_path=r_cfg.log_file,
                req_obj=req_obj,
                expected_id=next_id,
                mac_for_log=dev_mac,
                timeout_ms=r_cfg.timeout_ms,
                end_time=end_time,
            )
            next_id += 1

    _must_stop(end_time)
    return 0


def main() -> int:
    s_cfg, r_cfg = parse_args()

    try:
        ser = open_serial_port(s_cfg)
    except serial.SerialException as e:
        print(f"Unable to open port {s_cfg.port}: {e}", file=sys.stderr)
        return 2

    end_time: Optional[float] = None
    if r_cfg.time_s is not None:
        end_time = time.monotonic() + float(r_cfg.time_s)

    try:
        if r_cfg.count is not None:
            run_idx = 0
            while run_idx < r_cfg.count:
                run_idx += 1
                _ = run_once(ser, r_cfg, end_time=end_time)
            return 0

        while True:
            _ = run_once(ser, r_cfg, end_time=end_time)

    except _StopNow:
        return 0

    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

