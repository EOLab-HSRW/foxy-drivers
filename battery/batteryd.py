#!/usr/bin/env python3
"""
batteryd.py

Linux daemon wrapper around the sibling battery.py RobotBattery API.

Expected layout (install):

  /opt/batteryd/
    battery.py
    batteryd.py

Design:
  - battery.py owns USB serial discovery, termios, UART framing, packet parsing,
    and cached battery status.
  - batteryd.py owns daemon lifecycle, journald-friendly logging, status-file
    publication, and the local Unix-socket command API.
  - C/C++ clients can either read /run/batteryd/status or use the ASCII control
    socket at /run/batteryd/control.sock.

Control socket commands:
  PING
  VERSION
  GET
  SHUTDOWN

Responses:
  OK
  OK key=value key=value ...
  ERR error_code
"""

from __future__ import annotations

import io
import logging
import os
import queue
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

from battery import BATTERY_CMD_SEP, RobotBattery


# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

RUNTIME_DIR = os.environ.get("BATTERYD_RUNTIME_DIR", "/run/batteryd")
STATUS_PATH = os.path.join(RUNTIME_DIR, "status")
CONTROL_SOCKET_PATH = os.path.join(RUNTIME_DIR, "control.sock")

# Optional explicit serial path. If unset, battery.py's RobotBattery scans for
# the configured battery VID/PID by itself.
BATTERYD_SERIAL = os.environ.get("BATTERYD_SERIAL", "")

# Public daemon command SHUTDOWN maps to this raw battery command.
# The battery.py API already defines BATTERY_CMD_SEP as CRLF.
BATTERY_SHUTDOWN_COMMAND = os.environ.get("BATTERYD_SHUTDOWN_CMD", "QQ").encode("ascii") + BATTERY_CMD_SEP

MAX_CLIENT_COMMAND_BYTES = 128
STATUS_REFRESH_SEC = float(os.environ.get("BATTERYD_STATUS_REFRESH_SEC", "1.0"))
STALE_AFTER_SEC = float(os.environ.get("BATTERYD_STALE_AFTER_SEC", "3.0"))
RECONNECT_DELAY_SEC = float(os.environ.get("BATTERYD_RECONNECT_DELAY_SEC", "2.0"))

PROTOCOL_VERSION = 1
DAEMON_VERSION = "0.2.0"


# -----------------------------------------------------------------------------
# Runtime state
# -----------------------------------------------------------------------------

@dataclass
class DaemonState:
    battery: Optional[RobotBattery] = None
    online: int = 0
    last_error: str = ""


state = DaemonState()
state_lock = threading.Lock()
command_queue: "queue.Queue[bytes]" = queue.Queue()
stop_event = threading.Event()


# -----------------------------------------------------------------------------
# Logging
# -----------------------------------------------------------------------------

class SystemdJournalFormatter(logging.Formatter):
    """
    Formatter intended for services launched by systemd.

    systemd already records timestamp, unit name, PID, UID, GID, executable,
    cgroup, and other metadata. The daemon should therefore avoid duplicating
    timestamps in the message body.

    The leading <N> prefix is the traditional syslog priority prefix. journald
    understands it when SyslogLevelPrefix is enabled for the service, which is
    the default on normal systemd systems.
    """

    LEVEL_TO_SYSLOG_PRIORITY = {
        logging.CRITICAL: 2,
        logging.ERROR: 3,
        logging.WARNING: 4,
        logging.INFO: 6,
        logging.DEBUG: 7,
    }

    def format(self, record: logging.LogRecord) -> str:
        priority = self.LEVEL_TO_SYSLOG_PRIORITY.get(record.levelno, 6)
        message = super().format(record)
        return f"<{priority}>{message}"


def setup_logging() -> None:
    level_name = os.environ.get("BATTERYD_LOG_LEVEL", "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)

    handler = logging.StreamHandler(stream=sys.stderr)
    handler.setLevel(level)
    handler.setFormatter(SystemdJournalFormatter(
        fmt="batteryd[%(process)d]: %(levelname)s %(threadName)s: %(message)s"
    ))

    root = logging.getLogger()
    root.handlers.clear()
    root.setLevel(level)
    root.addHandler(handler)


# -----------------------------------------------------------------------------
# Status formatting and status file publication
# -----------------------------------------------------------------------------

def _int_or_default(value: object, default: int = 0) -> int:
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def snapshot_status_line() -> str:
    """
    Build the stable C/C++ client status line.

    Keep this format conservative: one ASCII line with space-separated key=value
    pairs and integer units where possible.
    """
    with state_lock:
        battery = state.battery
        daemon_online = state.online
        last_error = state.last_error

    if battery is None or not daemon_online:
        error_part = f" error={last_error}" if last_error else ""
        return (
            "online=0 "
            "stale=1 "
            "percent=-1 "
            "voltage_mv=0 "
            "current_ma=0 "
            "temperature_mc=0 "
            "time_to_empty_min=-1 "
            "cycle_count=-1 "
            "usb_out_1_mv=0 "
            "usb_out_2_mv=0 "
            "charger_voltage_mv=0 "
            "age_ms=-1 "
            "packet_count=0" + error_part
        )

    status = battery.status
    age_seconds = battery.last_status_age_seconds
    stale = 1 if battery.is_status_stale(STALE_AFTER_SEC) else 0
    online = 0 if stale else 1
    age_ms = -1 if age_seconds is None else int(age_seconds * 1000)

    if status is None:
        return (
            f"online=0 stale=1 percent=-1 voltage_mv=0 current_ma=0 "
            f"temperature_mc=0 time_to_empty_min=-1 cycle_count=-1 "
            f"usb_out_1_mv=0 usb_out_2_mv=0 charger_voltage_mv=0 "
            f"age_ms={age_ms} packet_count={battery.packet_count}"
        )

    temperature_mc = 0
    if status.temperature_celsius is not None:
        temperature_mc = int(status.temperature_celsius * 1000)

    return (
        f"online={online} "
        f"stale={stale} "
        f"percent={_int_or_default(status.percentage, -1)} "
        f"voltage_mv={_int_or_default(status.voltage_mv)} "
        f"current_ma={_int_or_default(status.current_ma)} "
        f"temperature_mc={temperature_mc} "
        f"time_to_empty_min={_int_or_default(status.time_to_empty_min, -1)} "
        f"cycle_count={_int_or_default(status.cycle_count, -1)} "
        f"usb_out_1_mv={_int_or_default(status.usb_out_1_mv)} "
        f"usb_out_2_mv={_int_or_default(status.usb_out_2_mv)} "
        f"charger_voltage_mv={_int_or_default(status.charger_voltage_mv)} "
        f"age_ms={age_ms} "
        f"packet_count={battery.packet_count}"
    )


def write_status_file_atomic() -> None:
    os.makedirs(RUNTIME_DIR, exist_ok=True)
    tmp_path = STATUS_PATH + ".tmp"
    line = snapshot_status_line()

    with io.open(tmp_path, "w", encoding="ascii") as f:
        f.write(line + "\n")
        f.flush()
        os.fsync(f.fileno())

    os.replace(tmp_path, STATUS_PATH)
    os.chmod(STATUS_PATH, 0o644)


# -----------------------------------------------------------------------------
# Battery worker
# -----------------------------------------------------------------------------

def create_battery() -> RobotBattery:
    if BATTERYD_SERIAL:
        return RobotBattery(device_path=BATTERYD_SERIAL, wait_for_first_status=True)

    return RobotBattery(wait_for_first_status=True)


def drain_control_commands(battery: RobotBattery) -> None:
    while True:
        try:
            raw_command = command_queue.get_nowait()
        except queue.Empty:
            return

        # RobotBattery owns the UARTManager object. It continuously reads from
        # the UART in its own background thread, but occasional command writes
        # are safe to serialize through this daemon worker.
        battery.uart_manager.write(raw_command)
        logging.info("sent battery command: %r", raw_command)


def battery_worker() -> None:
    while not stop_event.is_set():
        battery: Optional[RobotBattery] = None

        try:
            logging.info("opening battery API")
            battery = create_battery()

            info = battery.info
            logging.info(
                "battery opened: device=%s firmware=%s serial=%s",
                battery.device_path,
                info.firmware_version,
                info.serial_number,
            )

            with state_lock:
                state.battery = battery
                state.online = 1
                state.last_error = ""

            write_status_file_atomic()

            while not stop_event.is_set():
                drain_control_commands(battery)

                last_error = battery.last_error
                if last_error is not None:
                    raise RuntimeError(f"battery API error: {last_error}")

                write_status_file_atomic()
                stop_event.wait(STATUS_REFRESH_SEC)

        except Exception as exc:
            logging.exception("battery worker error")

            with state_lock:
                state.online = 0
                state.last_error = exc.__class__.__name__
                state.battery = None

            try:
                write_status_file_atomic()
            except Exception:
                logging.exception("failed to write offline status")

            if stop_event.wait(RECONNECT_DELAY_SEC):
                break

        finally:
            if battery is not None:
                battery.close()

            with state_lock:
                if state.battery is battery:
                    state.battery = None
                    state.online = 0


# -----------------------------------------------------------------------------
# Control socket server
# -----------------------------------------------------------------------------

def handle_control_command(command: str) -> str:
    command = command.strip().upper()

    if command == "":
        return "ERR empty_command\n"

    if command == "PING":
        return "OK\n"

    if command == "VERSION":
        return f"OK batteryd={DAEMON_VERSION} protocol={PROTOCOL_VERSION}\n"

    if command == "GET":
        return "OK " + snapshot_status_line() + "\n"

    if command == "SHUTDOWN":
        with state_lock:
            online = state.online
            battery = state.battery

        if not online or battery is None:
            return "ERR battery_offline\n"

        command_queue.put(BATTERY_SHUTDOWN_COMMAND)
        return "OK\n"

    return "ERR unknown_command\n"


def serve_one_control_client(conn: socket.socket) -> None:
    try:
        data = conn.recv(MAX_CLIENT_COMMAND_BYTES + 1)
        if not data:
            return

        if len(data) > MAX_CLIENT_COMMAND_BYTES:
            conn.sendall(b"ERR command_too_long\n")
            return

        try:
            command = data.decode("ascii")
        except UnicodeDecodeError:
            conn.sendall(b"ERR non_ascii_command\n")
            return

        response = handle_control_command(command)
        conn.sendall(response.encode("ascii"))

    except Exception:
        logging.exception("control client error")
        try:
            conn.sendall(b"ERR internal_error\n")
        except Exception:
            pass
    finally:
        conn.close()


def control_socket_server() -> None:
    os.makedirs(RUNTIME_DIR, exist_ok=True)

    if os.path.exists(CONTROL_SOCKET_PATH):
        os.unlink(CONTROL_SOCKET_PATH)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

    try:
        server.bind(CONTROL_SOCKET_PATH)
        os.chmod(CONTROL_SOCKET_PATH, 0o660)
        server.listen(16)
        server.settimeout(0.5)

        logging.info("control socket listening at %s", CONTROL_SOCKET_PATH)

        while not stop_event.is_set():
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue

            t = threading.Thread(target=serve_one_control_client, args=(conn,), daemon=True)
            t.start()

    finally:
        server.close()
        try:
            os.unlink(CONTROL_SOCKET_PATH)
        except FileNotFoundError:
            pass


# -----------------------------------------------------------------------------
# Process lifecycle
# -----------------------------------------------------------------------------

def install_signal_handlers() -> None:
    def stop(signum, frame):
        logging.info("received signal %s", signum)
        stop_event.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)


def main() -> int:
    setup_logging()
    install_signal_handlers()

    logging.info("starting batteryd")
    logging.info("runtime_dir=%s serial=%s", RUNTIME_DIR, BATTERYD_SERIAL or "auto")

    try:
        write_status_file_atomic()
    except Exception:
        logging.exception("failed to create initial status file")

    worker = threading.Thread(target=battery_worker, name="battery", daemon=True)
    worker.start()

    try:
        control_socket_server()
    finally:
        stop_event.set()
        worker.join(timeout=2.0)
        logging.info("batteryd stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

