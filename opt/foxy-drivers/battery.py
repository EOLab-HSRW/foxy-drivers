"""
This is the python hardware interface for Foxy robot.

It is mainly use to be consume by the batteryd (battery deamon)
after that you are suppose to use the battery using the 
plain ASCII text interface under /run/batteryd/status
or the socket use for control under /run/batteryd/control.sock
"""

from __future__ import annotations

import os
import re
import glob
import json
import time
import termios
import selectors
import threading

from pathlib import Path
from dataclasses import dataclass
from typing import Any, Optional


BATTERY_BAUD: int = 115200

# VID and PID refer to the microcontroller.
# They are not a unique identifier for a specific robot battery.
BATTERY_VID = "04D8".upper()
BATTERY_PID = "ECB7".upper()

BATTERY_CMD_SEP = b"\r\n"
BATTERY_CMD_CHECK = b"??"
BATTERY_RESPONSE_TIMEOUT: float = 2.0


def scan_for_battery() -> Optional[Path]:
    """
    Search for a /dev/ttyACM* device matching BATTERY_VID and BATTERY_PID.

    Returns:
        Path to the matching tty device, or None if not found.
    """

    for dev in glob.glob("/dev/ttyACM*"):
        dev_path = Path(dev)
        sysfs_path = Path("/sys/class/tty") / dev_path.name / "device"

        try:
            resolved_path = sysfs_path.resolve()
        except FileNotFoundError:
            continue

        for parent in [resolved_path] + list(resolved_path.parents):
            vid_file = parent / "idVendor"
            pid_file = parent / "idProduct"

            if not vid_file.exists() or not pid_file.exists():
                continue

            try:
                vid = vid_file.read_text().strip().upper()
                pid = pid_file.read_text().strip().upper()
            except OSError:
                continue

            if vid == BATTERY_VID and pid == BATTERY_PID:
                return dev_path

    return None


class UARTManager:
    """
    Small raw UART manager using os, termios, and selectors.

    It reads complete CRLF-terminated frames from the serial device.
    """

    def __init__(self, device_path: Path, baudrate: int):
        self.fd: int | None = None
        self.buf = bytearray()
        self.sel = selectors.DefaultSelector()
        self._closed = False

        baud_const = getattr(termios, f"B{baudrate}", None)
        if baud_const is None:
            raise ValueError(f"Unsupported baudrate: {baudrate}")

        fd = os.open(device_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

        try:
            attrs = termios.tcgetattr(fd)

            # attrs = [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
            attrs[0] = 0
            attrs[1] = 0

            attrs[2] = attrs[2] & ~termios.CSIZE
            attrs[2] = attrs[2] | termios.CS8 | termios.CREAD | termios.CLOCAL

            attrs[3] = 0

            attrs[4] = baud_const
            attrs[5] = baud_const

            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0

            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)

        except Exception:
            os.close(fd)
            raise

        self.fd = fd
        self.sel.register(fd, selectors.EVENT_READ)

    def close(self) -> None:
        if self._closed:
            return

        self._closed = True

        if self.fd is not None:
            try:
                self.sel.unregister(self.fd)
            except Exception:
                pass

        try:
            self.sel.close()
        except Exception:
            pass

        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            finally:
                self.fd = None

    def write(self, data: bytes) -> None:
        if self._closed or self.fd is None:
            raise RuntimeError("UART device is closed")

        os.write(self.fd, data)
        termios.tcdrain(self.fd)

    def frame(self, timeout: float = 2.0) -> bytes:
        """
        Read one complete CRLF-terminated frame.

        Returns:
            Frame bytes without the CRLF terminator.
        """

        if self._closed or self.fd is None:
            raise RuntimeError("UART device is closed")

        end = time.monotonic() + timeout

        while True:
            pos = self.buf.find(BATTERY_CMD_SEP)

            if pos >= 0:
                frame = bytes(self.buf[:pos])
                del self.buf[:pos + len(BATTERY_CMD_SEP)]
                return frame

            left = end - time.monotonic()

            if left <= 0:
                raise TimeoutError("no CRLF frame received")

            events = self.sel.select(left)

            if not events:
                raise TimeoutError("no CRLF frame received")

            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                continue

            if data:
                self.buf.extend(data)


@dataclass
class BatteryInfo:
    firmware_version: str | None = None
    serial_number: str | None = None


@dataclass(frozen=True)
class BatteryStatus:
    """
    Cached battery status.

    Public API names are intentionally clean:

        battery.percentage
        battery.temperature
        battery.voltage
        battery.current

    Raw battery packet names are preserved only in raw.
    """

    percentage: int | None = None

    temperature_kelvin: float | None = None
    temperature_celsius: float | None = None

    voltage_mv: int | None = None
    voltage: float | None = None

    current_ma: int | None = None
    current: float | None = None

    time_to_empty_min: int | None = None
    cycle_count: int | None = None

    usb_out_1_mv: int | None = None
    usb_out_1_voltage: float | None = None

    usb_out_2_mv: int | None = None
    usb_out_2_voltage: float | None = None

    charger_voltage_mv: int | None = None
    charger_voltage: float | None = None

    raw: dict[str, Any] | None = None

    @staticmethod
    def _as_int(value: Any) -> int | None:
        if value is None:
            return None

        try:
            return int(float(value))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _as_float(value: Any) -> float | None:
        if value is None:
            return None

        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @classmethod
    def from_packet(cls, packet: dict[str, Any]) -> BatteryStatus | None:
        """
        Convert a parsed battery packet into BatteryStatus.

        Expected packet format includes fields like:

            SOC(%)
            CellTemp(degK)
            CellVoltage(mV)
            Current(mA)
            TimeToEmpty(min)
            CycleCount
        """

        if "SOC(%)" not in packet:
            return None

        percentage = cls._as_int(packet.get("SOC(%)"))

        temp_k = cls._as_float(packet.get("CellTemp(degK)"))

        voltage_mv = cls._as_int(packet.get("CellVoltage(mV)"))
        current_ma = cls._as_int(packet.get("Current(mA)"))

        usb_1_mv = cls._as_int(packet.get("USB OUT-1(mV)"))
        usb_2_mv = cls._as_int(packet.get("USB OUT-2(mV)"))
        charger_mv = cls._as_int(packet.get("ChargerVoltage(mV)"))

        return cls(
            percentage=percentage,

            temperature_kelvin=temp_k,
            temperature_celsius=None if temp_k is None else temp_k - 273.15,

            voltage_mv=voltage_mv,
            voltage=None if voltage_mv is None else voltage_mv / 1000.0,

            current_ma=current_ma,
            current=None if current_ma is None else current_ma / 1000.0,

            time_to_empty_min=cls._as_int(packet.get("TimeToEmpty(min)")),
            cycle_count=cls._as_int(packet.get("CycleCount")),

            usb_out_1_mv=usb_1_mv,
            usb_out_1_voltage=None if usb_1_mv is None else usb_1_mv / 1000.0,

            usb_out_2_mv=usb_2_mv,
            usb_out_2_voltage=None if usb_2_mv is None else usb_2_mv / 1000.0,

            charger_voltage_mv=charger_mv,
            charger_voltage=None if charger_mv is None else charger_mv / 1000.0,

            raw=dict(packet),
        )


class RobotBattery:
    """
    Robot battery interface.

    The battery automatically transmits status every second.
    This class continuously reads those packets in a background thread,
    stores the newest status, and exposes clean property names.

    Example:

        battery = RobotBattery()

        print(battery.percentage)
        print(battery.temperature)
        print(battery.voltage)
        print(battery.current)
    """

    def __init__(
        self,
        device_path: Path | str | None = None,
        wait_for_first_status: bool = True,
    ):
        self.__info = BatteryInfo()
        self.__status: BatteryStatus | None = None
        self.__last_status_time: float | None = None
        self.__last_error: Exception | None = None
        self.__packet_count = 0

        self.__lock = threading.Lock()
        self.__stop_event = threading.Event()
        self.__reader_thread: threading.Thread | None = None
        self.__closed = False

        if device_path is None:
            found_device = scan_for_battery()

            if found_device is None:
                raise RuntimeError("Cannot create RobotBattery: no battery device found")

            self.device_path = found_device
        else:
            self.device_path = Path(device_path)

        self.uart_manager = UARTManager(self.device_path, BATTERY_BAUD)

        try:
            if not self.__check_if_battery():
                raise TimeoutError(
                    "Battery device was found, but no valid battery packet was received"
                )

            self.__reader_thread = threading.Thread(
                target=self.__read_forever,
                name="RobotBatteryReader",
                daemon=True,
            )
            self.__reader_thread.start()

            if wait_for_first_status and self.__status is None:
                self.wait_for_status(timeout=3.0)

        except Exception:
            self.close()
            raise

    def close(self) -> None:
        if self.__closed:
            return

        self.__closed = True
        self.__stop_event.set()

        if (
            self.__reader_thread is not None
            and self.__reader_thread is not threading.current_thread()
        ):
            self.__reader_thread.join(timeout=2.0)

        if hasattr(self, "uart_manager"):
            self.uart_manager.close()

    def __enter__(self) -> RobotBattery:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def __parse_packet(self, raw: bytes) -> dict[str, Any]:
        """
        Parse one raw battery frame.

        Handles packets like:

            {"SOC(%)":   100,"CellTemp(degK)":  298,
             "CellVoltage(mV)": 4112,"Current(mA)":-  10}

        The battery sometimes prints negative numbers as "-  10".
        """

        s = raw.decode("utf-8", "ignore")
        s = re.sub(r"\x00\s*", "", s).strip()
        s = re.sub(r"-\s+", "-", s)

        if not s:
            return {}

        # If multiple JSON-ish objects somehow end up in one frame, keep the first.
        if "}{" in s:
            s = s.split("}{", 1)[0] + "}"

        # Prefer real JSON parsing when possible.
        try:
            parsed = json.loads(s)
            if isinstance(parsed, dict):
                return parsed
        except json.JSONDecodeError:
            pass

        # Fallback parser for malformed JSON-like packets.
        if s.startswith("{") and s.endswith("}"):
            s = s[1:-1]

        packet: dict[str, Any] = {}

        for part in re.split(r",|\n", s):
            if ":" not in part:
                continue

            key, value = part.split(":", 1)

            key = key.strip().strip("'\"")
            value = value.strip().strip("'\"")

            try:
                if re.fullmatch(r"[-+]?\d+", value):
                    parsed_value: Any = int(value)
                else:
                    parsed_value = float(value)
            except ValueError:
                parsed_value = value

            packet[key] = parsed_value

        return packet

    def __consume_packet(self, packet: dict[str, Any]) -> None:
        """
        Update cached info/status from one parsed packet.
        """

        if not packet:
            return

        status = BatteryStatus.from_packet(packet)

        with self.__lock:
            if "FirmwareVersion" in packet:
                self.__info.firmware_version = str(packet.get("FirmwareVersion"))

            if "SerialNumber" in packet:
                self.__info.serial_number = str(packet.get("SerialNumber"))

            if status is not None:
                self.__status = status
                self.__last_status_time = time.monotonic()
                self.__packet_count += 1

    def __check_if_battery(self) -> bool:
        """
        Try to verify the device.

        This accepts either:
        - a FirmwareVersion/SerialNumber response after BATTERY_CMD_CHECK, or
        - a normal automatic SOC status packet.

        That makes the class work even when the battery is already streaming
        status once per second without being asked.
        """

        saw_valid_status = False
        end = time.monotonic() + BATTERY_RESPONSE_TIMEOUT

        try:
            self.uart_manager.write(BATTERY_CMD_CHECK)
        except OSError:
            return False

        while time.monotonic() < end:
            remaining = end - time.monotonic()

            try:
                raw = self.uart_manager.frame(timeout=remaining)
            except TimeoutError:
                break

            packet = self.__parse_packet(raw)
            self.__consume_packet(packet)

            if "FirmwareVersion" in packet or "SerialNumber" in packet:
                return True

            if BatteryStatus.from_packet(packet) is not None:
                saw_valid_status = True

        return saw_valid_status

    def __read_forever(self) -> None:
        """
        Background UART reader.

        This is the only code path that continuously reads the battery status.
        Properties do not touch the UART; they only return cached values.
        """

        while not self.__stop_event.is_set():
            try:
                raw = self.uart_manager.frame(timeout=1.5)
                packet = self.__parse_packet(raw)
                self.__consume_packet(packet)

            except TimeoutError:
                continue

            except OSError as exc:
                if not self.__stop_event.is_set():
                    with self.__lock:
                        self.__last_error = exc
                return

            except Exception as exc:
                # Ignore malformed packets but remember the last error.
                with self.__lock:
                    self.__last_error = exc

    def wait_for_status(self, timeout: float = 3.0) -> BatteryStatus:
        """
        Wait until at least one status packet has been received.
        """

        end = time.monotonic() + timeout

        while time.monotonic() < end:
            status = self.status

            if status is not None:
                return status

            time.sleep(0.02)

        raise TimeoutError("no battery status packet received")

    def is_status_stale(self, max_age_seconds: float = 3.0) -> bool:
        age = self.last_status_age_seconds

        if age is None:
            return True

        return age > max_age_seconds

    @property
    def info(self) -> BatteryInfo:
        with self.__lock:
            return BatteryInfo(
                firmware_version=self.__info.firmware_version,
                serial_number=self.__info.serial_number,
            )

    @property
    def status(self) -> BatteryStatus | None:
        with self.__lock:
            return self.__status

    @property
    def raw_status(self) -> dict[str, Any] | None:
        status = self.status

        if status is None or status.raw is None:
            return None

        return dict(status.raw)

    @property
    def last_error(self) -> Exception | None:
        with self.__lock:
            return self.__last_error

    @property
    def packet_count(self) -> int:
        with self.__lock:
            return self.__packet_count

    @property
    def last_status_age_seconds(self) -> float | None:
        with self.__lock:
            if self.__last_status_time is None:
                return None

            return time.monotonic() - self.__last_status_time

    @property
    def firmware_version(self) -> str | None:
        return self.info.firmware_version

    @property
    def serial_number(self) -> str | None:
        return self.info.serial_number

    @property
    def percentage(self) -> int | None:
        status = self.status
        return None if status is None else status.percentage

    @property
    def temperature(self) -> float | None:
        """
        Battery cell temperature in Celsius.
        """
        status = self.status
        return None if status is None else status.temperature_celsius

    @property
    def temperature_kelvin(self) -> float | None:
        status = self.status
        return None if status is None else status.temperature_kelvin

    @property
    def voltage(self) -> float | None:
        """
        Cell voltage in volts.
        """
        status = self.status
        return None if status is None else status.voltage

    @property
    def voltage_mv(self) -> int | None:
        status = self.status
        return None if status is None else status.voltage_mv

    @property
    def current(self) -> float | None:
        """
        Battery current in amps.

        Negative usually means discharging.
        Positive usually means charging.
        """
        status = self.status
        return None if status is None else status.current

    @property
    def current_ma(self) -> int | None:
        status = self.status
        return None if status is None else status.current_ma

    @property
    def time_to_empty_min(self) -> int | None:
        status = self.status
        return None if status is None else status.time_to_empty_min

    @property
    def cycle_count(self) -> int | None:
        status = self.status
        return None if status is None else status.cycle_count

    @property
    def usb_out_1_voltage(self) -> float | None:
        status = self.status
        return None if status is None else status.usb_out_1_voltage

    @property
    def usb_out_1_mv(self) -> int | None:
        status = self.status
        return None if status is None else status.usb_out_1_mv

    @property
    def usb_out_2_voltage(self) -> float | None:
        status = self.status
        return None if status is None else status.usb_out_2_voltage

    @property
    def usb_out_2_mv(self) -> int | None:
        status = self.status
        return None if status is None else status.usb_out_2_mv

    @property
    def charger_voltage(self) -> float | None:
        status = self.status
        return None if status is None else status.charger_voltage

    @property
    def charger_voltage_mv(self) -> int | None:
        status = self.status
        return None if status is None else status.charger_voltage_mv


def format_value(value: Any, suffix: str = "", digits: int = 2) -> str:
    if value is None:
        return "unknown"

    if isinstance(value, float):
        return f"{value:.{digits}f}{suffix}"

    return f"{value}{suffix}"
