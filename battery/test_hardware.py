"""
This program is mainly use to test battery.py
to verify the hardware interface to the battery.
"""
import time
from typing import Any
from battery import RobotBattery


def format_value(value: Any, suffix: str = "", digits: int = 2) -> str:
    if value is None:
        return "unknown"

    if isinstance(value, float):
        return f"{value:.{digits}f}{suffix}"

    return f"{value}{suffix}"

def main() -> None:
    with RobotBattery() as battery:
        print(f"Battery device: {battery.device_path}")
        print(f"Serial number: {battery.serial_number}")
        print(f"Firmware: {battery.firmware_version}")
        print()

        while True:
            print(
            " \n ".join(
                [
                    f"SOC: {format_value(battery.percentage, '%')}",
                    f"Temp: {format_value(battery.temperature, ' °C')}",
                    f"Cell voltage: {format_value(battery.voltage, ' V')}",
                    f"Current: {format_value(battery.current, ' A')}",
                    f"Time to empty: {format_value(battery.time_to_empty_min, ' min')}",
                    f"Cycles: {format_value(battery.cycle_count)}",
                    f"USB1: {format_value(battery.usb_out_1_voltage, ' V')}",
                    f"USB2: {format_value(battery.usb_out_2_voltage, ' V')}",
                    f"Charger: {format_value(battery.charger_voltage, ' V')}",
                    ]
                )
            )

            time.sleep(1.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
