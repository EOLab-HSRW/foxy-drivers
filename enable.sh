#!/usr/bin/env bash
set -e

# Jetson Nano J41 physical pin 29 = sysfs GPIO 149
GPIO=149
GPIO_PATH="/sys/class/gpio/gpio${GPIO}"

if [ "$EUID" -ne 0 ]; then
  echo "Run this script with sudo:"
  echo "  sudo $0"
  exit 1
fi

if [ ! -d "$GPIO_PATH" ]; then
  echo "$GPIO" > /sys/class/gpio/export
  sleep 0.2
fi

echo "out" > "$GPIO_PATH/direction"
echo "Enable HAT by setting physical GPIO 29 to HIGH"
echo "Note: this need to be HIGH all the time."
echo 1 > "$GPIO_PATH/value"

cleanup() {
  echo "$GPIO" > /sys/class/gpio/unexport 2>/dev/null || true
}
trap cleanup EXIT INT TERM
