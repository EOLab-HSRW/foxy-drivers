# foxy-drivers

Low-level drivers and daemon helpers for Foxy robot hardware.

## Installed components

- `/usr/local/include/foxy/foxy.h` — single-header C hardware driver.
- `/usr/local/include/foxy/batteryd.h` — header-only C client for the battery daemon.
- `/usr/local/include/foxy/battery.h` — compatibility wrapper for `batteryd.h`.
- `/opt/batteryd/battery.py` and `/opt/batteryd/batteryd.py` — battery daemon implementation.
- `/usr/local/sbin/foxy-hat-enable` — HAT GPIO mapping helper.
- `foxy-hat-enable.service` — enables the HAT mapping before `batteryd.service`.
- `batteryd.service` — publishes `/run/batteryd/status` and `/run/batteryd/control.sock`.
- `foxy-bringup.service` — starts the ROS 2 hardware bringup in the `bringup` tmux session.

## HAT GPIO enable logic

`foxy-hat-enable` defaults to `FOXY_HAT_ENABLE=auto`. In auto mode it enables the
HAT mapping only when Linux device-tree data identifies a Jetson Nano platform.

Environment overrides:

- `FOXY_HAT_ENABLE=true` forces the GPIO enable path.
- `FOXY_HAT_ENABLE=false` disables the helper and exits successfully.
- `FOXY_HAT_ENABLE_GPIO=<number>` overrides the sysfs GPIO number. The default is
  `149`, which is Jetson Nano J41 physical pin 29.

The helper intentionally leaves the GPIO exported and driven high because the HAT
mapping must remain enabled while the robot is running.
