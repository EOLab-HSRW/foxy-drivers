#!/usr/bin/env python3
import argparse
import socket
import sys
from pathlib import Path


DEFAULT_PATH = "/run/batteryd/status"


def read_status(path: str, timeout: float = 2.0) -> str:
    """
    Read one status payload from a Unix domain socket.

    Falls back to normal file read if the path is not a socket-like endpoint.
    """
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect(path)

            chunks = []
            while True:
                data = s.recv(4096)
                if not data:
                    break
                chunks.append(data)

            return b"".join(chunks).decode("utf-8", errors="replace").strip()

    except OSError as socket_error:
        # Useful because the example uses `cat /run/batteryd/status`.
        # If `cat` works, this may be a regular file, procfs/sysfs-style file, or FIFO.
        try:
            return Path(path).read_text(encoding="utf-8", errors="replace").strip()
        except OSError:
            raise socket_error


def parse_status(payload: str) -> dict:
    """
    Parse protocol format:

        online=1 stale=0 percent=45 voltage_mv=3575 ...

    Returns a dict with int values when possible.
    """
    status = {}

    for field in payload.split():
        if "=" not in field:
            raise ValueError(f"bad field without '=': {field!r}")

        key, value = field.split("=", 1)

        if not key:
            raise ValueError(f"empty key in field: {field!r}")

        try:
            value = int(value, 10)
        except ValueError:
            pass

        status[key] = value

    return status


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Minimal test client for batteryd status protocol"
    )
    parser.add_argument(
        "path",
        nargs="?",
        default=DEFAULT_PATH,
        help=f"Unix socket or status file path, default: {DEFAULT_PATH}",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="socket timeout in seconds",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="print raw daemon output only",
    )
    parser.add_argument(
        "--fail-stale",
        action="store_true",
        help="exit nonzero if stale=1",
    )
    parser.add_argument(
        "--fail-offline",
        action="store_true",
        help="exit nonzero if online=0",
    )

    args = parser.parse_args()

    try:
        payload = read_status(args.path, args.timeout)

        if args.raw:
            print(payload)
            return 0

        status = parse_status(payload)

    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    # Compact parsed output
    for key in sorted(status):
        print(f"{key}={status[key]}")

    if args.fail_stale and status.get("stale") == 1:
        print("ERROR: status is stale", file=sys.stderr)
        return 1

    if args.fail_offline and status.get("online") == 0:
        print("ERROR: battery daemon reports offline", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

