#!/usr/bin/env python3
"""Minimal user-space BX100 serial recorder.

No ROS, pyserial, sudo, or receiver driver is required. The raw file keeps
the exact bytes read from the serial device; the index file adds the RK3588
host receive timestamp to each complete line.
"""

import argparse
import os
import select
import sys
import termios
import time
from pathlib import Path


BAUD = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}


def configure(fd, baud):
    if baud not in BAUD:
        raise ValueError(f"unsupported baud rate: {baud}")
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = (termios.CLOCAL | termios.CREAD | termios.CS8) & ~(
        termios.PARENB | termios.CSTOPB | termios.CRTSCTS
    )
    attrs[3] = 0
    attrs[4] = BAUD[baud]
    attrs[5] = BAUD[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


def capture(args):
    raw_path = Path(args.raw_output)
    index_path = Path(args.index_output)
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    index_path.parent.mkdir(parents=True, exist_ok=True)

    fd = os.open(args.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure(fd, args.baud)
        started_ns = time.time_ns()
        buffer = bytearray()
        deadline = time.monotonic() + args.duration if args.duration > 0 else None

        with raw_path.open("wb") as raw, index_path.open("w", encoding="utf-8") as index:
            index.write("# host_receive_time_ns\tline\n")
            index.write(f"# port={args.port} baud={args.baud} started_ns={started_ns}\n")
            print(f"recording {args.port} at {args.baud} baud")
            print(f"raw output: {raw_path}")
            print(f"index output: {index_path}")

            while deadline is None or time.monotonic() < deadline:
                timeout = 1.0 if deadline is None else max(
                    0.0, min(1.0, deadline - time.monotonic())
                )
                readable, _, _ = select.select([fd], [], [], timeout)
                if not readable:
                    continue

                chunk = os.read(fd, 8192)
                if not chunk:
                    continue
                receive_ns = time.time_ns()
                raw.write(chunk)
                raw.flush()
                buffer.extend(chunk)

                while b"\n" in buffer:
                    line, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    line = line.rstrip(b"\r")
                    index.write(
                        f"{receive_ns}\t{line.decode('ascii', errors='replace')}\n"
                    )
                index.flush()

            if buffer:
                index.write(f"{time.time_ns()}\t# incomplete_line_hex={bytes(buffer).hex()}\n")
                index.flush()
            raw.flush()
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(
        description="Record BX100 serial data without ROS or pyserial"
    )
    parser.add_argument("--port", required=True, help="for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds; 0 means until Ctrl-C")
    parser.add_argument("--raw-output", required=True)
    parser.add_argument("--index-output", required=True)
    args = parser.parse_args()
    try:
        capture(args)
    except KeyboardInterrupt:
        print("stopped", file=sys.stderr)
    except (OSError, ValueError) as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
