#!/usr/bin/env python3
"""Run a host-side Servo2040 protocol fake for Android emulator tests."""

from __future__ import annotations

import argparse
import select
import socket

from servo2040_protocol_emulator import Servo2040ProtocolEmulator
from run_servo2040_socket_fake import exchange_complete


def run(host: str, port: int, verbose: bool) -> None:
    emu = Servo2040ProtocolEmulator()
    emu.set_analog(
        touches=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        voltage=7.4,
        current_amps=2.6,
    )
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((host, port))
        listener.listen(1)
        print(f"servo2040 tcp fake listening on {host}:{port}", flush=True)
        while True:
            conn, addr = listener.accept()
            with conn:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"servo2040 tcp fake connected from {addr[0]}:{addr[1]}", flush=True)
                pending = bytearray()
                while True:
                    readable, _, _ = select.select([conn], [], [], 0.1)
                    if not readable:
                        continue
                    chunk = conn.recv(4096)
                    if not chunk:
                        print("servo2040 tcp fake disconnected", flush=True)
                        break
                    pending.extend(chunk)
                    request, reply = exchange_complete(emu, pending)
                    if verbose and request:
                        print(f"rx={request.hex()} tx={reply.hex()}", flush=True)
                    if reply:
                        conn.sendall(reply)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18712)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    run(args.host, args.port, args.verbose)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
