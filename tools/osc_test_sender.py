#!/usr/bin/env python3
"""
osc_test_sender.py - pretend to be Unreal Engine. No installs needed (standard library only).

Sends /arm/target messages that trace a slow circle in front of the arm, exactly like
an Unreal actor would, so you can test the bridge + simulator (+ real arm) without
opening Unreal. It also prints the /arm/state messages the simulator sends back.

  1. npm run bridge
  2. open the simulator, Unreal Engine section -> Connect
  3. python3 tools/osc_test_sender.py            (Ctrl+C to stop)

Options: --host 127.0.0.1 --port 8000 --listen 8001 --seconds 20
"""
import argparse
import math
import socket
import struct
import time


def osc_string(s):
    b = s.encode() + b"\0"
    return b + b"\0" * (-len(b) % 4)  # pad to a multiple of 4 bytes


def osc_message(address, *floats):
    return osc_string(address) + osc_string("," + "f" * len(floats)) + b"".join(struct.pack(">f", f) for f in floats)


def osc_parse(data):
    end = data.index(b"\0")
    address = data[:end].decode()
    i = (end + 4) & ~3
    end = data.index(b"\0", i)
    tags = data[i + 1:end].decode()
    i = (end + 4) & ~3
    args = []
    for t in tags:
        if t == "f":
            args.append(struct.unpack(">f", data[i:i + 4])[0]); i += 4
        elif t == "i":
            args.append(struct.unpack(">i", data[i:i + 4])[0]); i += 4
        elif t == "s":
            end = data.index(b"\0", i); args.append(data[i:end].decode()); i = (end + 4) & ~3
    return address, args


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000, help="bridge OSC input port")
    ap.add_argument("--listen", type=int, default=8001, help="port to receive arm state on (0 = off)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N seconds (0 = run forever)")
    args = ap.parse_args()

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    inp = None
    if args.listen:
        inp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        inp.bind(("127.0.0.1", args.listen))
        inp.setblocking(False)

    print("Sending /arm/target to %s:%d ... Ctrl+C to stop" % (args.host, args.port))
    start = time.monotonic()
    last_print = 0
    try:
        while not args.seconds or time.monotonic() - start < args.seconds:
            t = time.monotonic() - start
            # Unreal coordinates relative to the arm base: cm, X forward, Y right, Z up.
            # The arm reaches ~30 cm, so this circle is about the size of a dinner plate.
            x = 18 + 4 * math.cos(t * 0.8)
            y = 6 * math.sin(t * 0.8)
            z = 14 + 3 * math.sin(t * 1.6)
            out.sendto(osc_message("/arm/target", x, y, z), (args.host, args.port))
            out.sendto(osc_message("/arm/gripper", 0.5 + 0.5 * math.sin(t * 2)), (args.host, args.port))
            if inp:
                try:
                    while True:
                        address, values = osc_parse(inp.recv(1024))
                        if address == "/arm/state" and time.monotonic() - last_print > 0.5:
                            last_print = time.monotonic()
                            print("arm state:", " ".join("%.1f" % v for v in values))
                except BlockingIOError:
                    pass
            time.sleep(1 / 30)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
