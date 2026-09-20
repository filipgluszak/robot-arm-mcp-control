#!/usr/bin/env python3
"""Send commands to the Arduino PWM controller.
Usage: python3 servo.py "S1 90" "WAIT 1" "S2 45" "?"
WAIT n pauses n seconds between commands (handled here, not on the Arduino).
"""
import sys, time, glob
import serial  # pip3 install pyserial

def find_port():
    ports = (glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
             + glob.glob("/dev/cu.wchusbserial*"))
    if not ports:
        sys.exit("No Arduino found. Is it plugged in and the Serial Monitor closed?")
    return ports[0]

def read_reply(s, quiet=0.3):
    out, last = b"", time.time()
    while time.time() - last < quiet:
        if s.in_waiting:
            out += s.read(s.in_waiting)
            last = time.time()
        time.sleep(0.02)
    return out.decode(errors="ignore")

with serial.Serial(find_port(), 115200, timeout=0.5) as s:
    time.sleep(2)            # the Uno restarts when the port opens
    s.reset_input_buffer()
    for cmd in sys.argv[1:]:
        if cmd.upper().startswith("WAIT"):
            time.sleep(float(cmd.split()[1]))
            continue
        s.write((cmd + "\n").encode())
        print(f">> {cmd}")
        print(read_reply(s), end="")
