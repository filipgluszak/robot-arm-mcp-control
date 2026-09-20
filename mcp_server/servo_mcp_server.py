#!/usr/bin/env python3
"""MCP server for the Arduino PWM robot arm controller (4x SG90 over serial).

Same wire protocol as servo.py, but keeps one persistent serial connection
open across all tool calls instead of reopening per invocation - so unlike
servo.py, it does NOT reset the Arduino (and lose servo positions) between
calls.
"""
import glob
import sys
import threading
import time
from typing import Optional

import serial
from mcp.server.mcpserver import MCPServer

mcp = MCPServer("robot-arm")

_lock = threading.Lock()
_ser: Optional[serial.Serial] = None

BAUD = 115200


def _find_port() -> str:
    ports = (glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
             + glob.glob("/dev/cu.wchusbserial*"))
    if not ports:
        raise RuntimeError("No Arduino found. Is it plugged in and the Serial Monitor closed?")
    return ports[0]


def _connect() -> serial.Serial:
    global _ser
    if _ser is not None and _ser.is_open:
        return _ser
    port = _find_port()
    _ser = serial.Serial(port, BAUD, timeout=0.5)
    time.sleep(2)  # the Uno resets when the port opens; give it time to boot
    _ser.reset_input_buffer()
    return _ser


def _read_reply(s: serial.Serial, quiet: float = 0.3) -> str:
    out, last = b"", time.time()
    while time.time() - last < quiet:
        if s.in_waiting:
            out += s.read(s.in_waiting)
            last = time.time()
        time.sleep(0.02)
    return out.decode(errors="ignore").strip()


def _send(cmd: str) -> str:
    with _lock:
        try:
            s = _connect()
            s.write((cmd + "\n").encode())
            return _read_reply(s) or "(no reply)"
        except (serial.SerialException, OSError) as e:
            global _ser
            _ser = None  # force reconnect on next call
            raise RuntimeError(f"Serial error, will reconnect next call: {e}")


def _check_servo(servo: int) -> None:
    if servo not in (1, 2, 3, 4):
        raise ValueError("servo must be 1, 2, 3, or 4")


def _check_angle(angle: int) -> None:
    if not (0 <= angle <= 180):
        raise ValueError("angle must be 0-180")


@mcp.tool()
def set_servo(servo: int, angle: int) -> str:
    """Set one servo (1-4) to an angle in degrees (0-180)."""
    _check_servo(servo)
    _check_angle(angle)
    return _send(f"S{servo} {angle}")


@mcp.tool()
def set_all(angle: int) -> str:
    """Set all 4 servos to the same angle in degrees (0-180)."""
    _check_angle(angle)
    return _send(f"ALL {angle}")


@mcp.tool()
def set_servo_power(servo: int, on: bool) -> str:
    """Relax (on=false) or re-enable (on=true) a single servo (1-4)."""
    _check_servo(servo)
    return _send(f"S{servo} {'ON' if on else 'OFF'}")


@mcp.tool()
def set_all_power(on: bool) -> str:
    """Relax (on=false) or re-enable (on=true) all 4 servos at once."""
    return _send(f"ALL {'ON' if on else 'OFF'}")


@mcp.tool()
def record_step() -> str:
    """Record the current pose as the next step of the onboard sequence (max 5 steps)."""
    return _send("REC")


@mcp.tool()
def list_steps() -> str:
    """List the recorded sequence steps and playback state."""
    return _send("LIST")


@mcp.tool()
def clear_sequence() -> str:
    """Erase the recorded sequence."""
    return _send("CLEAR")


@mcp.tool()
def play_sequence() -> str:
    """Play the recorded sequence in a loop."""
    return _send("PLAY")


@mcp.tool()
def stop_sequence() -> str:
    """Stop sequence playback."""
    return _send("STOP")


@mcp.tool()
def set_speed(percent: int) -> str:
    """Set sequence playback speed, 10-100 percent."""
    if not (10 <= percent <= 100):
        raise ValueError("percent must be 10-100")
    return _send(f"SPEED {percent}")


@mcp.tool()
def get_status() -> str:
    """Query current servo positions (angle + pulse width) and playback status."""
    return _send("?")


@mcp.tool()
def wait_seconds(seconds: float) -> str:
    """Pause for the given number of seconds. Useful for pacing a move sequence
    between other tool calls (the Arduino itself does not support delays)."""
    if seconds < 0 or seconds > 30:
        raise ValueError("seconds must be between 0 and 30")
    time.sleep(seconds)
    return f"waited {seconds}s"


@mcp.tool()
def raw_command(command: str) -> str:
    """Send a raw command string directly to the controller (escape hatch for
    anything not covered by the other tools, e.g. future firmware commands)."""
    return _send(command)


if __name__ == "__main__":
    mcp.run()
