#!/usr/bin/env python3
"""MCP server for the Arduino PWM robot arm controller (4x SG90 over serial).

Same wire protocol as servo.py, but keeps one persistent serial connection
open across all tool calls instead of reopening per invocation - so unlike
servo.py, it does NOT reset the Arduino (and lose servo positions) between
calls.
"""
import glob
import math
import re
import sys
import threading
import time
from typing import Optional

import serial
from mcp.server.mcpserver import MCPServer

import ik

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
    """Record the current pose as the next step of the onboard sequence (max 99
    steps). For any multi-point path (drawing a shape, a repeating move), prefer
    this + play_sequence() over manually stepping through set_servo calls - the
    Arduino runs the whole sequence and loop on its own once started, instead of
    needing one tool call per waypoint."""
    return _send("REC")


@mcp.tool()
def undo_step() -> str:
    """Delete the last recorded step of the onboard sequence."""
    return _send("UNDO")


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


# ---------------------------------------------------------------------------
# Inverse kinematics (MeArm geometry - see ik.py)
#
# Joint-to-servo mapping for THIS arm's channel assignment:
#   S4 = base   (a0)      S3 = shoulder (a1)      S2 = elbow (a2)
#   S1 = claw, not part of the position IK.
#
# Calibration: servo_deg = ZERO + SIGN * degrees(joint_angle_rad)
#
# SCALE is fixed at 1 deg-per-deg (not a free parameter): each SG90's 0-180
# command range is, by the servo's own design and the firmware's linear
# angleToMicros() mapping, a direct linear degree scale - that part is not
# a guess.
#
# Calibration status (as of 2026-09-20):
#   - BASE: direction confirmed (S4=90 is straight-ahead / a0=0; +/-40mm
#     sideways gave a clean symmetric +/-18.4 deg split around center).
#   - SHOULDER (S3) and ELBOW (S2): both servos are base-mounted (per this
#     arm's actual MeArm-style construction: S3 drives the main upper-arm
#     bar directly, S2 drives a second parallel "control" bar that sets the
#     forearm's ABSOLUTE angle via the parallelogram linkage - S3 sits on
#     the right side of the arm and S2 on the left, viewed from behind).
#     ELBOW_ZERO went through two wrong guesses before landing here:
#       1) ZERO=90, SIGN=-1 (initial untested guess)
#       2) ZERO=180, SIGN=+1 (revised to fit the safe servo range for a
#          couple of forward-reach IK targets - looked right in one
#          combined S3+S2 photo test, but that test couldn't actually
#          distinguish the two hypotheses since both joints moved together)
#       3) ZERO=90, SIGN=+1 (this one) - found by isolating each joint:
#          moving S2 ALONE (76->110) moved the claw straight UP with
#          almost no reach change, and moving S3 ALONE (90->50, S2 fixed)
#          moved it up AND back. Both match this constant's predictions
#          quantitatively (not just directionally) via ik.forward() - see
#          git history for the worked numbers.
#   - NOT yet verified: absolute position accuracy in mm (no ruler
#     cross-check done - direction and relative magnitude look right,
#     absolute mm are still whatever L1/L2/L3 says), and the X (sideways)
#     sign specifically through move_to_xyz's solver path (base direction
#     was only tested via direct set_servo). L1/L2/L3 are still the
#     official MeArm v3.0 defaults, not measured on this specific arm.
#
# Lesson learned: a combined multi-joint test can pass "looks about right"
# even with a wrong per-joint model, because the joints' errors can partly
# cancel or align by coincidence for that one test. Isolate one joint at a
# time (hold the others fixed) when calibrating.
# ---------------------------------------------------------------------------

BASE_ZERO, BASE_SIGN = 90.0, 1.0
SHOULDER_ZERO, SHOULDER_SIGN = 90.0, -1.0
ELBOW_ZERO, ELBOW_SIGN = 90.0, 1.0

# Safe ranges for this specific arm (narrower than the servo's raw 0-180) -
# see project notes: S4 rotation 30-160, S3 height 20-90, S2 reach 70-120.
SAFE_RANGE = {4: (30, 160), 3: (20, 90), 2: (70, 120)}


def _rad_to_servo_deg(rad: float, zero: float, sign: float) -> float:
    return zero + sign * math.degrees(rad)


def _servo_deg_to_rad(deg: float, zero: float, sign: float) -> float:
    return math.radians((deg - zero) / sign)


def _check_safe_range(servo: int, angle: float) -> None:
    lo, hi = SAFE_RANGE[servo]
    if not (lo <= angle <= hi):
        raise ValueError(
            f"Computed S{servo} angle {angle:.1f} deg is outside this arm's "
            f"safe range ({lo}-{hi}). Target point is likely unreachable or "
            f"the IK calibration (ZERO/SIGN constants) needs adjusting."
        )


@mcp.tool()
def move_to_xyz(x: float, y: float, z: float) -> str:
    """Move the claw to a target position using inverse kinematics.

    Coordinate frame: origin is directly above the base rotation axis, at
    shoulder height. y = forward (mm), x = sideways (mm, sign/direction not
    yet verified against the physical arm), z = up from shoulder height (mm).
    Uses MeArm v3.0 default geometry (L1=L2=80mm, L3=22mm) - see ik.py.

    NOTE: the mapping from solved joint angles to this arm's S2/S3/S4
    servo commands uses reasoned-but-unverified calibration constants
    (see the module-level comment above this function in
    servo_mcp_server.py). Treat results with suspicion until cross-checked
    against get_status() / a visual check, especially for shoulder/elbow.
    """
    angles = ik.solve(x, y, z)
    if angles is None:
        raise ValueError(f"({x}, {y}, {z}) is not reachable by this arm's geometry")

    s4 = _rad_to_servo_deg(angles.base, BASE_ZERO, BASE_SIGN)
    s3 = _rad_to_servo_deg(angles.shoulder, SHOULDER_ZERO, SHOULDER_SIGN)
    s2 = _rad_to_servo_deg(angles.elbow, ELBOW_ZERO, ELBOW_SIGN)

    _check_safe_range(4, s4)
    _check_safe_range(3, s3)
    _check_safe_range(2, s2)

    results = [
        _send(f"S4 {round(s4)}"),
        _send(f"S3 {round(s3)}"),
        _send(f"S2 {round(s2)}"),
    ]
    return (f"-> S4={round(s4)} S3={round(s3)} S2={round(s2)}  "
            f"(solved base={math.degrees(angles.base):.1f} "
            f"shoulder={math.degrees(angles.shoulder):.1f} "
            f"elbow={math.degrees(angles.elbow):.1f} deg)")


@mcp.tool()
def get_xyz_estimate() -> str:
    """Estimate the current claw position (x, y, z mm) from the current S2/S3/S4
    readout, using forward kinematics and the same calibration as move_to_xyz.
    Subject to the same unverified-calibration caveat."""
    status = _send("?")
    found = {int(m.group(1)): float(m.group(2))
             for m in re.finditer(r"S(\d): (-?\d+) deg", status)}
    for s in (2, 3, 4):
        if s not in found:
            raise RuntimeError(f"Could not parse S{s} from status: {status!r}")

    angles = ik.JointAngles(
        base=_servo_deg_to_rad(found[4], BASE_ZERO, BASE_SIGN),
        shoulder=_servo_deg_to_rad(found[3], SHOULDER_ZERO, SHOULDER_SIGN),
        elbow=_servo_deg_to_rad(found[2], ELBOW_ZERO, ELBOW_SIGN),
    )
    x, y, z = ik.forward(angles)
    return f"x={x:.1f} y={y:.1f} z={z:.1f} mm (from S4={found[4]:.0f} S3={found[3]:.0f} S2={found[2]:.0f})"


if __name__ == "__main__":
    mcp.run()
