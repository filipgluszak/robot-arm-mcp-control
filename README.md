# robot-arm-mcp-control

Control a 4-DOF SG90 servo robot arm (Arduino Uno) from an LLM agent via the
[Model Context Protocol](https://modelcontextprotocol.io), or from the command
line.

Three layers:

- **`firmware/pwm_servo_controller.ino`** — Arduino sketch. Drives 4 SG90
  servos, plus an optional 2.4" ST7789 TFT + rotary encoder for local manual
  control, and a small onboard sequence recorder (record up to 5 poses,
  play them back in a loop). Talks over USB serial at 115200 baud.
- **`servo.py`** — minimal CLI: `python3 servo.py "S1 90" "WAIT 1" "S2 45" "?"`.
  Simple, but it reopens the serial port on every invocation, which resets
  the Arduino (Uno auto-resets on DTR) and loses whatever pose the arm was
  in.
- **`mcp_server/servo_mcp_server.py`** — an MCP server exposing the same
  command set as typed tools (`set_servo`, `get_status`, etc.) to any MCP
  client (Claude Code, Claude Desktop, ...). Unlike `servo.py`, it keeps one
  serial connection open for the life of the server process, so the Arduino
  is not reset between calls.

## Hardware

- Arduino Uno
- 4x SG90 micro servos, signal wires on `A0`-`A3`
- Optional: 2.4" ST7789 SPI TFT + EC11 rotary encoder + push button, for
  local on-device control (see pin mapping in the `.ino` header comment)
- **A separate 5V supply for the servos** (3A+), with its ground tied to the
  Arduino's ground. Do not power 4 servos from the Uno's own 5V pin.

## Firmware

Open `firmware/pwm_servo_controller.ino` in the Arduino IDE. Requires, via
Library Manager:

- Adafruit GFX Library
- Adafruit ST7735 and ST7789 Library
- Servo (built in)

Flash it to the Uno. Serial commands (115200 baud, "Newline" line ending):

```
S1 90     set servo 1 to 90 deg (S1..S4, 0..180)
S3        select servo 3 on the screen
S2 OFF    relax servo 2 / S2 ON re-enable it
ALL 45    set all servos to 45 deg
ALL OFF   relax all / ALL ON re-enable all
REC       record current pose as next step (max 5)
LIST      show recorded steps
CLEAR     erase the sequence
PLAY      play the sequence in a loop
STOP      stop playback
SPEED 50  playback speed 10..100 %
?         show status
HELP      show this list
```

Per-servo pulse-width calibration (`SERVO_MIN_US` / `SERVO_MAX_US` in the
sketch) may need tuning per servo — see the comment above those constants.

## CLI usage

```bash
pip3 install pyserial
python3 servo.py "S1 90" "WAIT 1" "ALL 45" "?"
```

`WAIT n` is handled client-side (a plain `time.sleep`), not sent to the
board. Every invocation opens a fresh serial connection, which resets the
Arduino — chain everything you need into one invocation rather than calling
the script repeatedly if you want to preserve pose between commands.

## MCP server usage

```bash
cd mcp_server
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt
```

Copy `.mcp.json.example` to `.mcp.json` at your project root (or merge it
into an existing one) and fix the two paths to point at your checkout:

```json
{
  "mcpServers": {
    "robot-arm": {
      "command": "/absolute/path/to/robot-arm-mcp-control/mcp_server/.venv/bin/python3",
      "args": ["/absolute/path/to/robot-arm-mcp-control/mcp_server/servo_mcp_server.py"]
    }
  }
}
```

Restart your MCP client (e.g. Claude Code) so it picks up the new server.
Tools exposed:

| Tool | Purpose |
|---|---|
| `set_servo(servo, angle)` | Move one servo (1-4) to an angle 0-180° |
| `set_all(angle)` | Move all 4 servos to the same angle |
| `set_servo_power(servo, on)` | Relax / re-enable a single servo |
| `set_all_power(on)` | Relax / re-enable all 4 servos |
| `record_step()` | Record current pose as the next onboard sequence step (max 5) |
| `list_steps()` | List recorded sequence steps + playback state |
| `clear_sequence()` | Erase the recorded sequence |
| `play_sequence()` | Loop-play the recorded sequence |
| `stop_sequence()` | Stop playback |
| `set_speed(percent)` | Playback speed, 10-100% |
| `get_status()` | Angle + pulse width (µs) for all 4 servos, playback state |
| `wait_seconds(seconds)` | Pause up to 30s, for pacing a move sequence |
| `raw_command(command)` | Escape hatch — send any raw string to the firmware |
| `move_to_xyz(x, y, z)` | Move the claw to a target position via inverse kinematics (see below) |
| `get_xyz_estimate()` | Estimate current claw position from current servo readout (forward kinematics) |

The Arduino auto-detects on `/dev/cu.usbmodem*`, `/dev/cu.usbserial*`, or
`/dev/cu.wchusbserial*` (macOS/Linux naming). Adjust `_find_port()` in
`servo_mcp_server.py` if your board enumerates differently.

## Inverse kinematics

`mcp_server/ik.py` is a Python port of the official
[MeArm-Arduino](https://github.com/MeArm/MeArm-Arduino) solver — this arm's
physical design (and most AliExpress clones of it) is the open-source MeArm,
a 4-bar parallelogram arm. `solve(x, y, z)` returns the three joint angles
(base, shoulder, elbow) needed to reach a point, in a coordinate frame with
the origin directly above the base rotation axis, at shoulder height (`y`
forward, `x` sideways, `z` up), using the official v3.0 link lengths
(`L1=L2=80mm`, `L3=22mm`). Verified self-consistent via forward/inverse
round-trip tests.

**Calibration status** (last verified 2026-09-20 against the physical arm):
`move_to_xyz` converts the solver's joint angles (radians) into this arm's
`S2`(elbow)/`S3`(shoulder)/`S4`(base) servo commands using per-axis
`ZERO`/`SIGN` constants at the top of the IK section in
`servo_mcp_server.py`. The *scale* (degrees per radian) is not a guess —
each SG90's 0-180° command range is linearly 1:1 with physical degrees by
design.

- **Base**: direction confirmed — `S4=90` is straight-ahead (`a0=0`), and
  ±40mm sideways gave a clean symmetric ±18.4° split around center.
- **Shoulder (S3) / elbow (S2)**: both servos are base-mounted, per this
  arm's actual construction — S3 directly drives the main upper-arm bar,
  and S2 drives a second parallel "control" bar that sets the forearm's
  *absolute* angle via the parallelogram linkage (S3 on the right side of
  the arm, S2 on the left, viewed from behind). `ELBOW_ZERO`/`SIGN` took
  three attempts to get right — worth reading if you hit a similar wall:
  1. `ZERO=90, SIGN=-1` — untested initial guess.
  2. `ZERO=180, SIGN=+1` — revised to fit the safe servo range for a couple
     of IK targets, and *appeared* confirmed by a combined S3+S2
     before/after photo test. It wasn't actually confirmed — that test
     moved both joints at once, so a wrong elbow model could still produce
     a plausible-looking result if the shoulder's (correct) contribution
     dominated what the photo showed.
  3. `ZERO=90, SIGN=+1` (current) — found by isolating each joint: moving
     S2 *alone* moved the claw straight up with almost no reach change;
     moving S3 *alone* (S2 held fixed) moved it up and back. Both match
     this constant's predictions quantitatively, not just directionally.
  **Lesson**: a multi-joint test can look right with a wrong per-joint
  model, because errors can partly cancel for that one test. Isolate one
  joint at a time (hold the others fixed) when calibrating a multi-DOF arm.
- **Not yet verified**: absolute position accuracy in mm (no ruler
  cross-check done — direction and relative magnitude check out, absolute
  mm are still whatever `L1`/`L2`/`L3` says), and the `x` (sideways) sign
  specifically through `move_to_xyz`'s solver path (only the base servo's
  direction was tested directly, via `set_servo`). `L1`/`L2`/`L3` are still
  the official MeArm v3.0 defaults (80/80/22mm), not measured on this arm.

To finish calibrating:

1. Call `move_to_xyz` for a couple of distinct, unambiguous target points
   (e.g. straight up-and-close vs. far-and-low) and photograph or watch
   each pose after it fully settles — comparing motion blur mid-move gives
   misleading results.
2. If a joint moves the wrong direction, flip that axis's `SIGN` constant.
   If it's moving the right direction but consistently offset, adjust that
   axis's `ZERO` constant — see the 2026-09-20 note above for what a wrong
   `ZERO` typically looks like (angles landing far outside the safe range).
3. Re-measure `L1`/`L2`/`L3` with a ruler (shoulder-pivot to elbow-pivot,
   elbow-pivot to wrist-pivot, etc. — see `Geometry.md` in the MeArm repo for
   exactly which points to measure between) once direction is confirmed but
   absolute distance is off, and update the constants at the top of `ik.py`.

## Notes from actually using it

- SG90s are cheap and imprecise — treat the 0-180° range as nominal, not
  exact, and calibrate `SERVO_MIN_US`/`SERVO_MAX_US` per servo if one buzzes
  at an end of its range.
- If you assign this arm a role per servo (e.g. claw / reach / height /
  rotation), the safe range per role is usually narrower than 0-180° and is
  specific to your build — the MCP server does not currently enforce that,
  it only validates the hardware's own 0-180°/10-100% bounds.
- A servo that accepts commands (firmware echoes the new angle) but doesn't
  visibly move is worth checking for current draw at the power supply — a
  climbing/stalled current with no motion usually means a mechanical bind,
  not a wiring or firmware problem.
