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

The Arduino auto-detects on `/dev/cu.usbmodem*`, `/dev/cu.usbserial*`, or
`/dev/cu.wchusbserial*` (macOS/Linux naming). Adjust `_find_port()` in
`servo_mcp_server.py` if your board enumerates differently.

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
