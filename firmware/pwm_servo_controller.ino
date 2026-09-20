/*
  PWM controller: 4 SG90 servos, 2.4" ST7789 TFT + EC11 encoder board, Arduino Uno
  with sequence recording and looped playback.

  Local controls:
    K0 short press       -> next servo
    Encoder short press  -> previous servo
    Turn encoder         -> move selected servo (or change speed while playing)
    Encoder long press   -> record current pose as next step (max 5)
    K0 long press        -> start / stop playback

  Serial Monitor commands (115200 baud, line ending "Newline"):
    S1 90     set servo 1 to 90 deg (S1..S4, 0..180)
    S3        select servo 3 on the screen
    S2 OFF    relax servo 2 / S2 ON re-enable it
    ALL 45    set all servos to 45 deg
    ALL OFF / ALL ON
    REC       record current pose as next step (max 5)
    LIST      show recorded steps
    CLEAR     erase the sequence
    PLAY      play the sequence in a loop
    STOP      stop playback
    SPEED 50  playback speed 10..100 %
    ?         show status
    HELP      show commands

  Libraries (Library Manager):
    - Adafruit ST7735 and ST7789 Library
    - Adafruit GFX Library
    - Servo (built in)

  Display / encoder wiring:
    GND->GND, VCC->5V, SCL->D13, SDA->D11, RES->D8, DC->D9, CS->D10, BLK->D6
    A->D2, B->D3, PUSH->D4, K0->D5

  Servo signal wires:
    Servo 1 -> A0, Servo 2 -> A1, Servo 3 -> A2, Servo 4 -> A3

  SERVO POWER: do NOT power 4 servos from the Uno's 5V pin.
  Use a separate 5V supply (at least 3A) for the servos' red wires,
  and connect that supply's GND to the Uno's GND.
*/

#include <SPI.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ---- Display / input pins ----
#define TFT_CS    10
#define TFT_DC     9
#define TFT_RST    8
#define TFT_BLK    6
#define ENC_A      2
#define ENC_B      3
#define ENC_PUSH   4
#define KEY_K0     5

// ---- Servo settings ----
#define NUM_SERVOS 4
const uint8_t SERVO_PINS[NUM_SERVOS] = {A0, A1, A2, A3};

// Pulse width at 0 and 180 deg, one value per servo (S1, S2, S3, S4).
// SG90 calibration: send "S1 0" and "S1 180". If the servo keeps buzzing
// at an end, move that end inward by 50 us (e.g. 600 -> 650, 2300 -> 2250).
// Allowed range: min 544..1050, max 1900..2400.
const int SERVO_MIN_US[NUM_SERVOS] = {600, 600, 600, 600};
const int SERVO_MAX_US[NUM_SERVOS] = {2300, 2300, 2300, 2300};

#define SLEW_US_PER_TICK  25  // manual moves: max pulse change per tick
#define PLAY_SLEW_MAX_US  40  // playback at 100 % speed (~0.85 s full sweep)
#define SLEW_TICK_MS      20
#define DEG_PER_CLICK      2  // how far one encoder click moves the servo
#define SPEED_PER_CLICK    5  // speed change per encoder click while playing
#define STEPS_PER_DETENT   4  // change to 2 if one click moves twice

// ---- Sequence settings ----
#define MAX_STEPS      5
#define DWELL_MS     300      // pause at each step before moving on
#define LONG_PRESS_MS 800

// ---- Layout ----
#define ROW_Y0   68
#define ROW_H    42
#define BAR_X   160
#define BAR_W   150
#define BAR_H    20
#define COLOR_GREY  0x7BEF
#define COLOR_DARK  0x4208

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Servo servos[NUM_SERVOS];

int  angles[NUM_SERVOS]    = {90, 90, 90, 90};   // target angle
int  currentUs[NUM_SERVOS];                      // pulse actually sent now
bool enabled[NUM_SERVOS]   = {true, true, true, true};
uint8_t current = 0;

// Sequence state
uint8_t seq[MAX_STEPS][NUM_SERVOS];
uint8_t seqLen = 0;
bool playing = false;
uint8_t playStep = 0;
uint8_t speedPct = 50;
bool dwelling = false;
unsigned long dwellStart = 0;

// ---- Encoder (interrupt driven) ----
volatile long encSteps = 0;
volatile uint8_t encState = 0;
const int8_t QUAD_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void encoderISR() {
  encState = ((encState << 2) | (digitalRead(ENC_A) << 1) | digitalRead(ENC_B)) & 0x0F;
  encSteps += QUAD_TABLE[encState];
}

long readEncoder() {
  noInterrupts();
  long s = encSteps;
  interrupts();
  return s / STEPS_PER_DETENT;
}

// ---- Buttons: short press (on release) and long press (while held) ----
#define BTN_NEXT 0           // index for the K0 key
#define BTN_PREV 1           // index for the encoder push button
#define EV_NONE  0
#define EV_SHORT 1
#define EV_LONG  2
const uint8_t btnPins[2] = {KEY_K0, ENC_PUSH};
bool btnLastState[2] = {false, false};
bool btnLongFired[2] = {false, false};
unsigned long btnLastChange[2] = {0, 0};

uint8_t buttonEvent(uint8_t i) {
  bool state = (digitalRead(btnPins[i]) == LOW);
  unsigned long now = millis();

  if (state != btnLastState[i] && now - btnLastChange[i] > 30) {
    btnLastState[i] = state;
    btnLastChange[i] = now;
    if (state) {
      btnLongFired[i] = false;               // just pressed
    } else if (!btnLongFired[i]) {
      return EV_SHORT;                        // released before long press
    }
  } else if (state && btnLastState[i] && !btnLongFired[i] &&
             now - btnLastChange[i] >= LONG_PRESS_MS) {
    btnLongFired[i] = true;
    return EV_LONG;
  }
  return EV_NONE;
}

// ---- Servo helpers ----
int angleToMicros(uint8_t i, int a) {
  return map(a, 0, 180, SERVO_MIN_US[i], SERVO_MAX_US[i]);
}

// ---- Display ----
void drawStatus() {
  char buf[28];
  if (playing) {
    snprintf(buf, sizeof(buf), "PLAY %d/%d  speed %3d%%  ", playStep + 1, seqLen, speedPct);
  } else {
    snprintf(buf, sizeof(buf), "Steps %d/%d speed %3d%%  ", seqLen, MAX_STEPS, speedPct);
  }
  tft.setTextSize(2);
  tft.setTextColor(playing ? ST77XX_GREEN : ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(4, 34);
  tft.print(buf);
}

void drawRow(uint8_t i) {
  int16_t y = ROW_Y0 + i * ROW_H;
  bool sel = (i == current);
  uint16_t fg = enabled[i] ? (sel ? ST77XX_YELLOW : ST77XX_WHITE) : COLOR_GREY;
  uint16_t barColor = enabled[i] ? (sel ? ST77XX_GREEN : ST77XX_BLUE) : COLOR_DARK;
  char buf[16];

  // "> S1  90" label + angle (or "off")
  tft.setTextSize(3);
  tft.setTextColor(fg, ST77XX_BLACK);
  tft.setCursor(4, y);
  if (enabled[i]) {
    snprintf(buf, sizeof(buf), "%cS%d %3d", sel ? '>' : ' ', i + 1, angles[i]);
  } else {
    snprintf(buf, sizeof(buf), "%cS%d off", sel ? '>' : ' ', i + 1);
  }
  tft.print(buf);

  // Position bar
  int16_t fill = (long)angles[i] * (BAR_W - 2) / 180;
  tft.drawRect(BAR_X, y, BAR_W, BAR_H, fg);
  tft.fillRect(BAR_X + 1, y + 1, fill, BAR_H - 2, barColor);
  tft.fillRect(BAR_X + 1 + fill, y + 1, BAR_W - 2 - fill, BAR_H - 2, ST77XX_BLACK);

  // Pulse width under the bar
  tft.setTextSize(1);
  tft.setTextColor(fg, ST77XX_BLACK);
  tft.setCursor(BAR_X, y + BAR_H + 3);
  snprintf(buf, sizeof(buf), "%4d us  ", angleToMicros(i, angles[i]));
  tft.print(buf);
}

void selectServo(uint8_t next) {
  uint8_t prev = current;
  current = next;
  drawRow(prev);             // un-highlight old row
  drawRow(current);          // highlight new row
}

// Sets a new target; the servo moves there smoothly (see slewUpdate)
void setAngle(uint8_t i, int a) {
  a = constrain(a, 0, 180);
  if (a == angles[i]) return;
  angles[i] = a;
  drawRow(i);
}

void enableServo(uint8_t i, bool on) {
  if (on && !enabled[i]) {
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US[i], SERVO_MAX_US[i]);
    servos[i].writeMicroseconds(currentUs[i]);
    enabled[i] = true;
  } else if (!on && enabled[i]) {
    servos[i].detach();      // stop sending pulses: servo goes limp
    enabled[i] = false;
  }
  drawRow(i);
}

// Moves each servo a small step toward its target every SLEW_TICK_MS
void slewUpdate() {
  static unsigned long lastTick = 0;
  if (millis() - lastTick < SLEW_TICK_MS) return;
  lastTick = millis();

  int step = SLEW_US_PER_TICK;
  if (playing) step = max(1, PLAY_SLEW_MAX_US * speedPct / 100);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!enabled[i]) continue;
    int target = angleToMicros(i, angles[i]);
    int c = currentUs[i];
    if (c == target) continue;
    if (target > c) c = min(c + step, target);
    else            c = max(c - step, target);
    currentUs[i] = c;
    servos[i].writeMicroseconds(c);
  }
}

// ---- Sequence ----
void setSpeed(int p) {
  speedPct = constrain(p, 10, 100);
  drawStatus();
}

void recordStep() {
  if (playing) {
    Serial.println(F("Stop playback before recording"));
    return;
  }
  if (seqLen >= MAX_STEPS) {
    Serial.println(F("Sequence full (5 steps). Use CLEAR to start over."));
    return;
  }
  for (uint8_t i = 0; i < NUM_SERVOS; i++) seq[seqLen][i] = angles[i];
  seqLen++;
  drawStatus();
  Serial.print(F("Step "));
  Serial.print(seqLen);
  Serial.println(F(" recorded"));
}

void clearSequence() {
  playing = false;
  seqLen = 0;
  drawStatus();
  Serial.println(F("Sequence cleared"));
}

void goToStep(uint8_t k) {
  playStep = k;
  dwelling = false;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) setAngle(i, seq[k][i]);
  drawStatus();
}

void startPlay() {
  if (seqLen < 2) {
    Serial.println(F("Record at least 2 steps first (REC)"));
    return;
  }
  playing = true;
  goToStep(0);
  Serial.println(F("Playing"));
}

void stopPlay() {
  if (!playing) return;
  playing = false;
  drawStatus();
  Serial.println(F("Stopped"));
}

// Advances to the next step once every servo has reached its target
void playUpdate() {
  if (!playing) return;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (enabled[i] && currentUs[i] != angleToMicros(i, angles[i])) return;
  }
  if (!dwelling) {
    dwelling = true;
    dwellStart = millis();
  } else if (millis() - dwellStart >= DWELL_MS) {
    goToStep((playStep + 1) % seqLen);
  }
}

// ---- Serial commands ----
char cmdBuf[24];
uint8_t cmdLen = 0;

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  S1 90     set servo 1 to 90 deg (S1..S4, 0..180)"));
  Serial.println(F("  S3        select servo 3 on the screen"));
  Serial.println(F("  S2 OFF    relax servo 2 / S2 ON re-enable it"));
  Serial.println(F("  ALL 45    set all servos to 45 deg"));
  Serial.println(F("  ALL OFF   relax all / ALL ON re-enable all"));
  Serial.println(F("  REC       record current pose as next step (max 5)"));
  Serial.println(F("  LIST      show recorded steps"));
  Serial.println(F("  CLEAR     erase the sequence"));
  Serial.println(F("  PLAY      play the sequence in a loop"));
  Serial.println(F("  STOP      stop playback"));
  Serial.println(F("  SPEED 50  playback speed 10..100 %"));
  Serial.println(F("  ?         show status"));
  Serial.println(F("  HELP      show this list"));
}

void printStatus() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(i == current ? F("> S") : F("  S"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(angles[i]);
    Serial.print(F(" deg, "));
    Serial.print(angleToMicros(i, angles[i]));
    Serial.print(F(" us"));
    Serial.println(enabled[i] ? F("") : F("  (OFF)"));
  }
  Serial.print(F("Steps: "));
  Serial.print(seqLen);
  Serial.print(F("/5, speed "));
  Serial.print(speedPct);
  Serial.println(playing ? F("%, PLAYING") : F("%, stopped"));
}

void printSequence() {
  if (seqLen == 0) {
    Serial.println(F("No steps recorded"));
    return;
  }
  for (uint8_t k = 0; k < seqLen; k++) {
    Serial.print(F("Step "));
    Serial.print(k + 1);
    Serial.print(F(":"));
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      Serial.print(F("  S"));
      Serial.print(i + 1);
      Serial.print(F("="));
      Serial.print(seq[k][i]);
    }
    Serial.println();
  }
}

void handleCommand(char *cmd) {
  int n, a;
  char arg[8];

  if (strcmp(cmd, "?") == 0) {
    printStatus();
  } else if (strcmp(cmd, "HELP") == 0) {
    printHelp();
  } else if (strcmp(cmd, "REC") == 0) {
    recordStep();
  } else if (strcmp(cmd, "LIST") == 0) {
    printSequence();
  } else if (strcmp(cmd, "CLEAR") == 0) {
    clearSequence();
  } else if (strcmp(cmd, "PLAY") == 0) {
    startPlay();
  } else if (strcmp(cmd, "STOP") == 0) {
    stopPlay();
  } else if (sscanf(cmd, "SPEED %d", &a) == 1) {
    setSpeed(a);
    Serial.print(F("Speed "));
    Serial.print(speedPct);
    Serial.println(F("%"));
  } else if (strcmp(cmd, "ALL OFF") == 0) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) enableServo(i, false);
    printStatus();
  } else if (strcmp(cmd, "ALL ON") == 0) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) enableServo(i, true);
    printStatus();
  } else if (sscanf(cmd, "ALL %d", &a) == 1) {
    stopPlay();
    for (uint8_t i = 0; i < NUM_SERVOS; i++) setAngle(i, a);
    printStatus();
  } else {
    int count = sscanf(cmd, "S%d %7s", &n, arg);
    if (count >= 1 && n >= 1 && n <= NUM_SERVOS) {
      uint8_t i = n - 1;
      if (count == 1) {
        selectServo(i);
        Serial.print(F("Selected S"));
        Serial.println(n);
      } else if (strcmp(arg, "OFF") == 0) {
        enableServo(i, false);
        Serial.print(F("S")); Serial.print(n); Serial.println(F(" OFF"));
      } else if (strcmp(arg, "ON") == 0) {
        enableServo(i, true);
        Serial.print(F("S")); Serial.print(n); Serial.println(F(" ON"));
      } else {
        stopPlay();
        setAngle(i, atoi(arg));
        Serial.print(F("S")); Serial.print(n);
        Serial.print(F(" -> ")); Serial.print(angles[i]); Serial.println(F(" deg"));
      }
    } else {
      Serial.print(F("Unknown command: "));
      Serial.println(cmd);
      printHelp();
    }
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = toupper(c);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_PUSH, INPUT_PULLUP);
  pinMode(KEY_K0, INPUT_PULLUP);

  encState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);

  // Servos start at their stored angle
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    currentUs[i] = angleToMicros(i, angles[i]);
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US[i], SERVO_MAX_US[i]);
    servos[i].writeMicroseconds(currentUs[i]);
  }

  tft.init(240, 320);
  tft.setRotation(3);        // landscape, knob and buttons on the right
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextSize(3);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(34, 4);
  tft.print("PWM controller");
  drawStatus();
  tft.drawFastHLine(0, 57, 320, ST77XX_BLUE);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) drawRow(i);

  Serial.println(F("PWM controller ready"));
  printHelp();
}

long lastEncPos = 0;

void loop() {
  readSerial();

  // K0: short = next servo, long = play / stop
  uint8_t evNext = buttonEvent(BTN_NEXT);
  if (evNext == EV_SHORT) selectServo((current + 1) % NUM_SERVOS);
  if (evNext == EV_LONG)  { if (playing) stopPlay(); else startPlay(); }

  // Encoder push: short = previous servo, long = record step
  uint8_t evPrev = buttonEvent(BTN_PREV);
  if (evPrev == EV_SHORT) selectServo((current + NUM_SERVOS - 1) % NUM_SERVOS);
  if (evPrev == EV_LONG)  recordStep();

  // Encoder: move selected servo, or change speed while playing
  long pos = readEncoder();
  long delta = pos - lastEncPos;
  if (delta != 0) {
    lastEncPos = pos;
    if (playing) setSpeed(speedPct + (int)delta * SPEED_PER_CLICK);
    else         setAngle(current, angles[current] + (int)delta * DEG_PER_CLICK);
  }

  slewUpdate();
  playUpdate();
}