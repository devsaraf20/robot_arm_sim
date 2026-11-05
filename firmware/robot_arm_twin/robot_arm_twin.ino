/*
 * robot_arm_twin.ino — digital-twin firmware for a 4-servo desktop arm.
 *
 * Talks to the browser simulator over USB serial (Web Serial) using the plain-text
 * protocol in src/protocol.js. Type commands in the Serial Monitor (115200 baud,
 * "Newline") to test without the browser:
 *
 *   ?              hello              W           write mode (browser drives arm)
 *   T a b c d      move (servo deg)   R           read mode  (arm drives browser)
 *   V 60           max speed deg/s    H           homing routine
 *   E              EMERGENCY STOP     C           clear e-stop
 *   K              auto-calibrate feedback (feedback servos only)
 *   L j a          learn calibration point: joint j is now at a degrees (pot arms)
 *
 * Angles are LOGICAL servo degrees: base, shoulder, elbow 0..180 and gripper 0..90.
 * (logical = simulator angle - joint minimum; see src/ik.js.)
 *
 * SAFETY (enforced here even if the browser misbehaves):
 *   - every target is clamped to SAFE_MIN..SAFE_MAX
 *   - joints never move faster than maxSpeed (and never above FW_MAX_SPEED)
 *   - E-stop (command E or the button on D2) stops sending pulses to the servos
 *     and stays latched until the host sends C
 *   - if the host stops talking for HOST_TIMEOUT_MS the arm holds where it is
 *   - on power-up the arm moves to HOME slowly (with feedback it starts from where
 *     it actually is, so there is no sudden jump)
 *
 * Wiring and parts: see docs/HARDWARE.md.
 */
#include <Servo.h>
#include <EEPROM.h>

// ============================== CONFIGURATION ==============================
#define JOINTS 4
#define HAS_FEEDBACK 0      // 1 = analog position feedback wired to A0..A3
#define PARALLEL_LINKAGE 0  // 1 = MeArm-style arm where the elbow servo sets the
                            //     forearm angle relative to the GROUND (see HARDWARE.md)
#define FW_VERSION "1.0"

const uint8_t SERVO_PINS[JOINTS]    = {9, 10, 11, 6};     // base, shoulder, elbow, gripper
const uint8_t FEEDBACK_PINS[JOINTS] = {A0, A1, A2, A3};
const uint8_t ESTOP_PIN = 2;        // push button to GND (uses internal pull-up)
const uint8_t LED_PIN = LED_BUILTIN;

// Safety limits in logical degrees. Start wide, then TIGHTEN them for your arm so it
// can never hit the table or itself.
const float SAFE_MIN[JOINTS] = {0, 0, 0, 0};
const float SAFE_MAX[JOINTS] = {180, 180, 180, 90};
const float HOME_POSE[JOINTS] = {90, 70, 50, 45};  // = simulator home (0, 70, -100, 45)

const float FW_MAX_SPEED = 180;   // deg/s, hard cap no matter what the host asks for
const float HOMING_SPEED = 30;    // deg/s
const unsigned long HOST_TIMEOUT_MS = 1000;

// Per-servo mounting fixes (applied only to the PWM output).
const bool  INVERT[JOINTS] = {false, false, false, false};
const float TRIM[JOINTS]   = {0, 0, 0, 0};
const int   PULSE_MIN_US = 500;   // pulse at 0 deg  (many servos: 1000)
const int   PULSE_MAX_US = 2500;  // pulse at 180 deg (many servos: 2000)

const unsigned long TICK_MS = 20;    // motion update: 50 Hz
const unsigned long REPORT_MS = 40;  // position report: 25 Hz
// ===========================================================================

Servo servos[JOINTS];
float current[JOINTS];   // where we are commanding the servos right now (logical deg)
float target[JOINTS];    // where we are heading
float maxSpeed = 90;
bool estop = false;
bool homing = false;
bool attached = false;
enum Mode { MODE_WRITE, MODE_READ } mode = MODE_WRITE;
unsigned long lastHostMs = 0;

// ---- Feedback calibration: two (raw ADC, degrees) points per joint, kept in EEPROM ----
struct Calibration {
  uint16_t magic;
  int16_t rawA[JOINTS], rawB[JOINTS];
  float degA[JOINTS], degB[JOINTS];
};
const uint16_t CAL_MAGIC = 0xA12B;
Calibration cal;

void loadCalibration() {
  EEPROM.get(0, cal);
  if (cal.magic != CAL_MAGIC) {  // never calibrated: rough defaults
    cal.magic = CAL_MAGIC;
    for (uint8_t i = 0; i < JOINTS; i++) { cal.rawA[i] = 100; cal.degA[i] = 0; cal.rawB[i] = 900; cal.degB[i] = 180; }
  }
}
void saveCalibration() { EEPROM.put(0, cal); }

// ---- Linkage helpers ------------------------------------------------------
// With a parallel linkage the elbow servo's position depends on the shoulder too.
// "out" = what the elbow servo must do; "in" = turn a measured servo value back.
float linkOut(uint8_t i, float logical, float shoulder) {
#if PARALLEL_LINKAGE
  if (i == 2) return logical + shoulder - 90;
#endif
  (void)shoulder; (void)i;
  return logical;
}
float linkIn(uint8_t i, float p, float shoulder) {
#if PARALLEL_LINKAGE
  if (i == 2) return p - shoulder + 90;
#endif
  (void)shoulder; (void)i;
  return p;
}

// ---- Output ---------------------------------------------------------------
void writeServo(uint8_t i, float logical) {
  float h = linkOut(i, logical, current[1]);
  if (INVERT[i]) h = 180 - h;
  h = constrain(h + TRIM[i], 0, 180);
  int us = PULSE_MIN_US + (long)(h * 100) * (PULSE_MAX_US - PULSE_MIN_US) / 18000L;
  servos[i].writeMicroseconds(us);
}

void attachAll() {
  if (attached) return;
  // Write the position BEFORE attaching so the very first pulse is the right one.
  for (uint8_t i = 0; i < JOINTS; i++) { writeServo(i, current[i]); servos[i].attach(SERVO_PINS[i]); }
  attached = true;
}
void detachAll() {
  for (uint8_t i = 0; i < JOINTS; i++) servos[i].detach();  // no pulses = servo goes limp
  attached = false;
}

// ---- Input ----------------------------------------------------------------
float readRaw(uint8_t i) {
  long sum = 0;
  for (uint8_t n = 0; n < 4; n++) sum += analogRead(FEEDBACK_PINS[i]);
  return sum / 4.0;
}
// Measured logical angles (only meaningful with HAS_FEEDBACK).
void readMeasured(float out[JOINTS]) {
  for (uint8_t i = 0; i < JOINTS; i++) {
    float raw = readRaw(i);
    float span = cal.rawB[i] - cal.rawA[i];
    float p = span == 0 ? cal.degA[i] : cal.degA[i] + (raw - cal.rawA[i]) * (cal.degB[i] - cal.degA[i]) / span;
    out[i] = linkIn(i, p, i > 1 ? out[1] : 0);  // shoulder (i=1) is computed before elbow
  }
}

// ---- Safety ---------------------------------------------------------------
float clampJoint(uint8_t i, float v) { return constrain(v, SAFE_MIN[i], SAFE_MAX[i]); }

void triggerEstop(const char *why) {
  if (estop) return;
  estop = true;
  homing = false;
  detachAll();
  for (uint8_t i = 0; i < JOINTS; i++) target[i] = current[i];
  Serial.print(F("S ESTOP ")); Serial.println(why);
}

// Start from the REAL position when we can measure it, otherwise from the last command.
void syncToActual() {
#if HAS_FEEDBACK
  readMeasured(current);
  for (uint8_t i = 0; i < JOINTS; i++) current[i] = clampJoint(i, current[i]);
#endif
  for (uint8_t i = 0; i < JOINTS; i++) target[i] = current[i];
}

void startHoming() {
  if (estop) { Serial.println(F("! e-stop active: send C first")); return; }
  if (mode == MODE_READ) { mode = MODE_WRITE; syncToActual(); }
  attachAll();
  for (uint8_t i = 0; i < JOINTS; i++) target[i] = HOME_POSE[i];
  homing = true;
  Serial.println(F("S HOMING"));
}

// Move every joint a little towards its target, never faster than the speed limit.
bool slewStep(float speed, float dtSeconds) {
  float maxStep = speed * dtSeconds;
  bool moving = false;
  for (uint8_t i = 0; i < JOINTS; i++) {
    float d = target[i] - current[i];
    if (fabs(d) > 0.05) moving = true;
    current[i] += constrain(d, -maxStep, maxStep);
    writeServo(i, current[i]);
  }
  return moving;
}

// Blocking move used by calibration. Still honours the e-stop button and 'E'.
bool blockingMove(const float pose[JOINTS], float speed) {
  for (uint8_t i = 0; i < JOINTS; i++) target[i] = clampJoint(i, pose[i]);
  while (true) {
    if (digitalRead(ESTOP_PIN) == LOW) { triggerEstop("button"); return false; }
    if (Serial.available() && Serial.peek() == 'E') { Serial.read(); triggerEstop("host"); return false; }
    if (!slewStep(speed, TICK_MS / 1000.0)) return true;
    delay(TICK_MS);
  }
}

void autoCalibrate() {
#if HAS_FEEDBACK
  if (estop || mode != MODE_WRITE) { Serial.println(F("! calibrate needs write mode, no e-stop")); return; }
  Serial.println(F("S CAL START"));
  attachAll();
  float pose[JOINTS];
  for (uint8_t i = 0; i < JOINTS; i++) pose[i] = HOME_POSE[i];
  if (!blockingMove(pose, HOMING_SPEED)) return;
  for (uint8_t j = 0; j < JOINTS; j++) {
    float a = SAFE_MIN[j] + 15, b = SAFE_MAX[j] - 15;
    pose[j] = a;
    if (!blockingMove(pose, HOMING_SPEED)) return;
    delay(400);
    cal.rawA[j] = readRaw(j); cal.degA[j] = linkOut(j, a, current[1]);
    pose[j] = b;
    if (!blockingMove(pose, HOMING_SPEED)) return;
    delay(400);
    cal.rawB[j] = readRaw(j); cal.degB[j] = linkOut(j, b, current[1]);
    pose[j] = HOME_POSE[j];
    if (!blockingMove(pose, HOMING_SPEED)) return;
    Serial.print(F("# joint ")); Serial.print(j); Serial.print(F(" raw "));
    Serial.print(cal.rawA[j]); Serial.print(F("..")); Serial.println(cal.rawB[j]);
  }
  saveCalibration();
  Serial.println(F("S CAL DONE"));
#else
  Serial.println(F("! no feedback: set HAS_FEEDBACK 1"));
#endif
}

// Potentiometer / leader arms: "L j angle" = joint j is physically at `angle` now.
// Learn one low (<90) and one high (>=90) point per joint.
void learnPoint(int j, float angle) {
#if HAS_FEEDBACK
  if (j < 0 || j >= JOINTS) return;
  float measured[JOINTS];
  readMeasured(measured);
  float p = linkOut(j, angle, measured[1]);
  int raw = readRaw(j);
  if (angle < 90) { cal.rawA[j] = raw; cal.degA[j] = p; } else { cal.rawB[j] = raw; cal.degB[j] = p; }
  saveCalibration();
  Serial.print(F("# learned joint ")); Serial.print(j); Serial.print(F(" raw ")); Serial.print(raw);
  Serial.print(F(" = ")); Serial.println(angle);
#else
  (void)j; (void)angle;
  Serial.println(F("! no feedback: set HAS_FEEDBACK 1"));
#endif
}

// ---- Command handling -------------------------------------------------------
void hello() {
  Serial.print(F("HELLO robot-arm-twin " FW_VERSION " "));
  Serial.print(JOINTS); Serial.print(' '); Serial.println(HAS_FEEDBACK);
  Serial.println(estop ? F("S ESTOP") : mode == MODE_READ ? F("S MODE R") : homing ? F("S HOMING") : F("S READY"));
}

void handleLine(char *line) {
  char *tok = strtok(line, " ");
  if (!tok) return;
  char c = tok[0];
  lastHostMs = millis();
  switch (c) {
    case '?': hello(); break;
    case 'E': triggerEstop("host"); break;
    case 'C':
      if (estop && digitalRead(ESTOP_PIN) == LOW) {
        Serial.println(F("! release the e-stop button first"));
        Serial.println(F("S ESTOP button still pressed"));  // keep the browser in e-stop too
        break;
      }
      if (estop) {
        estop = false;
        syncToActual();
        if (mode == MODE_WRITE) attachAll();
        Serial.println(F("S READY"));
      }
      break;
    case 'H': startHoming(); break;
    case 'W':
      // Only resync when coming back from read mode, so a W sent while the arm is
      // homing (e.g. right after the browser connects) doesn't cancel the homing.
      if (mode != MODE_WRITE) {
        mode = MODE_WRITE;
        if (!estop) { syncToActual(); attachAll(); }
      }
      Serial.println(F("S MODE W"));
      if (homing) Serial.println(F("S HOMING"));
      break;
    case 'R':
#if HAS_FEEDBACK
      mode = MODE_READ;
      homing = false;
      detachAll();  // go limp so you can move the arm by hand
      Serial.println(F("S MODE R"));
#else
      Serial.println(F("! read mode needs feedback: set HAS_FEEDBACK 1"));
#endif
      break;
    case 'V': {
      char *v = strtok(NULL, " ");
      if (v) maxSpeed = constrain(atof(v), 5, FW_MAX_SPEED);
      break;
    }
    case 'T': {
      if (estop || homing || mode != MODE_WRITE) break;  // ignored while unsafe/busy
      for (uint8_t i = 0; i < JOINTS; i++) {
        char *v = strtok(NULL, " ");
        if (!v) break;
        target[i] = clampJoint(i, atof(v));
      }
      attachAll();
      break;
    }
    case 'K': autoCalibrate(); break;
    case 'L': {
      char *j = strtok(NULL, " ");
      char *a = strtok(NULL, " ");
      if (j && a) learnPoint(atoi(j), atof(a));
      break;
    }
    default:
      Serial.print(F("! unknown command: ")); Serial.println(tok);
  }
}

void readSerial() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { buf[len] = 0; handleLine(buf); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = ch;
    else len = 0;  // line too long: drop it
  }
}

void report() {
  float p[JOINTS];
#if HAS_FEEDBACK
  readMeasured(p);
#else
  for (uint8_t i = 0; i < JOINTS; i++) p[i] = current[i];
#endif
  Serial.print('P');
  for (uint8_t i = 0; i < JOINTS; i++) { Serial.print(' '); Serial.print(p[i], 1); }
  Serial.println();
}

// ---- Arduino entry points ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  loadCalibration();

  // Power-up homing: start from the real pose if we can measure it, else assume the
  // arm was left near HOME (park it there before switching off!).
  for (uint8_t i = 0; i < JOINTS; i++) current[i] = HOME_POSE[i];
  syncToActual();
  hello();
  if (digitalRead(ESTOP_PIN) == LOW) triggerEstop("button held at boot");
  else startHoming();
}

void loop() {
  readSerial();
  if (digitalRead(ESTOP_PIN) == LOW) triggerEstop("button");

  unsigned long now = millis();
  static unsigned long lastTick = 0, lastReport = 0;

  if (now - lastTick >= TICK_MS) {
    float dt = (now - lastTick) / 1000.0;
    if (dt > 0.1) dt = 0.1;
    lastTick = now;

    if (!estop && mode == MODE_WRITE) {
      // Watchdog: host went silent mid-move? Hold position instead of carrying on.
      if (!homing && now - lastHostMs > HOST_TIMEOUT_MS)
        for (uint8_t i = 0; i < JOINTS; i++) target[i] = current[i];
      bool moving = slewStep(homing ? min(HOMING_SPEED, maxSpeed) : maxSpeed, dt);
      if (homing && !moving) { homing = false; Serial.println(F("S HOMED")); }
    }
    // LED: solid = ready, slow blink = e-stop, fast blink = homing, off = read mode.
    bool led = estop ? (now / 500) % 2 : homing ? (now / 100) % 2 : mode == MODE_WRITE;
    digitalWrite(LED_PIN, led);
  }

  if (now - lastReport >= REPORT_MS) {
    lastReport = now;
    report();
  }
}
