/*
  RC CAR FIRMWARE  (Arduino Nano + I/O expansion shield)

  Hardware : 2x N20 12V rear motors on a TA6586, steering servo, 3 ultrasonic sonars,
             MPU-9250 (I2C), HC-05/HC-06 Bluetooth, MOSFET headlights, battery divider on A6.

  FEATURES
   - Proportional throttle + steering over Bluetooth, with a link-loss failsafe
   - Adjustable acceleration ramp, Eco / Normal / Sport drive modes
   - Electronic differential (inner rear wheel slows in turns)
   - Speed-sensitive steering
   - Gyro: stability assist, straight-line heading hold, flip detection, crash detection
   - Non-blocking sonar reads, speed-scaled auto-brake, wall steer-assist
   - Autonomous modes: Roam, Follow, Corridor (+ Hold-speed in manual)
   - Low-battery limp mode and battery cutoff
   - Warning flash patterns on the headlights, hazard lights
   - Start-up self-test reported by headlight flashes
   - Optional buzzer (set HAS_BUZZER 1)

  PHONE PROTOCOL  (text lines ending in \n, 9600 baud by default)
  App -> car (any valid command also counts as a heartbeat, send something at least every 200 ms):
    D,<throttle>,<steer>   throttle -100..100 (+ forward), steer -100..100 (+ right)
    X                      emergency brake
    M,<0|1|2>              drive mode: 0 Eco, 1 Normal, 2 Sport
    A,<0|1|2|3>            0 manual, 1 Roam, 2 Follow, 3 Corridor
    K,<0|1>                hold speed on/off (locks the current throttle, steering stays live)
    L,<0|1>                headlights
    B,<0-100>              headlight brightness in % (software PWM on D2; warning flashes stay full brightness)
    H,<0|1>                hazard lights
    V,<0|1>                horn (needs a buzzer)
    R,<ms>                 ramp time to full throttle in ms (0 = instant / launch mode)
    S,<0|1>                gyro stability assist on/off
    G,<0|1>                straight-line heading hold on/off
    Z                      re-zero gyro drift and yaw (car must be stationary)
    C,<1|0>                1 = level the tilt readings and zero the gyro, saved to EEPROM (car still, on a flat surface)
                           0 = clear the saved level trim
    N                      flip the yaw direction (saved to EEPROM)
    P                      ping (heartbeat only)
  Car -> app (about 4 lines per second each):
    S,<distCenter>,<distLeft>,<distRight>,<driveMode>,<autoMode>,<flags>
    I,<roll>,<pitch>,<yawDeg>,<gForce x10>,<battery volts x100>,<speed %>
  flags bitmask: 1 failsafe, 2 limp, 4 battery cutoff, 8 flipped, 16 front blocked,
                 32 crash, 64 hold speed, 128 stability assist, 256 heading hold

  START-UP SELF-TEST (headlight flashes): one long flash = all good.
    2 short = IMU not found, 3 short = a sonar saw no echo, 4 short = battery voltage odd.

  BATTERY SENSE (optional, HAS_BATT_SENSE)
   The HX-2S-D20 BMS has no signal wire, so the Arduino cannot read it. To show the battery voltage,
   wire pack + (after the power switch) -> 10k -> A6 -> 10k -> GND, then set HAS_BATT_SENSE to 1.
   A full 2S pack (8.4 V) gives 4.2 V at A6. Do NOT set it to 1 with nothing on A6: a floating pin
   makes the car think the battery is dead (cutoff, throttle blocked, warning beeps).

  NOTES
   - There is no rear sensor, so reversing (including the Roam back-up) is not protected.
   - SoftwareSerial (D9/D10) blocks interrupts while it sends or receives a byte, which can make the
     servo twitch and slows sonar timing. It is great for bench work with USB debugging. For the
     final car, set USE_HW_SERIAL_BT 1 (move the Bluetooth module back to the shield UART socket,
     and unplug it when uploading), or raise the HC-05 to 38400 baud and set BT_BAUD to match.
   - Steering: positive = right. If the wheels turn the wrong way, set SERVO_DIR to -1.
   - First tests: wheels off the ground, check each motor direction (INVERT_LEFT / INVERT_RIGHT).
*/

#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// =====================================================================
//  USER CONFIG
// =====================================================================
#define USE_HW_SERIAL_BT 1     // 1 = Bluetooth on the shield UART socket (D0/D1). Unplug the module before uploading.
                               // 0 = Bluetooth on D9/D10 with SoftwareSerial (USB debugging works, but servo may twitch)
#define HAS_BUZZER       1     // active buzzer wired to D12 (0 = no buzzer)
#define BUZZER_ACTIVE_LOW 0    // 1 if your buzzer module is "low-level trigger" (beeps when I/O is LOW).
                               // If it buzzes constantly after power-up, set this to 1.
#define HAS_BATT_SENSE   0     // 1 = two 10k resistors wired from the pack + to A6 and from A6 to GND (see header).
                               // 0 = nothing on A6: battery readout, limp mode and cutoff are all switched off.
                               // Leaving A6 unwired with this set to 1 makes the car think the battery is dead.
const long BT_BAUD = 9600;     // 38400 is smoother if you set the HC-05 with AT+UART=38400,0,0

// Motor wiring tweaks
const bool SWAP_SIDES    = false;  // true if left and right are swapped
const bool INVERT_LEFT   = false;  // true if the left wheel spins backwards on forward
const bool INVERT_RIGHT  = false;
const int8_t SERVO_DIR   = 1;      // -1 if "steer right" turns the car left
const uint8_t SERVO_CENTER = 90;   // adjust to trim steering
const uint8_t SERVO_RANGE  = 35;   // degrees each side (55..125). Do not exceed what the linkage allows
const int8_t GYRO_SIGN   = 1;      // start value only. Flip it from the app (Calibrate > Flip yaw direction), it is saved in EEPROM

// =====================================================================
//  PINS
// =====================================================================
const uint8_t PIN_MOTOR_L_A = 3;   // TA6586 d0
const uint8_t PIN_MOTOR_L_B = 5;   // TA6586 d1
const uint8_t PIN_MOTOR_R_A = 6;   // TA6586 d2
const uint8_t PIN_MOTOR_R_B = 11;  // TA6586 d3
const uint8_t PIN_SERVO     = 4;
const uint8_t PIN_HEADLIGHT = 2;
const uint8_t PIN_BUZZER    = 12;
const uint8_t PIN_BATT      = A6;
const uint8_t BT_RX_PIN     = 9;   // to Bluetooth TX
const uint8_t BT_TX_PIN     = 10;  // to Bluetooth RX (through a divider if you want to be safe)

// sonar index: 0 = center, 1 = left, 2 = right
const uint8_t SONAR_TRIG[3] = { 7, A3, A1 };
const uint8_t SONAR_ECHO[3] = { 8, A2, A0 };
const uint8_t SONAR_SEQ[4]  = { 0, 1, 0, 2 };   // center is read twice per cycle

// =====================================================================
//  TUNING
// =====================================================================
struct DriveMode { uint8_t maxPct; uint16_t rampMs; uint8_t diffPct; uint8_t minSteerPct; };
const DriveMode MODES[3] = {
  {  55, 600, 25, 40 },   // Eco
  {  80, 300, 30, 50 },   // Normal
  { 100, 150, 35, 60 }    // Sport
};

const uint8_t MOTOR_MIN_PWM = 35;        // lowest PWM that reliably turns the N20s

// Obstacles
const int   BRAKE_BASE_CM     = 15;      // brake distance when crawling
const float BRAKE_CM_PER_PCT  = 0.30f;   // extra cm of brake distance per 1% throttle (100% = 45 cm)
const int   CAR_HALF_WIDTH_CM = 7;       // half the car's width
const int   SIDE_MARGIN_CM    = 3;
const int   SIDE_SIN_PCT      = 64;      // sin of side-sonar angle x100 (30 deg=50, 40 deg=64, 45 deg=71)
const int   SIDE_COS_PCT      = 77;      // cos of side-sonar angle x100 (30 deg=87, 40 deg=77, 45 deg=71)
const int   WALL_ASSIST_CM    = 35;      // manual wall steer-assist starts inside this distance
const float WALL_KP           = 1.5f;
const float WALL_ASSIST_MAX   = 40.0f;

// Failsafe and battery
const unsigned long LINK_TIMEOUT_MS = 600;
const float R1 = 10000.0f, R2 = 10000.0f;
const float V_REF = 5.0f;
const float BATT_CAL = 1.0f;             // adjust until telemetry matches a multimeter
const float LIMP_V = 6.8f, LIMP_CLEAR_V = 7.0f;
const float CUTOFF_V = 6.4f, CUTOFF_CLEAR_V = 6.9f;
const float LIMP_MAX_PCT = 50.0f;

// Gyro assists
const float FLIP_DEG = 60.0f;
const unsigned long FLIP_MS = 300;
const float CRASH_G = 2.5f;
const float HH_KP = 2.0f, HH_KD = 0.10f, HH_MAX = 25.0f;               // heading hold
const float STAB_KP = 0.15f, STAB_DEADBAND_DPS = 40.0f;                // stability assist
const float YAW_MODEL_DPS = 220.0f, STAB_MAX = 20.0f;                  // expected yaw at full steer + full speed

// Autonomous modes
const float ROAM_SPEED_PCT = 50.0f, ROAM_BACK_PCT = 35.0f, ROAM_TURN_PCT = 40.0f;
const unsigned long ROAM_BACK_MS = 650, ROAM_TURN_MS = 600;
const float FOLLOW_TARGET_CM = 30.0f, FOLLOW_LOSE_CM = 90.0f, FOLLOW_DEADBAND_CM = 5.0f;
const float CORRIDOR_SPEED_PCT = 45.0f;

// =====================================================================
//  GLOBALS
// =====================================================================
#if USE_HW_SERIAL_BT
  #define BTPORT Serial
  #define DBG_BEGIN() ((void)0)
  #define DBG(x)      ((void)0)
  #define DBGLN(x)    ((void)0)
#else
  SoftwareSerial BTPORT(BT_RX_PIN, BT_TX_PIN);
  #define DBG_BEGIN() Serial.begin(115200)
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#endif

Servo steeringServo;

// Commands and modes
int8_t  cmdThrottle = 0, cmdSteer = 0;
uint8_t driveMode = 1, autoMode = 0;
uint16_t rampMs = 300;
bool lightsUser = false, hazard = false, hornOn = false;
uint8_t headBrightness = 100;       // 0..100 %, software PWM on D2
bool holdSpeed = false; int8_t holdValue = 0;
bool stabAssist = true, headingHold = true;

// Safety state
bool linkSeen = false, linkLost = false, faultPrev = false, needNeutral = true;
bool flipped = false, cutoff = false, limp = false, crashFlag = false;
bool blockedFront = false, blockedPrev = false;
unsigned long lastPacketMs = 0, brakeUntil = 0, obstacleFlashUntil = 0, crashFlagUntil = 0;

// Drive state
float speedCur = 0;                 // actual output, -100..100 %
bool  hhActive = false; float hhTarget = 0;

// Sonar
uint16_t distNow[3] = { 999, 999, 999 }, distPrev[3] = { 999, 999, 999 };
uint8_t sonarSeqPos = 0, sonarPhase = 0;
unsigned long sonarT = 0;
const unsigned long SONAR_GAP_US = 3000, SONAR_RISE_TIMEOUT_US = 6000, SONAR_FALL_TIMEOUT_US = 12000;

// IMU
bool imuOk = false;
float roll = 0, pitch = 0, yawDeg = 0, gyroZ = 0, gyroBiasZ = 0, gMag = 1.0f;
float rollTrim = 0, pitchTrim = 0;      // how far from level the board sits on the chassis (saved in EEPROM)
int8_t gyroSign = GYRO_SIGN;            // saved in EEPROM
unsigned long lastImuUs = 0, tiltStart = 0;
uint8_t crashCount = 0;

// Battery
float battV = 0; bool battValid = false;

// Roam state machine
enum { ROAM_GO = 0, ROAM_BACK = 1, ROAM_TURN = 2 };
uint8_t roamState = ROAM_GO; int8_t roamDir = 1; unsigned long roamUntil = 0;

// Timers
unsigned long lastCtrlMs = 0, lastImuMs = 0, lastBattMs = 0, lastTelMs = 0, lastDbgMs = 0;
bool telToggle = false;

// Bluetooth line buffer
const uint8_t RX_BUF = 24;
char rxBuf[RX_BUF]; uint8_t rxLen = 0;

// =====================================================================
//  SMALL HELPERS
// =====================================================================
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float minf(float a, float b) { return a < b ? a : b; }
static inline int8_t clampPct(long v) { return (int8_t)(v < -100 ? -100 : (v > 100 ? 100 : v)); }

uint16_t distF(uint8_t i) { return distNow[i] > distPrev[i] ? distNow[i] : distPrev[i]; }  // ignores one-off glitches

// =====================================================================
//  MOTORS
// =====================================================================
uint8_t pctToPwm(float p) {
  if (p <= 0) return 0;
  if (p > 100) p = 100;
  return (uint8_t)(MOTOR_MIN_PWM + (255 - MOTOR_MIN_PWM) * (p / 100.0f));
}

void driveMotor(uint8_t pa, uint8_t pb, float pct, bool invert) {
  if (invert) pct = -pct;
  if (pct > 0.5f)       { analogWrite(pa, pctToPwm(pct));  digitalWrite(pb, LOW); }
  else if (pct < -0.5f) { digitalWrite(pa, LOW);           analogWrite(pb, pctToPwm(-pct)); }
  else                  { digitalWrite(pa, LOW);           digitalWrite(pb, LOW); }   // coast
}

void setMotors(float leftPct, float rightPct) {
  if (SWAP_SIDES) { float t = leftPct; leftPct = rightPct; rightPct = t; }
  driveMotor(PIN_MOTOR_L_A, PIN_MOTOR_L_B, leftPct,  INVERT_LEFT);
  driveMotor(PIN_MOTOR_R_A, PIN_MOTOR_R_B, rightPct, INVERT_RIGHT);
}

void brakeMotors() {
  digitalWrite(PIN_MOTOR_L_A, HIGH); digitalWrite(PIN_MOTOR_L_B, HIGH);
  digitalWrite(PIN_MOTOR_R_A, HIGH); digitalWrite(PIN_MOTOR_R_B, HIGH);
}

void emergencyStop(unsigned long ms) {
  brakeUntil = millis() + ms;
  speedCur = 0;
  hhActive = false;
}

// =====================================================================
//  SONAR  (non-blocking state machine)
// =====================================================================
void sonarFinish(uint8_t s, uint16_t value) {
  distPrev[s] = distNow[s];
  distNow[s] = value;
  sonarSeqPos = (sonarSeqPos + 1) & 3;
  sonarPhase = 0;
  sonarT = micros();
}

void sonarUpdate() {
  unsigned long nowUs = micros();
  uint8_t s = SONAR_SEQ[sonarSeqPos];
  switch (sonarPhase) {
    case 0:   // idle gap, then trigger (never wait forever on a sensor whose echo line is stuck high)
      if (nowUs - sonarT >= SONAR_GAP_US) {
        if (!digitalRead(SONAR_ECHO[s])) {
          digitalWrite(SONAR_TRIG[s], HIGH);
          delayMicroseconds(10);
          digitalWrite(SONAR_TRIG[s], LOW);
          sonarT = micros();
          sonarPhase = 1;
        } else if (nowUs - sonarT > 80000UL) {
          sonarFinish(s, 999);
        }
      }
      break;
    case 1:   // wait for the echo pulse to start
      if (digitalRead(SONAR_ECHO[s])) { sonarT = nowUs; sonarPhase = 2; }
      else if (nowUs - sonarT > SONAR_RISE_TIMEOUT_US) sonarFinish(s, 999);
      break;
    case 2: { // wait for the echo pulse to end
      if (!digitalRead(SONAR_ECHO[s])) {
        unsigned long width = nowUs - sonarT;
        uint16_t cm = (uint16_t)(width / 58UL);
        if (cm < 2) cm = 2;
        sonarFinish(s, cm);
      } else if (nowUs - sonarT > SONAR_FALL_TIMEOUT_US) {
        sonarFinish(s, 999);
      }
      break;
    }
  }
}

bool sideHit(uint16_t d, int thr) {
  if (d >= 900) return false;
  long lateral = (long)d * SIDE_SIN_PCT / 100;
  long fwd     = (long)d * SIDE_COS_PCT / 100;
  return lateral < (CAR_HALF_WIDTH_CM + SIDE_MARGIN_CM) && fwd < thr;
}

// Is something in the way at this throttle? (brake distance grows with speed)
bool frontBlocked(float pct, bool useSides) {
  int thr = BRAKE_BASE_CM + (int)(pct * BRAKE_CM_PER_PCT);
  if (distF(0) < thr) return true;
  if (useSides && (sideHit(distF(1), thr) || sideHit(distF(2), thr))) return true;
  return false;
}

float wallAssist() {
  float a = 0;
  float dL = distF(1), dR = distF(2);
  if (dL < WALL_ASSIST_CM) a += (WALL_ASSIST_CM - dL) * WALL_KP;    // near left wall -> steer right
  if (dR < WALL_ASSIST_CM) a -= (WALL_ASSIST_CM - dR) * WALL_KP;
  return clampf(a, -WALL_ASSIST_MAX, WALL_ASSIST_MAX);
}

// =====================================================================
//  IMU (MPU-9250, raw I2C so no library is needed)
// =====================================================================
const uint8_t MPU_ADDR = 0x68;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuRead8(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

int16_t wireRead16() {                             // two separate reads: C++ does not fix the order inside one expression
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

bool imuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  ax = wireRead16();
  ay = wireRead16();
  az = wireRead16();
  wireRead16();                                    // temperature
  wireRead16();                                    // gyro X
  wireRead16();                                    // gyro Y
  gz = wireRead16();
  return true;
}

// ---- saved calibration (EEPROM) ----
struct CalData { uint8_t magic; int16_t rollX100; int16_t pitchX100; int8_t sign; };
const uint8_t CAL_MAGIC = 0xA7;

void saveCal() {
  CalData c;
  c.magic = CAL_MAGIC;
  c.rollX100 = (int16_t)(rollTrim * 100.0f);
  c.pitchX100 = (int16_t)(pitchTrim * 100.0f);
  c.sign = gyroSign;
  EEPROM.put(0, c);                                // only writes bytes that changed
}

void loadCal() {
  CalData c;
  EEPROM.get(0, c);
  if (c.magic != CAL_MAGIC || (c.sign != 1 && c.sign != -1)) return;
  rollTrim = c.rollX100 / 100.0f;
  pitchTrim = c.pitchX100 / 100.0f;
  gyroSign = c.sign;
}

// Gyro zero (always) and, when level is true, the board's tilt trim. The car must be still.
// The level step is skipped if the surface is tilted more than 25 degrees (or the car is upside down).
void imuCalibrate(bool level) {
  float sumG = 0, sumX = 0, sumY = 0, sumZ = 0; int n = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax, ay, az, gz;
    if (imuReadRaw(ax, ay, az, gz)) {
      sumG += gz / 65.5f;
      sumX += ax; sumY += ay; sumZ += az;
      n++;
    }
    delay(5);
  }
  if (n > 50) {
    gyroBiasZ = sumG / n;
    if (level) {
      float fx = sumX / n / 8192.0f, fy = sumY / n / 8192.0f, fz = sumZ / n / 8192.0f;
      float r = atan2(fy, fz) * 57.2958f;
      float p = atan2(-fx, sqrt(fy * fy + fz * fz)) * 57.2958f;
      if (fabs(r) < 25.0f && fabs(p) < 25.0f) { rollTrim = r; pitchTrim = p; saveCal(); }
    }
  }
  yawDeg = 0; hhActive = false;
}

bool imuInit() {
  Wire.begin();
  Wire.setClock(400000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);
#endif
  mpuWrite(0x6B, 0x01);                            // wake up, use gyro clock
  delay(100);
  uint8_t who = mpuRead8(0x75);
  if (who != 0x71 && who != 0x73 && who != 0x70 && who != 0x68) return false;
  mpuWrite(0x1A, 0x03);                            // gyro low-pass 41 Hz
  mpuWrite(0x1B, 0x08);                            // gyro +-500 dps
  mpuWrite(0x1C, 0x08);                            // accel +-4 g
  mpuWrite(0x1D, 0x02);                            // accel low-pass ~99 Hz
  imuCalibrate(false);                             // gyro zero only; the level trim comes from EEPROM
  lastImuUs = micros();
  return true;
}

void imuUpdate() {
  if (!imuOk) return;
  int16_t ax, ay, az, gz;
  if (!imuReadRaw(ax, ay, az, gz)) return;
  unsigned long nowUs = micros();
  float dt = (nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  float fx = ax / 8192.0f, fy = ay / 8192.0f, fz = az / 8192.0f;
  float rawZ = gz / 65.5f - gyroBiasZ;
  gyroZ = gyroSign * rawZ;
  yawDeg += gyroZ * dt;
  roll  = atan2(fy, fz) * 57.2958f - rollTrim;
  pitch = atan2(-fx, sqrt(fy * fy + fz * fz)) * 57.2958f - pitchTrim;
  gMag  = sqrt(fx * fx + fy * fy + fz * fz);

  // slowly learn the gyro bias while the car sits still
  if (speedCur == 0 && fabs(rawZ) < 1.5f) gyroBiasZ += rawZ * 0.002f;

  unsigned long now = millis();

  // flip detection
  bool tilted = fabs(roll) > FLIP_DEG || fabs(pitch) > FLIP_DEG;
  if (tilted) {
    if (tiltStart == 0) tiltStart = now | 1;
    else if (now - tiltStart > FLIP_MS && !flipped) { flipped = true; emergencyStop(500); }
  } else {
    tiltStart = 0;
    if (fabs(roll) < 30.0f && fabs(pitch) < 30.0f) flipped = false;
  }

  // crash detection (hard hit while moving)
  if (gMag > CRASH_G) {
    if (++crashCount >= 2 && fabs(speedCur) > 5.0f) {
      emergencyStop(400);
      obstacleFlashUntil = now + 700;
      crashFlag = true; crashFlagUntil = now + 2000;
      crashCount = 0;
    }
  } else crashCount = 0;
  if (crashFlag && now > crashFlagUntil) crashFlag = false;
}

// =====================================================================
//  BATTERY
// =====================================================================
void batteryUpdate() {
#if !HAS_BATT_SENSE
  battV = 0; battValid = false; limp = false; cutoff = false;   // nothing wired to A6
  return;
#endif
  float raw = analogRead(PIN_BATT);
  float v = (raw / 1023.0f) * V_REF * ((R1 + R2) / R2) * BATT_CAL;
  battV = (battV <= 0.1f) ? v : battV * 0.9f + v * 0.1f;
  battValid = battV > 3.0f;                         // below that we assume USB-only bench testing
  if (!battValid) { limp = false; cutoff = false; return; }
  if (!limp && battV < LIMP_V) limp = true;
  else if (limp && battV > LIMP_CLEAR_V) limp = false;
  if (!cutoff && battV < CUTOFF_V) cutoff = true;
  else if (cutoff && battV > CUTOFF_CLEAR_V) cutoff = false;
}

// =====================================================================
//  AUTONOMOUS MODES
// =====================================================================
void autoDrive(float &t, float &s) {
  unsigned long now = millis();
  float dC = distF(0), dL = distF(1), dR = distF(2);
  t = 0; s = 0;

  switch (autoMode) {
    case 1:   // ROAM: drive, steer away from walls, back out when stuck
      switch (roamState) {
        case ROAM_GO: {
          if (frontBlocked(ROAM_SPEED_PCT, true)) {
            roamDir = (dR >= dL) ? 1 : -1;            // turn toward the more open side
            roamState = ROAM_BACK; roamUntil = now + ROAM_BACK_MS;
            break;
          }
          float slow = clampf((dC - 25.0f) / 55.0f, 0.5f, 1.0f);
          t = ROAM_SPEED_PCT * slow;
          s = clampf((minf(dR, 100.0f) - minf(dL, 100.0f)) * 0.9f, -100.0f, 100.0f);
          break;
        }
        case ROAM_BACK:   // reverse with wheels opposite so the nose swings to the open side
          t = -ROAM_BACK_PCT; s = -roamDir * 100.0f;
          if (now >= roamUntil) { roamState = ROAM_TURN; roamUntil = now + ROAM_TURN_MS; }
          break;
        case ROAM_TURN:
          t = ROAM_TURN_PCT; s = roamDir * 100.0f;
          if (frontBlocked(ROAM_TURN_PCT, true)) {
            roamDir = (dR >= dL) ? 1 : -1;
            roamState = ROAM_BACK; roamUntil = now + ROAM_BACK_MS;
          } else if (now >= roamUntil) roamState = ROAM_GO;
          break;
      }
      break;

    case 2: {  // FOLLOW: keep a set distance from whatever is in front
      if (dC > FOLLOW_LOSE_CM) break;
      float err = dC - FOLLOW_TARGET_CM;
      if (fabs(err) >= FOLLOW_DEADBAND_CM) t = clampf(err * 2.0f, -30.0f, 45.0f);
      s = clampf((minf(dL, 100.0f) - minf(dR, 100.0f)) * 1.2f, -100.0f, 100.0f);
      break;
    }

    case 3:    // CORRIDOR: stay centered between two walls
      t = CORRIDOR_SPEED_PCT;
      s = clampf((minf(dR, 150.0f) - minf(dL, 150.0f)) * 0.8f, -100.0f, 100.0f);
      break;
  }
}

// =====================================================================
//  MAIN CONTROL LOOP  (every 10 ms)
// =====================================================================
void controlTick() {
  unsigned long now = millis();

  // ---- link and fault state ----
  linkLost = linkSeen && (now - lastPacketMs > LINK_TIMEOUT_MS);
  bool fault = linkLost || flipped || cutoff;
  if (fault && !faultPrev) {
    emergencyStop(300);
    autoMode = 0; holdSpeed = false;
    cmdThrottle = 0; cmdSteer = 0;
  }
  faultPrev = fault;
  if (fault) needNeutral = true;
  else if (needNeutral && abs(cmdThrottle) < 5) needNeutral = false;

  // ---- target throttle and steering ----
  float tgtT = 0, tgtS = 0;
  float maxPct = MODES[driveMode].maxPct;
  if (limp && maxPct > LIMP_MAX_PCT) maxPct = LIMP_MAX_PCT;

  if (!fault) {
    if (autoMode == 0) {
      tgtS = cmdSteer;
      if (!needNeutral) {
        tgtT = holdSpeed ? holdValue : cmdThrottle;
        tgtT *= maxPct / 100.0f;
      }
    } else {
      autoDrive(tgtT, tgtS);
      tgtT = clampf(tgtT, -maxPct, maxPct);
    }
  }

  // ---- obstacle auto-brake ----
  blockedFront = false;
  if (tgtT > 1.0f) {
    blockedFront = frontBlocked(tgtT > speedCur ? tgtT : speedCur, autoMode != 2);
    if (blockedFront) {
      tgtT = 0;
      if (!blockedPrev && speedCur > 10.0f) { emergencyStop(250); obstacleFlashUntil = now + 700; }
    }
  }
  blockedPrev = blockedFront;

  // ---- ramp ----
  if (now < brakeUntil) {
    brakeMotors();
    speedCur = 0;
  } else {
    float step = (rampMs == 0) ? 100.0f : 1000.0f / rampMs;       // % per 10 ms
    float st = step;
    if (fabs(tgtT) < fabs(speedCur) && tgtT * speedCur >= 0) st = step * 2.0f;   // slow down faster
    if (speedCur < tgtT)      { speedCur += st; if (speedCur > tgtT) speedCur = tgtT; }
    else if (speedCur > tgtT) { speedCur -= st; if (speedCur < tgtT) speedCur = tgtT; }
  }

  // ---- steering: speed-sensitive + gyro and wall assists ----
  float f = fabs(speedCur) / 100.0f;
  float scale = 1.0f - (1.0f - MODES[driveMode].minSteerPct / 100.0f) * f;
  float steerOut = tgtS * scale;
  float corr = 0;
  float dirSign = (speedCur < 0) ? -1.0f : 1.0f;

  if (autoMode == 0 && !fault && imuOk && fabs(speedCur) > 15.0f) {
    bool driverSteering = fabs(tgtS) >= 8.0f;
    if (headingHold && !driverSteering) {
      if (!hhActive) { hhActive = true; hhTarget = yawDeg; }
      corr += dirSign * clampf(HH_KP * (yawDeg - hhTarget) + HH_KD * gyroZ, -HH_MAX, HH_MAX);
    } else hhActive = false;

    if (stabAssist && driverSteering) {
      float expected = -(steerOut / 100.0f) * f * YAW_MODEL_DPS * dirSign;
      float excess = gyroZ - expected;
      if (fabs(excess) > STAB_DEADBAND_DPS) {
        float e = (excess > 0) ? excess - STAB_DEADBAND_DPS : excess + STAB_DEADBAND_DPS;
        corr += dirSign * clampf(STAB_KP * e, -STAB_MAX, STAB_MAX);
      }
    }
  } else hhActive = false;

  if (autoMode == 0 && !fault && speedCur > 10.0f) corr += wallAssist();

  steerOut = clampf(steerOut + corr, -100.0f, 100.0f);
  steeringServo.write((int)(SERVO_CENTER + SERVO_DIR * steerOut * SERVO_RANGE / 100.0f));

  // ---- electronic differential and motor output ----
  if (now >= brakeUntil) {
    float s = steerOut / 100.0f;
    float diff = MODES[driveMode].diffPct / 100.0f;
    float left = speedCur, right = speedCur;
    if (s > 0) right *= (1.0f - diff * s);          // right turn: right wheel is the inner one
    else       left  *= (1.0f + diff * s);          // left turn:  left wheel is the inner one
    setMotors(left, right);
  }
}

// =====================================================================
//  LIGHTS AND BUZZER
// =====================================================================
// D2 has no hardware PWM, so dim the headlights with a ~400 Hz software PWM. Called every loop pass.
void headlightWrite(bool lit, uint8_t duty) {
  if (!lit || duty == 0)  { digitalWrite(PIN_HEADLIGHT, LOW);  return; }
  if (duty >= 100)        { digitalWrite(PIN_HEADLIGHT, HIGH); return; }
  const unsigned long PERIOD_US = 2500UL;
  unsigned long phase = micros() % PERIOD_US;
  digitalWrite(PIN_HEADLIGHT, (phase < (PERIOD_US * duty) / 100UL) ? HIGH : LOW);
}

void buzzerWrite(bool on) {
#if HAS_BUZZER
  digitalWrite(PIN_BUZZER, (on != (bool)BUZZER_ACTIVE_LOW) ? HIGH : LOW);
#else
  (void)on;
#endif
}

void lightsUpdate() {
  unsigned long now = millis();
  bool warn = false, warnOn = false, hazardOnly = false;

  if (flipped)                          { warn = true; warnOn = (now % 120) < 60; }                 // strobe
  else if (linkLost || cutoff)          { warn = true; warnOn = (now % 200) < 100; }                // fast blink
  else if (now < obstacleFlashUntil)    { warn = true; warnOn = (now % 200) < 100; }                // 3 quick flashes
  else if (limp)                        { warn = true; unsigned long m = now % 2000; warnOn = (m < 80) || (m >= 200 && m < 280); }  // double blink
  else if (hazard)                      { warn = true; hazardOnly = true; warnOn = (now % 1000) < 500; }

  if (warn) headlightWrite(warnOn, 100);                // warnings always full brightness
  else      headlightWrite(lightsUser, headBrightness);
  buzzerWrite(hornOn || (warn && warnOn && !hazardOnly));
}

// =====================================================================
//  BLUETOOTH
// =====================================================================
void handleLine(char *line) {
  char cmd = line[0];
  long a = 0, b = 0;
  char *p = strchr(line, ',');
  if (p) { a = atol(p + 1); p = strchr(p + 1, ','); if (p) b = atol(p + 1); }

  bool known = true;
  switch (cmd) {
    case 'D': cmdThrottle = clampPct(a); cmdSteer = clampPct(b); break;
    case 'X': cmdThrottle = 0; holdSpeed = false; emergencyStop(300); break;
    case 'M': driveMode = (uint8_t)(a < 0 ? 0 : (a > 2 ? 2 : a)); rampMs = MODES[driveMode].rampMs; break;
    case 'A': autoMode = (uint8_t)(a < 0 ? 0 : (a > 3 ? 3 : a)); roamState = ROAM_GO; holdSpeed = false; break;
    case 'K':
      if (a && autoMode == 0 && cmdThrottle != 0) { holdSpeed = true; holdValue = cmdThrottle; }
      else holdSpeed = false;
      break;
    case 'L': lightsUser = (a != 0); break;
    case 'B': headBrightness = (uint8_t)(a < 0 ? 0 : (a > 100 ? 100 : a)); break;
    case 'H': hazard = (a != 0); break;
    case 'V': hornOn = (a != 0); break;
    case 'R': rampMs = (uint16_t)(a < 0 ? 0 : (a > 3000 ? 3000 : a)); break;
    case 'S': stabAssist = (a != 0); break;
    case 'G': headingHold = (a != 0); hhActive = false; break;
    case 'Z': if (speedCur == 0 && imuOk) imuCalibrate(false); break;
    case 'C':
      if (strchr(line, ',') == NULL) break;                            // ignore a damaged "C" with no value
      if (a == 0) { rollTrim = 0; pitchTrim = 0; saveCal(); }          // C,0 clears the saved level trim
      else if (speedCur == 0 && imuOk) imuCalibrate(true);             // C,1 level + gyro zero, saved
      break;
    case 'N': gyroSign = -gyroSign; yawDeg = 0; hhActive = false; saveCal(); break;   // flip yaw direction
    case 'P': break;
    default: known = false; break;
  }
  if (known) { lastPacketMs = millis(); linkSeen = true; }
}

void readBluetooth() {
  while (BTPORT.available() > 0) {
    char c = BTPORT.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) { rxBuf[rxLen] = 0; handleLine(rxBuf); }
      rxLen = 0;
    } else if (rxLen < RX_BUF - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;                                   // overflow: drop the line
    }
  }
}

void telemetryUpdate() {
  unsigned long now = millis();
  if (now - lastTelMs < 125) return;
  lastTelMs = now;
  if (!linkSeen) return;
  telToggle = !telToggle;

  if (telToggle) {
    int flags = (linkLost ? 1 : 0) | (limp ? 2 : 0) | (cutoff ? 4 : 0) | (flipped ? 8 : 0) |
                (blockedFront ? 16 : 0) | (crashFlag ? 32 : 0) | (holdSpeed ? 64 : 0) |
                (stabAssist ? 128 : 0) | (headingHold ? 256 : 0);
    BTPORT.print(F("S,"));  BTPORT.print(distF(0));
    BTPORT.print(',');      BTPORT.print(distF(1));
    BTPORT.print(',');      BTPORT.print(distF(2));
    BTPORT.print(',');      BTPORT.print(driveMode);
    BTPORT.print(',');      BTPORT.print(autoMode);
    BTPORT.print(',');      BTPORT.println(flags);
  } else {
    BTPORT.print(F("I,"));  BTPORT.print((int)roll);
    BTPORT.print(',');      BTPORT.print((int)pitch);
    BTPORT.print(',');      BTPORT.print((int)yawDeg);
    BTPORT.print(',');      BTPORT.print((int)(gMag * 10.0f));
    BTPORT.print(',');      BTPORT.print((int)(battV * 100.0f));
    BTPORT.print(',');      BTPORT.println((int)speedCur);
  }
}

// =====================================================================
//  START-UP SELF-TEST
// =====================================================================
void flashN(uint8_t n, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(PIN_HEADLIGHT, HIGH); delay(onMs);
    digitalWrite(PIN_HEADLIGHT, LOW);  delay(offMs);
  }
}

// Only checks that the sensor answers a trigger (echo line rises). Works in an empty room too.
bool sonarProbe(uint8_t i) {
  for (uint8_t t = 0; t < 3; t++) {
    unsigned long guard = micros();
    while (digitalRead(SONAR_ECHO[i]) && micros() - guard < 250000UL) {}    // let a stuck-high echo finish
    digitalWrite(SONAR_TRIG[i], LOW); delayMicroseconds(2);
    digitalWrite(SONAR_TRIG[i], HIGH); delayMicroseconds(10);
    digitalWrite(SONAR_TRIG[i], LOW);
    unsigned long t0 = micros();
    while (micros() - t0 < 5000UL) {
      if (digitalRead(SONAR_ECHO[i])) {
        unsigned long t1 = micros();
        while (digitalRead(SONAR_ECHO[i]) && micros() - t1 < 250000UL) {}   // wait for the pulse to end
        return true;
      }
    }
    delay(40);
  }
  return false;
}

void selfTest() {
  bool sonarBad = false;
  for (uint8_t i = 0; i < 3; i++) {
    if (!sonarProbe(i)) {
      sonarBad = true;
      DBG(F("Self-test: no echo from sonar ")); DBGLN(i);
    }
    delay(30);
  }
  bool imuBad = !imuOk;
  bool battBad = battValid && (battV < 6.0f || battV > 8.7f);
  if (imuBad) DBGLN(F("Self-test: IMU not found"));
  if (battBad) { DBG(F("Self-test: odd battery voltage ")); DBGLN(battV); }
  if (!battValid) DBGLN(F("Self-test: no battery reading (USB-only?)"));

  delay(300);
  if (!imuBad && !sonarBad && !battBad) {
    flashN(1, 700, 300);
  } else {
    if (imuBad)  { flashN(2, 150, 150); delay(700); }
    if (sonarBad){ flashN(3, 150, 150); delay(700); }
    if (battBad) { flashN(4, 150, 150); delay(700); }
  }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  DBG_BEGIN();
  BTPORT.begin(BT_BAUD);
  loadCal();

  pinMode(PIN_MOTOR_L_A, OUTPUT); pinMode(PIN_MOTOR_L_B, OUTPUT);
  pinMode(PIN_MOTOR_R_A, OUTPUT); pinMode(PIN_MOTOR_R_B, OUTPUT);
  setMotors(0, 0);

  pinMode(PIN_HEADLIGHT, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  buzzerWrite(false);                              // silent from the first moment
  for (uint8_t i = 0; i < 3; i++) { pinMode(SONAR_TRIG[i], OUTPUT); pinMode(SONAR_ECHO[i], INPUT); }

  steeringServo.attach(PIN_SERVO);
  steeringServo.write(SERVO_CENTER);

  for (uint8_t i = 0; i < 20; i++) { batteryUpdate(); delay(5); }
  imuOk = imuInit();
  rampMs = MODES[driveMode].rampMs;
  selfTest();

  sonarT = micros();
  DBGLN(F("RC car ready."));
}

void loop() {
  readBluetooth();
  sonarUpdate();

  unsigned long now = millis();
  if (now - lastImuMs >= 10)  { lastImuMs = now;  imuUpdate(); }
  if (now - lastCtrlMs >= 10) { lastCtrlMs = now; controlTick(); }
  if (now - lastBattMs >= 100){ lastBattMs = now; batteryUpdate(); }

  lightsUpdate();
  telemetryUpdate();

#if !USE_HW_SERIAL_BT
  if (now - lastDbgMs >= 1000) {
    lastDbgMs = now;
    DBG(F("batt=")); DBG(battV); DBG(F(" C/L/R=")); DBG(distF(0)); DBG('/'); DBG(distF(1)); DBG('/'); DBGLN(distF(2));
  }
#endif
}
