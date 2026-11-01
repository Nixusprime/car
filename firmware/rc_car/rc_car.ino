/*
  RC CAR FIRMWARE  (Arduino Nano + I/O expansion shield)

  Hardware : 2x N20 12V rear motors on a TA6586, steering servo, 3 ultrasonic sonars,
             MPU-9250 (I2C), HC-05/HC-06 Bluetooth, MOSFET headlights, battery divider on A6.

  FEATURES
   - Proportional throttle + steering over Bluetooth, with a link-loss failsafe
   - Adjustable acceleration ramp, Eco / Normal / Sport drive modes
   - Electronic differential from the car's geometry (wheelbase, track, steering angle); the inner wheel
     slows and the outer wheel speeds up so the car keeps its speed through a turn
   - Speed-sensitive steering
   - Gyro: stability assist, straight-line heading hold, flip detection, crash detection
   - Collision system that works from the car's real speed (throttle, coasting, rolling): smooth speed limit,
     steer-around allowance, hard brake, emergency steering that hands back to the driver
   - ACC: hold speed that keeps a gap behind whatever is ahead; wall keeping; hill hold and launch assist
   - IMU motion sensing: real speed, stuck cut-off, turn check, learned forward direction (Calibrate > Motion)
   - Held brake (swipe to brake) with an ABS-style spin guard; steering eases back to center after release
   - Non-blocking sonar reads
   - Autonomous modes: Roam (forward turn first, short slow reverse as a last resort), Follow, Corridor
   - Low-battery limp mode and battery cutoff
   - Warning flash patterns on the headlights, hazard lights
   - Start-up self-test reported by headlight flashes
   - Buzzer (active or passive, see BUZZER_TYPE)

  PHONE PROTOCOL  (text lines ending in \n, 9600 baud by default)
  App -> car (any valid command also counts as a heartbeat, send something at least every 200 ms):
    D,<throttle>,<steer>[,<brake>]  throttle -100..100 (+ forward), steer -100..100 (+ right), brake 1 = hold the short brake
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
    F,<0-150>              electronic differential strength in % (100 = exact geometry, not saved)
    T,<mask>               assists on/off: 1 collision, 2 emergency steer, 4 wall keeping, 8 hill hold,
                           16 ABS guard, 32 stuck cut-off, 64 launch assist
    U,<ms>                 how long the steering takes to return to center after release (0 = instant)
    J,<50-200>             following gap scale in %
    Z                      re-zero gyro drift and yaw (car must be stationary)
    C,<1|0|2>              1 = level the tilt readings and zero the gyro, saved to EEPROM (car still, on a flat surface)
                           0 = clear the saved level trim
                           2 = motion calibration: the car drives itself ~30 cm straight at a wall 50-400 cm ahead and
                               learns the MPU's forward direction and its speed scale (saved)
    N                      flip the yaw direction (saved to EEPROM)
    P                      ping (heartbeat only)
  Car -> app (each of the three lines about 3 times per second):
    S,<distCenter>,<distLeft>,<distRight>,<driveMode>,<autoMode>,<flags>
    I,<roll>,<pitch>,<yawDeg>,<gForce x10>,<battery volts x100>,<speed %>
    M,<own speed cm/s>,<closing speed cm/s>,<yaw rate dps>,<expected yaw rate dps>,<slope deg x10>,<speed cap %>,<calState>,<forwardKnown>
  flags bitmask: 1 failsafe, 2 limp, 4 battery cutoff, 8 flipped, 16 front blocked,
                 32 crash, 64 hold speed, 128 stability assist, 256 heading hold,
                 512 stuck, 1024 emergency steering, 2048 hard braking, 4096 speed limited,
                 8192 hill hold, 16384 drifting or slipping, 32768 autonomous mode gave up
  calState: 0 idle, 1 running, 2 done, 3 no wall 50-400 cm ahead, 4 aborted, 5 odd result

  START-UP SELF-TEST (headlight flashes): one long flash = all good.
    2 short = IMU not found, 3 short = a sonar saw no echo, 4 short = battery voltage odd.

  BATTERY SENSE (optional, HAS_BATT_SENSE)
   The HX-2S-D20 BMS has no signal wire, so the Arduino cannot read it. To show the battery voltage,
   wire pack + (after the power switch) -> 10k -> A6 -> 10k -> GND, then set HAS_BATT_SENSE to 1.
   A full 2S pack (8.4 V) gives 4.2 V at A6. Do NOT set it to 1 with nothing on A6: a floating pin
   makes the car think the battery is dead (cutoff, throttle blocked, warning beeps).

  NOTES
   - There is no rear sensor, so reversing is blind. Autonomous modes avoid it: Roam tries a forward turn first and
     reverses only briefly and slowly (with beeps), and gives up after two reverses in a row.
   - The collision system is best effort: the sonar beams are narrow and soft or angled surfaces can miss the echo.
     Speeds are estimated, so tune BRAKE_DECEL, COMFORT_DECEL and the gaps on your own floor.
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
#define BUZZER_TYPE      2     // 0 = active buzzer, beeps when I/O is HIGH
                               // 1 = active buzzer on a "low-level trigger" module (beeps when I/O is LOW)
                               // 2 = passive buzzer (only clicks on plain DC): the sketch makes a ~2.5 kHz tone itself
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
struct DriveMode { uint8_t maxPct; uint16_t rampMs; uint8_t minSteerPct; };
const DriveMode MODES[3] = {
  {  55, 600, 40 },   // Eco
  {  80, 300, 50 },   // Normal
  { 100, 150, 60 }    // Sport
};

// Car geometry for the electronic differential (see differentialOut)
const float WHEELBASE_CM   = 17.0f;      // front axle to rear axle
const float TRACK_CM       = 11.0f;      // rear wheel centre to rear wheel centre
const float WHEEL_LOCK_DEG = 30.0f;      // how far the front wheels really turn at full steering. Measure it:
                                         // at full lock, turning-circle radius R (cm, from the middle of the rear axle)
                                         // gives WHEEL_LOCK_DEG = atan(17 / R) in degrees.

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
const float ROAM_SPEED_PCT = 50.0f, ROAM_BACK_PCT = 25.0f, ROAM_TURN_PCT = 35.0f;
const unsigned long ROAM_BACK_MS = 450, ROAM_TURN_MS = 600;
const float FOLLOW_TARGET_CM = 30.0f, FOLLOW_LOSE_CM = 90.0f, FOLLOW_DEADBAND_CM = 5.0f;
const float CORRIDOR_SPEED_PCT = 45.0f;

// Motion model and collision system (distances in cm, speeds in cm/s)
const float SPEED_CMS_PER_PCT = 0.9f;      // default: 90 cm/s at 100 % throttle. Calibrate > Motion measures the real value and saves it
const float SENSOR_LATENCY_S  = 0.15f;     // sonar update plus filter delay
const float BRAKE_DECEL       = 350.0f;    // cm/s^2 with the short brake (a first guess, tune on your floor)
const float COMFORT_DECEL     = 140.0f;    // cm/s^2 used for the smooth speed limit
const float GAP_BASE_CM       = 12.0f;     // stop this far short of an obstacle when crawling
const float GAP_PER_CMS       = 0.12f;     // plus this much for every cm/s of speed
const float GAP_MIN_CM        = 5.0f;      // closer than this counts as "about to touch"
const float FOLLOW_EXTRA_CM   = 8.0f;      // ACC keeps this much more than the stop gap
const float ACC_GAIN          = 1.6f;      // 1/s, how firmly ACC closes a gap error
const float STEER_CLEAR_MIN   = 15.0f;     // % of steering before the steer-around allowance starts
const float EVADE_MIN_CMS     = 20.0f;     // emergency steering only above this speed
const unsigned long EVADE_MS  = 700;       // how long emergency steering holds
const float ABS_YAW_DPS       = 120.0f;    // spin rate that makes the ABS guard pulse the brake
const float LOW_SPEED_DIFF_BOOST = 1.5f;   // extra differential strength when crawling (2.5x at 10 % speed)
const float LAUNCH_GAIN       = 1.6f;      // % of extra throttle per degree of uphill slope
const float HILL_HOLD_DEG     = 5.0f;      // hold the brake on slopes steeper than this
const unsigned long STUCK_MS  = 1200;      // throttle on, no movement, for this long = stuck

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
float diffGain = 1.0f;              // differential strength, 1.0 = exact geometry (set live from the app)
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

// Assists (switched from the app with T)
bool collisionOn = true, evadeOn = true, wallKeepOn = true, hillHoldOn = true, absOn = true, stuckOn = true, launchOn = true;
float gapScale = 1.0f;               // J: following gap scale
uint16_t steerReturnMs = 500;        // U: how long steering takes to return to center after release
bool brakeInput = false;             // swipe-to-brake from the app (third number of D)

// Motion estimate
float kSpeed = SPEED_CMS_PER_PCT;    // cm/s per 1 % throttle (learned)
bool  fwdValid = false;              // true once the MPU's forward direction is known
float fwdCos = 1, fwdSin = 0;        // car's forward direction in the MPU's x/y plane
float restX = 0, restY = 0;          // slow average of gravity on the MPU's x/y (g)
float aDynX = 0, aDynY = 0, aFwd = 0;// acceleration with gravity removed (g)
float vImu = 0;                      // forward speed from integrating the accelerometer (cm/s, leaky)
float vSonar = 0;                    // closing speed on whatever is straight ahead (cm/s, + = closing)
float vOwn = 0;                      // best guess of the car's own forward speed (cm/s)
float slopeDeg = 0;                  // + = nose up
uint16_t lastCenterCm = 999; unsigned long lastCenterMs = 0;

// Collision system outputs
float capPct = 100;                  // speed limit from the collision system and ACC (0..100 %)
bool hardBrake = false, evading = false, hillHolding = false, stuck = false, driftFlag = false, autoGaveUp = false;
int8_t evadeDir = 0; unsigned long evadeUntil = 0, evadeLockout = 0, hardBrakeUntil = 0;
float steerEased = 0, steerNow = 0;

// Motion calibration (C,2)
uint8_t calStep = 0;                 // 0 idle, 1 checking, 2 driving, 3 stopping, 4 measuring
uint8_t calState = 0;                // 0 idle, 1 running, 2 done, 3 no wall 50-400 cm ahead, 4 aborted or failed, 5 odd result
unsigned long calT0 = 0;
float calD0 = 0, calInt = 0, calMx = 0, calMy = 0; uint8_t calN = 0;

// Roam escape ladder
uint8_t roamBackCount = 0; unsigned long roamBackWindow = 0;

// Buzzer
bool buzzOn = false, buzzLevel = false; unsigned long buzzUs = 0;

// Bluetooth line buffer
const uint8_t RX_BUF = 24;
char rxBuf[RX_BUF]; uint8_t rxLen = 0;

// =====================================================================
//  SMALL HELPERS
// =====================================================================
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float maxf(float a, float b) { return a > b ? a : b; }
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

// Open-loop electronic differential from Ackermann geometry.
//   Turn radius of the rear axle centre:  R = L / tan(d)            (L = wheelbase, d = front wheel angle)
//   Each rear wheel travels on a circle R -/+ T/2 around the same centre (T = track), so
//   wheel speed = v * (1 -/+ (T / (2 R))) = v * (1 -/+ (T / (2 L)) * tan(d))
// v is the speed of the axle centre, so the car keeps its speed: the inner wheel slows by the same
// amount the outer wheel speeds up. If the outer wheel would pass 100 %, both are scaled down together
// so the ratio (and so the turning circle) stays right.
// steerPct is -100..100 (+ = right), diffGain is the live tuning factor from the app (1.0 = pure geometry).
// At low speed the effect is boosted (tight, almost pivoting turns), fading out above 40 % speed.
void differentialOut(float v, float steerPct, float &left, float &right) {
  if (steerPct > -0.5f && steerPct < 0.5f) { left = v; right = v; return; }
  static float lastSteer = 1000.0f, lastTan = 0.0f;
  if (fabs(steerPct - lastSteer) > 0.25f) {                       // tan() is slow on an AVR, only redo it when the steering moved
    lastSteer = steerPct;
    lastTan = tan(steerPct * 0.01f * WHEEL_LOCK_DEG * 0.0174533f);
  }
  float lowSpeed = clampf((40.0f - fabs(v)) / 30.0f, 0.0f, 1.0f);   // 1 when crawling, 0 above 40 % speed
  float k = (TRACK_CM / (2.0f * WHEELBASE_CM)) * lastTan * diffGain * (1.0f + LOW_SPEED_DIFF_BOOST * lowSpeed);
  if (k > 0.95f) k = 0.95f;                                       // the inner wheel may nearly stop but never reverses
  if (k < -0.95f) k = -0.95f;
  left  = v * (1.0f + k);                                         // right turn (k > 0): left wheel is the outer one
  right = v * (1.0f - k);
  float m = fabs(left) > fabs(right) ? fabs(left) : fabs(right);
  if (m > 100.0f) { left *= 100.0f / m; right *= 100.0f / m; }
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
  if (s == 0) {                                      // closing speed on whatever is straight ahead
    unsigned long nowMs = millis();
    if (lastCenterMs != 0 && value < 400 && lastCenterCm < 400) {
      float dt = (nowMs - lastCenterMs) * 0.001f;
      if (dt > 0.02f && dt < 0.4f) {
        float v = ((float)lastCenterCm - (float)value) / dt;
        if (v > -250.0f && v < 250.0f) vSonar += 0.2f * (v - vSonar);
      }
    } else vSonar *= 0.8f;
    lastCenterCm = value; lastCenterMs = nowMs;
  }
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

// ---- saved motion calibration (EEPROM, separate from the level trim) ----
struct MotionCal { uint8_t magic; int16_t cosX1000; int16_t sinX1000; int16_t kX1000; };
const uint8_t MCAL_MAGIC = 0xB1;
const int MCAL_ADDR = 16;

void saveMotionCal() {
  MotionCal m;
  m.magic = MCAL_MAGIC;
  m.cosX1000 = (int16_t)(fwdCos * 1000.0f);
  m.sinX1000 = (int16_t)(fwdSin * 1000.0f);
  m.kX1000 = (int16_t)(kSpeed * 1000.0f);
  EEPROM.put(MCAL_ADDR, m);
}

void loadMotionCal() {
  MotionCal m;
  EEPROM.get(MCAL_ADDR, m);
  if (m.magic != MCAL_MAGIC) return;
  float c = m.cosX1000 / 1000.0f, s = m.sinX1000 / 1000.0f, k = m.kX1000 / 1000.0f;
  float mag = sqrt(c * c + s * s);
  if (mag < 0.9f || mag > 1.1f || k < 0.3f || k > 2.0f) return;   // garbage: ignore
  fwdCos = c / mag; fwdSin = s / mag; kSpeed = k; fwdValid = true;
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

  // ---- motion estimate: acceleration without gravity, forward speed, slope ----
  static bool restInit = false;
  if (!restInit) { restX = fx; restY = fy; restInit = true; }
  restX += (fx - restX) * 0.01f;                     // ~1 s average = how gravity falls on x and y
  restY += (fy - restY) * 0.01f;
  aDynX = fx - restX; aDynY = fy - restY;
  if (fwdValid) {
    float aMag = sqrt(aDynX * aDynX + aDynY * aDynY);
    aFwd = aDynX * fwdCos + aDynY * fwdSin;
    vImu += aFwd * 981.0f * dt;                      // g -> cm/s^2
    vImu -= vImu * 0.4f * dt;                        // leak, so sensor drift can't build up
    if (speedCur == 0 && aMag < 0.03f && fabs(gyroZ) < 2.0f) vImu -= vImu * 8.0f * dt;   // standing still: pull to zero
    float rf = clampf(restX * fwdCos + restY * fwdSin, -0.7f, 0.7f);                    // gravity along the car's forward axis
    slopeDeg = atan2(rf, sqrt(1.0f - rf * rf)) * 57.2958f;
  } else { aFwd = 0; vImu = 0; slopeDeg = 0; }

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
// Roam escape ladder when the front is blocked:
//   1) a tight forward turn if there is room (the differential is strong at low speed),
//   2) otherwise a short, slow reverse (blind: there is no rear sensor),
//   3) after two reverses in a row, stop and ask the driver to take over.
void roamEscape(unsigned long now, float dC, float dL, float dR) {
  roamDir = (dR >= dL) ? 1 : -1;                              // toward the more open side
  bool forwardRoom = (dC > 18.0f) && (maxf(dL, dR) > 35.0f);
  if (forwardRoom) {
    roamState = ROAM_TURN; roamUntil = now + ROAM_TURN_MS;
    return;
  }
  if (now - roamBackWindow > 8000UL) { roamBackWindow = now; roamBackCount = 0; }
  if (roamBackCount >= 2) { autoGaveUp = true; autoMode = 0; holdSpeed = false; return; }
  roamBackCount++;
  roamState = ROAM_BACK; roamUntil = now + ROAM_BACK_MS;
}

void autoDrive(float &t, float &s) {
  unsigned long now = millis();
  float dC = distF(0), dL = distF(1), dR = distF(2);
  t = 0; s = 0;

  switch (autoMode) {
    case 1:   // ROAM: drive, steer away from walls, turn forward when possible, back up only as a last resort
      switch (roamState) {
        case ROAM_GO: {
          if (frontBlocked(ROAM_SPEED_PCT, true)) { roamEscape(now, dC, dL, dR); break; }
          float slow = clampf((dC - 25.0f) / 55.0f, 0.5f, 1.0f);
          t = ROAM_SPEED_PCT * slow;
          s = clampf((minf(dR, 100.0f) - minf(dL, 100.0f)) * 0.9f, -100.0f, 100.0f);
          break;
        }
        case ROAM_BACK:   // slow, short reverse with the wheels turned so the nose swings to the open side
          t = -ROAM_BACK_PCT; s = -roamDir * 100.0f;
          if (now >= roamUntil || gMag > 1.5f) { roamState = ROAM_TURN; roamUntil = now + ROAM_TURN_MS; }   // a bump ends the reverse at once
          break;
        case ROAM_TURN:   // forward arc toward the open side
          t = ROAM_TURN_PCT; s = roamDir * 100.0f;
          if (frontBlocked(ROAM_TURN_PCT, true)) roamEscape(now, dC, dL, dR);
          else if (now >= roamUntil) roamState = ROAM_GO;
          break;
      }
      break;

    case 2: {  // FOLLOW: keep a set distance from whatever is in front (backs off only when really close)
      if (dC > FOLLOW_LOSE_CM) break;
      float err = dC - FOLLOW_TARGET_CM;
      if (err > FOLLOW_DEADBAND_CM)  t = clampf(err * 2.0f, 0.0f, 45.0f);
      else if (err < -12.0f)         t = clampf(err * 0.8f, -10.0f, 0.0f);
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
//  MOTION, COLLISION SYSTEM AND ACC
// =====================================================================
// Yaw rate (deg/s, + = left turn) the car should have for this steering and speed:  w = v * tan(d) / L
float expectedYawDps(float steerPct, float speedPct) {
  float d = steerPct * 0.01f * WHEEL_LOCK_DEG * 0.0174533f;
  return -(speedPct * kSpeed / WHEELBASE_CM) * tan(d) * 57.2958f;
}

// Which side has room to swing into?  +1 right, -1 left, 0 none.
int8_t freeSide(float dC, float dL, float dR) {
  float l = dL >= 900.0f ? 300.0f : dL;
  float r = dR >= 900.0f ? 300.0f : dR;
  float best = (r >= l) ? r : l;
  int8_t dir = (r >= l) ? 1 : -1;
  if (best * (SIDE_SIN_PCT / 100.0f) < 14.0f) return 0;       // not enough sideways room for the car
  if (best < 45.0f || best < 1.5f * dC) return 0;             // a wall ahead also shows up on the side sonars
  return dir;
}

// Works on the car's real speed (throttle, coasting, rolling), not on the throttle.
// Outputs: capPct (speed limit), hardBrake, evading/evadeDir, blockedFront, vOwn.
void collisionUpdate(float steerIn, unsigned long now) {
  float dC = distF(0), dL = distF(1), dR = distF(2);
  float vModel = speedCur > 0 ? speedCur * kSpeed : 0;
  float vInertial = vImu > 0 ? vImu : 0;
  vOwn = vModel > vInertial ? vModel : vInertial;
  float vClose = vSonar > 0 ? vSonar : 0;
  if (vClose < vOwn * 0.85f) vClose = vOwn * 0.85f;           // the sonar rate is noisy: never let it sit below the car's own speed

  float gap = (GAP_BASE_CM + GAP_PER_CMS * vClose) * gapScale;
  float lat = vClose * SENSOR_LATENCY_S;

  // steering toward free space makes the road ahead look longer
  float dEff = dC;
  float w = clampf((fabs(steerIn) - STEER_CLEAR_MIN) / 45.0f, 0.0f, 1.0f);
  if (w > 0.0f) {
    float side = (steerIn > 0) ? dR : dL;
    if (side >= 900.0f) side = 300.0f;
    if (side * (SIDE_SIN_PCT / 100.0f) >= 14.0f) {
      float freeFwd = side * (SIDE_COS_PCT / 100.0f);
      if (freeFwd > dEff) dEff = dEff + w * (freeFwd - dEff);
    }
  }
  bool wide = dEff >= 900.0f;

  float cap = 100.0f;
  float aReq = 0.0f;
  if (collisionOn && !wide) {
    float avail = dEff - gap - lat;
    float vmax = sqrt(2.0f * COMFORT_DECEL * (avail > 0 ? avail : 0));
    cap = vmax / kSpeed;
    float room = dEff - GAP_MIN_CM - lat;
    aReq = (room <= 1.0f) ? 9999.0f : (vClose * vClose) / (2.0f * room);   // deceleration needed to stop before touching
  }

  // ACC: keep a gap behind whatever is ahead while a speed is held
  if (holdSpeed && autoMode == 0 && dC < 300.0f) {
    float vLead = vOwn - vClose; if (vLead < 0) vLead = 0;
    float gFollow = gap + FOLLOW_EXTRA_CM * gapScale;
    float vAcc = vLead + ACC_GAIN * (dC - gFollow);
    if (vAcc < 0) vAcc = 0;
    float c = vAcc / kSpeed;
    if (c < cap) cap = c;
  }
  if (cap > 100.0f) cap = 100.0f;
  if (cap < capPct) { capPct -= 8.0f; if (capPct < cap) capPct = cap; }      // drops fast, rises slowly
  else              { capPct += 2.0f; if (capPct > cap) capPct = cap; }

  // hard brake: slam when the crash is near-unavoidable, or when the car is gliding into something
  bool want = false;
  if (collisionOn && vClose > 10.0f) {
    if (aReq > BRAKE_DECEL * 0.8f) want = true;
    else if (speedCur < 8.0f && aReq > COMFORT_DECEL * 1.2f) want = true;
  }
  if (want) {
    if (!hardBrake && vOwn > 10.0f) obstacleFlashUntil = now + 700;
    hardBrake = true; hardBrakeUntil = now + 300;
  } else if (hardBrake && now > hardBrakeUntil && (vClose < 8.0f || aReq < COMFORT_DECEL * 0.5f)) {
    hardBrake = false;
  }
  if (!collisionOn) hardBrake = false;

  // emergency steering: braking alone can't stop the car, so swing toward free space
  int8_t fs = freeSide(dC, dL, dR);
  bool unavoidable = collisionOn && evadeOn && vClose > EVADE_MIN_CMS && aReq > BRAKE_DECEL && dC > 10.0f;
  bool driverSteersClear = (w >= 0.5f) && fs != 0 && ((steerIn > 0) == (fs > 0));
  if (evading) {
    bool driverOverride = (steerIn * evadeDir) < -60.0f;             // pushing the pad hard the other way takes it back
    if (driverOverride) evadeLockout = now + 1000;
    if (driverOverride || now > evadeUntil) { evading = false; evadeDir = 0; }
  } else if (unavoidable && fs != 0 && !driverSteersClear && now > evadeLockout) {
    evading = true; evadeDir = fs; evadeUntil = now + EVADE_MS;
  }

  blockedFront = collisionOn && !wide && (hardBrake || capPct < 8.0f);
}

// Stuck = throttle on but the car isn't going anywhere (blocked by something the sonar can't see, wheels in the air, ...)
void stuckUpdate(unsigned long now) {
  static unsigned long driveSince = 0, lastMoving = 0, sampleAt = 0, staticSince = 0;
  static float lastD = 0; static bool fromStill = false;
  if (fabs(speedCur) > 5.0f || fabs(vImu) > 8.0f) lastMoving = now;
  if (!stuckOn || stuck) { driveSince = 0; staticSince = 0; return; }
  if (speedCur < 25.0f) { driveSince = 0; staticSince = 0; return; }
  if (driveSince == 0) { driveSince = now; fromStill = (now - lastMoving > 400UL); staticSince = 0; sampleAt = now; lastD = distF(0); }

  bool isStuck = false;
  // 1) from a standstill the accelerometer must have picked up a decent speed within about a second
  if (fwdValid && fromStill && now - driveSince > 900UL && now - driveSince < 1500UL) {
    if (vImu < 0.3f * speedCur * kSpeed && fabs(gyroZ) < 8.0f) isStuck = true;
  }
  // 2) something close ahead whose distance doesn't change while the car is being driven at it
  if (now - sampleAt >= 400UL) {
    float d = distF(0);
    if (d < 150.0f && fabs(d - lastD) < 3.0f) { if (staticSince == 0) staticSince = now; }
    else staticSince = 0;
    lastD = d; sampleAt = now;
  }
  if (staticSince != 0 && now - staticSince > STUCK_MS && now - driveSince > STUCK_MS) isStuck = true;

  if (isStuck) { stuck = true; speedCur = 0; holdSpeed = false; autoMode = 0; driveSince = 0; staticSince = 0; }
}

// Car-drives-itself calibration (C,2): learns the MPU's forward direction and the cm per % of throttle.
void motionCalTick(unsigned long now, float &tgtT, float &tgtS, bool &braking) {
  if (calStep == 0) return;
  tgtS = 0;
  if (abs(cmdThrottle) > 10 || flipped || linkLost) { calStep = 0; calState = 4; tgtT = 0; return; }
  if (calStep == 1) {                                          // still, and a wall 50..400 cm ahead?
    tgtT = 0;
    if (now - calT0 > 500UL) {
      float d = distF(0);
      if (d < 50.0f || d >= 400.0f) { calState = 3; calStep = 0; return; }
      calD0 = d; calInt = 0; calMx = 0; calMy = 0; calN = 0;
      calStep = 2; calT0 = now;
    }
  } else if (calStep == 2) {                                   // drive straight at 45 %
    float d = distF(0);
    tgtT = 45.0f;
    calInt += speedCur * 0.01f;                                // % x seconds
    unsigned long t = now - calT0;
    if (t > 40UL && t < 450UL) {
      float m = sqrt(aDynX * aDynX + aDynY * aDynY);
      if (m > 0.03f && calN < 250) { calMx += aDynX; calMy += aDynY; calN++; }
    }
    if ((calD0 - d >= 25.0f) || d < 22.0f || t > 1100UL) { calStep = 3; calT0 = now; }
  } else if (calStep == 3) {                                   // brake and let it settle
    tgtT = 0; braking = true;
    if (now - calT0 > 900UL) { calStep = 4; }
  } else if (calStep == 4) {
    tgtT = 0;
    float dEnd = distF(0);
    float travelled = calD0 - dEnd;
    float mag = sqrt(calMx * calMx + calMy * calMy);
    calStep = 0;
    if (calN < 5 || mag < 0.04f * calN || calInt < 5.0f || travelled < 8.0f) { calState = 5; return; }
    float k = travelled / calInt;                              // cm per (% x s)
    if (k < 0.3f || k > 2.0f) { calState = 5; return; }
    fwdCos = calMx / mag; fwdSin = calMy / mag; fwdValid = true;
    kSpeed = k; vImu = 0;
    saveMotionCal();
    calState = 2;
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
    cmdThrottle = 0; cmdSteer = 0; brakeInput = false;
  }
  faultPrev = fault;
  if (fault) needNeutral = true;
  else if (needNeutral && abs(cmdThrottle) < 5) needNeutral = false;
  if (stuck && abs(cmdThrottle) < 5) stuck = false;            // let go of the throttle to clear "stuck"
  if (brakeInput) {                                            // swipe-to-brake also cancels hold speed and autonomy
    holdSpeed = false; autoMode = 0; needNeutral = true;
  }

  // ---- target throttle and steering ----
  float tgtT = 0, tgtS = 0;
  float maxPct = MODES[driveMode].maxPct;
  if (limp && maxPct > LIMP_MAX_PCT) maxPct = LIMP_MAX_PCT;

  if (!fault) {
    if (autoMode == 0) {
      tgtS = cmdSteer;
      if (!needNeutral && !stuck) {
        tgtT = holdSpeed ? holdValue : cmdThrottle;
        tgtT *= maxPct / 100.0f;
      }
    } else {
      autoDrive(tgtT, tgtS);
      tgtT = clampf(tgtT, -maxPct, maxPct);
    }
  }

  bool calBraking = false;
  if (!fault) motionCalTick(now, tgtT, tgtS, calBraking);

  // ---- launch assist: extra throttle on an uphill start ----
  if (launchOn && fwdValid && tgtT > 2.0f && slopeDeg > 3.0f && speedCur < 35.0f) {
    float boost = clampf(slopeDeg * LAUNCH_GAIN, 0.0f, 30.0f) * (1.0f - speedCur / 35.0f);
    tgtT = minf(tgtT + boost, maxPct);
  }

  // ---- collision system, ACC, stuck detection ----
  collisionUpdate(tgtS, now);
  if (tgtT > capPct) tgtT = capPct;
  stuckUpdate(now);
  if (stuck) { tgtT = 0; }
  bool brakeActive = brakeInput || hardBrake || calBraking;

  // ---- ramp ----
  if (brakeActive || now < brakeUntil) {
    speedCur = 0;
    hhActive = false;
  } else {
    float step = (rampMs == 0) ? 100.0f : 1000.0f / rampMs;       // % per 10 ms
    float st = step;
    if (fabs(tgtT) < fabs(speedCur) && tgtT * speedCur >= 0) st = step * 2.0f;   // slow down faster
    if (speedCur < tgtT)      { speedCur += st; if (speedCur > tgtT) speedCur = tgtT; }
    else if (speedCur > tgtT) { speedCur -= st; if (speedCur < tgtT) speedCur = tgtT; }
  }

  // ---- steering: ease back to center, speed-sensitive, gyro and wall assists, emergency steering ----
  if (autoMode == 0 && !evading) {
    if (steerReturnMs > 0 && fabs(tgtS) < fabs(steerEased) && tgtS * steerEased >= 0) {
      float stp = 1000.0f / steerReturnMs;                         // % per 10 ms: 100 % in steerReturnMs
      if (steerEased > tgtS) { steerEased -= stp; if (steerEased < tgtS) steerEased = tgtS; }
      else                   { steerEased += stp; if (steerEased > tgtS) steerEased = tgtS; }
    } else steerEased = tgtS;
    tgtS = steerEased;
  } else steerEased = tgtS;

  float f = fabs(speedCur) / 100.0f;
  float scale = 1.0f - (1.0f - MODES[driveMode].minSteerPct / 100.0f) * f;
  float steerOut = tgtS * scale;
  float corr = 0;
  float dirSign = (speedCur < 0) ? -1.0f : 1.0f;

  if (autoMode == 0 && !fault && imuOk && fabs(speedCur) > 15.0f && !evading) {
    bool driverSteering = fabs(tgtS) >= 8.0f;
    if (headingHold && !driverSteering) {
      if (!hhActive) { hhActive = true; hhTarget = yawDeg; }
      corr += dirSign * clampf(HH_KP * (yawDeg - hhTarget) + HH_KD * gyroZ, -HH_MAX, HH_MAX);
    } else hhActive = false;

    if (stabAssist && driverSteering) {
      float expected = expectedYawDps(steerOut, speedCur);
      float excess = gyroZ - expected;
      if (fabs(excess) > STAB_DEADBAND_DPS) {
        float e = (excess > 0) ? excess - STAB_DEADBAND_DPS : excess + STAB_DEADBAND_DPS;
        corr += dirSign * clampf(STAB_KP * e, -STAB_MAX, STAB_MAX);
      }
    }
  } else hhActive = false;

  if (autoMode == 0 && !fault && wallKeepOn && !evading && vOwn > 8.0f) corr += wallAssist();   // works while coasting too

  steerOut = clampf(steerOut + corr, -100.0f, 100.0f);
  if (evading) { steerOut = evadeDir * 100.0f; steerEased = steerOut; }
  steerNow = steerOut;
  steeringServo.write((int)(SERVO_CENTER + SERVO_DIR * steerOut * SERVO_RANGE / 100.0f));

  // ---- turn check: turning with straight steering, or not turning when asked to ----
  static unsigned long mismatchSince = 0;
  if (fabs(speedCur) > 20.0f && imuOk && !evading) {
    float yawExp = expectedYawDps(steerOut, speedCur);
    bool bad = (fabs(steerOut) < 6.0f && fabs(gyroZ) > 15.0f) || (fabs(yawExp) > 30.0f && fabs(gyroZ) < 0.35f * fabs(yawExp));
    if (bad) { if (mismatchSince == 0) mismatchSince = now; } else mismatchSince = 0;
  } else mismatchSince = 0;
  driftFlag = (mismatchSince != 0 && now - mismatchSince > 600UL);

  // ---- hill hold: stay put on a slope when the driver isn't asking for anything ----
  bool onSlope = fabs(slopeDeg) > (hillHolding ? HILL_HOLD_DEG * 0.7f : HILL_HOLD_DEG);
  hillHolding = hillHoldOn && fwdValid && autoMode == 0 && !fault && !brakeActive &&
                fabs(speedCur) < 1.0f && fabs(tgtT) < 3.0f && onSlope;

  // ---- motor output ----
  if (brakeActive || now < brakeUntil) {
    bool on = true;                                                // solid short brake...
    if (evading)                                  on = (now % 60) < 40;     // ...but pulsed while swerving, so the car can still steer
    else if (absOn && fabs(gyroZ) > ABS_YAW_DPS)  on = (now % 80) >= 30;   // ...and pulsed when the gyro sees a spin
    if (on) brakeMotors(); else setMotors(0, 0);
  } else if (hillHolding) {
    brakeMotors();
  } else {
    float left, right;
    differentialOut(speedCur, steerOut, left, right);
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

// Passive buzzers only click on plain DC, they need a tone. buzzerTick() makes ~2.5 kHz by toggling the pin;
// it is called many times per loop. (tone() is not usable: it takes Timer2, which drives the motor pins D3 and D11.)
void buzzerWrite(bool on) {
#if HAS_BUZZER
  buzzOn = on;
  #if BUZZER_TYPE == 0
    digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
  #elif BUZZER_TYPE == 1
    digitalWrite(PIN_BUZZER, on ? LOW : HIGH);
  #else
    if (!on && buzzLevel) { digitalWrite(PIN_BUZZER, LOW); buzzLevel = false; }
  #endif
#else
  (void)on;
#endif
}

void buzzerTick() {
#if HAS_BUZZER && BUZZER_TYPE == 2
  if (!buzzOn) return;
  unsigned long n = micros();
  if (n - buzzUs >= 200UL) {
    buzzUs = n;
    buzzLevel = !buzzLevel;
    digitalWrite(PIN_BUZZER, buzzLevel ? HIGH : LOW);
  }
#endif
}

void lightsUpdate() {
  unsigned long now = millis();
  bool warn = false, warnOn = false, hazardOnly = false;

  if (flipped)                          { warn = true; warnOn = (now % 120) < 60; }                 // strobe
  else if (linkLost || cutoff)          { warn = true; warnOn = (now % 200) < 100; }                // fast blink
  else if (evading)                     { warn = true; warnOn = (now % 120) < 60; }                 // swerving: strobe
  else if (now < obstacleFlashUntil)    { warn = true; warnOn = (now % 200) < 100; }                // 3 quick flashes
  else if (stuck || autoGaveUp)         { warn = true; warnOn = (now % 400) < 200; }                // stuck: slow urgent blink
  else if (limp)                        { warn = true; unsigned long m = now % 2000; warnOn = (m < 80) || (m >= 200 && m < 280); }  // double blink
  else if (hazard)                      { warn = true; hazardOnly = true; warnOn = (now % 1000) < 500; }

  bool reverseBeep = (autoMode != 0 && speedCur < -2.0f && (now % 500) < 250);   // the car is backing up blind
  if (warn) headlightWrite(warnOn, 100);                // warnings always full brightness
  else      headlightWrite(lightsUser, headBrightness);
  buzzerWrite(hornOn || reverseBeep || (warn && warnOn && !hazardOnly));
}

// =====================================================================
//  BLUETOOTH
// =====================================================================
void handleLine(char *line) {
  char cmd = line[0];
  long a = 0, b = 0, c = 0;
  char *p = strchr(line, ',');
  if (p) {
    a = atol(p + 1);
    p = strchr(p + 1, ',');
    if (p) { b = atol(p + 1); p = strchr(p + 1, ','); if (p) c = atol(p + 1); }
  }

  bool known = true;
  switch (cmd) {
    case 'D': cmdThrottle = clampPct(a); cmdSteer = clampPct(b); brakeInput = (c != 0); break;
    case 'X': cmdThrottle = 0; holdSpeed = false; emergencyStop(300); break;
    case 'M': driveMode = (uint8_t)(a < 0 ? 0 : (a > 2 ? 2 : a)); rampMs = MODES[driveMode].rampMs; break;
    case 'A': autoMode = (uint8_t)(a < 0 ? 0 : (a > 3 ? 3 : a)); roamState = ROAM_GO; holdSpeed = false;
              autoGaveUp = false; roamBackCount = 0; break;
    case 'K':
      if (a && autoMode == 0 && cmdThrottle > 0) { holdSpeed = true; holdValue = cmdThrottle; }   // ACC holds forward speed only
      else holdSpeed = false;
      break;
    case 'L': lightsUser = (a != 0); break;
    case 'B': headBrightness = (uint8_t)(a < 0 ? 0 : (a > 100 ? 100 : a)); break;
    case 'H': hazard = (a != 0); break;
    case 'V': hornOn = (a != 0); break;
    case 'R': rampMs = (uint16_t)(a < 0 ? 0 : (a > 3000 ? 3000 : a)); break;
    case 'S': stabAssist = (a != 0); break;
    case 'G': headingHold = (a != 0); hhActive = false; break;
    case 'F': diffGain = (float)(a < 0 ? 0 : (a > 150 ? 150 : a)) / 100.0f; break;   // differential strength in %
    case 'T':                                                                         // assist switches
      collisionOn = (a & 1) != 0; evadeOn = (a & 2) != 0; wallKeepOn = (a & 4) != 0; hillHoldOn = (a & 8) != 0;
      absOn = (a & 16) != 0; stuckOn = (a & 32) != 0; launchOn = (a & 64) != 0;
      if (!stuckOn) stuck = false;
      break;
    case 'U': steerReturnMs = (uint16_t)(a < 0 ? 0 : (a > 2000 ? 2000 : a)); break;
    case 'J': gapScale = (float)(a < 50 ? 50 : (a > 200 ? 200 : a)) / 100.0f; break;
    case 'Z': if (speedCur == 0 && imuOk) imuCalibrate(false); break;
    case 'C':
      if (strchr(line, ',') == NULL) break;                            // ignore a damaged "C" with no value
      if (a == 0) { rollTrim = 0; pitchTrim = 0; saveCal(); }          // C,0 clears the saved level trim
      else if (a == 2) {                                               // C,2 motion calibration: the car drives itself ~30 cm
        if (calStep == 0 && speedCur == 0 && imuOk && autoMode == 0 && !holdSpeed && !linkLost && !flipped && !brakeInput) {
          calStep = 1; calState = 1; calT0 = millis();
        }
      }
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
  if (now - lastTelMs < 100) return;
  lastTelMs = now;
  if (!linkSeen) return;
  static uint8_t phase = 0;
  uint8_t ph = phase;
  phase = (phase + 1) % 3;

  if (ph == 0) {
    uint16_t flags = (linkLost ? 1 : 0) | (limp ? 2 : 0) | (cutoff ? 4 : 0) | (flipped ? 8 : 0) |
                     (blockedFront ? 16 : 0) | (crashFlag ? 32 : 0) | (holdSpeed ? 64 : 0) |
                     (stabAssist ? 128 : 0) | (headingHold ? 256 : 0) | (stuck ? 512 : 0) |
                     (evading ? 1024 : 0) | (hardBrake || brakeInput ? 2048 : 0) |
                     (capPct < 99.0f ? 4096 : 0) | (hillHolding ? 8192 : 0) | (driftFlag ? 16384 : 0) |
                     (autoGaveUp ? 32768U : 0);
    BTPORT.print(F("S,"));  BTPORT.print(distF(0));
    BTPORT.print(',');      BTPORT.print(distF(1));
    BTPORT.print(',');      BTPORT.print(distF(2));
    BTPORT.print(',');      BTPORT.print(driveMode);
    BTPORT.print(',');      BTPORT.print(autoMode);
    BTPORT.print(',');      BTPORT.println(flags);
  } else if (ph == 1) {
    BTPORT.print(F("I,"));  BTPORT.print((int)roll);
    BTPORT.print(',');      BTPORT.print((int)pitch);
    BTPORT.print(',');      BTPORT.print((int)yawDeg);
    BTPORT.print(',');      BTPORT.print((int)(gMag * 10.0f));
    BTPORT.print(',');      BTPORT.print((int)(battV * 100.0f));
    BTPORT.print(',');      BTPORT.println((int)speedCur);
  } else {
    BTPORT.print(F("M,"));  BTPORT.print((int)vOwn);
    BTPORT.print(',');      BTPORT.print((int)vSonar);
    BTPORT.print(',');      BTPORT.print((int)gyroZ);
    BTPORT.print(',');      BTPORT.print((int)expectedYawDps(steerNow, speedCur));
    BTPORT.print(',');      BTPORT.print((int)(slopeDeg * 10.0f));
    BTPORT.print(',');      BTPORT.print((int)capPct);
    BTPORT.print(',');      BTPORT.print(calState);
    BTPORT.print(',');      BTPORT.println(fwdValid ? 1 : 0);
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
  loadMotionCal();

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

  buzzerWrite(true);                               // short chirp: proves the buzzer works
  for (unsigned long t0 = millis(); millis() - t0 < 120UL; ) buzzerTick();
  buzzerWrite(false);

  sonarT = micros();
  DBGLN(F("RC car ready."));
}

void loop() {
  readBluetooth();
  buzzerTick();
  sonarUpdate();
  buzzerTick();

  unsigned long now = millis();
  if (now - lastImuMs >= 10)  { lastImuMs = now;  imuUpdate(); }
  buzzerTick();
  if (now - lastCtrlMs >= 10) { lastCtrlMs = now; controlTick(); }
  buzzerTick();
  if (now - lastBattMs >= 100){ lastBattMs = now; batteryUpdate(); }

  lightsUpdate();
  buzzerTick();
  telemetryUpdate();
  buzzerTick();

#if !USE_HW_SERIAL_BT
  if (now - lastDbgMs >= 1000) {
    lastDbgMs = now;
    DBG(F("batt=")); DBG(battV); DBG(F(" C/L/R=")); DBG(distF(0)); DBG('/'); DBG(distF(1)); DBG('/'); DBGLN(distF(2));
  }
#endif
}
