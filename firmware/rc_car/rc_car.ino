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
   - Collision system that works from the car's real speed (throttle, coasting, rolling, even a gentle push):
     smooth speed limit, steer-around allowance, early steer-around when there is room, hard brake,
     emergency steering that hands back to the driver
   - ACC: hold speed that keeps a gap behind whatever is ahead; wall keeping; hill hold and launch assist
   - IMU motion sensing: real speed, stuck cut-off, turn check, learned forward direction (Calibrate > Motion)
   - Held brake (swipe to brake); ABS per drive mode: pulses the brake while fast and spinning, solid brake once slow
   - Traction control on launch (strength per drive mode), using the accelerometer
   - Sonar master switch; top speed per autonomous mode
   - Calibrate > Motion and braking: a 2 m run at full power, then a hard stop (learns forward direction, speed
     scale and the real braking rate)
   - Steering eases back to center after release
   - Non-blocking sonar reads
   - Autonomous modes Roam / Follow / Corridor: the logic runs in the phone app (it reads the sonar line and drives with D),
     the car keeps its own collision system, failsafe and stuck cut-off on top. No phone = no autonomy (and the car stops).
   - Low-battery limp mode and battery cutoff
   - Warning flash patterns on the headlights, hazard lights
   - Start-up self-test reported by headlight flashes
   - Buzzer (active or passive, see BUZZER_TYPE)

  PHONE PROTOCOL  (text lines ending in \n, 9600 baud by default)
  App -> car (any valid command also counts as a heartbeat, send something at least every 200 ms):
    D,<throttle>,<steer>[,<brake>]  throttle -100..100 (+ forward), steer -100..100 (+ right), brake 1 = hold the short brake
    X                      emergency brake
    M,<0|1|2>              drive mode: 0 Eco, 1 Normal, 2 Sport
    A,<0|1|2|3>[,<gaveUp>] 0 manual, 1 Roam, 2 Follow, 3 Corridor. Only tells the car an auto mode is active (the app then
                           drives with D). A,0,1 = the app's autopilot gave up (flag 32768)
    K,<0|1>                hold speed on/off (locks the current throttle, steering stays live)
    L,<0|1>                headlights
    B,<0-100>              headlight brightness in % (software PWM on D2; warning flashes stay full brightness)
    H,<0|1>                hazard lights
    V,<0|1>                horn (needs a buzzer)
    R,<ms>                 ramp time to full throttle in ms (0 = instant / launch mode)
    S,<0|1>                gyro stability assist on/off
    G,<0|1>                straight-line heading hold on/off
    F,<0-150>              electronic differential strength in % (100 = exact geometry, not saved)
    T,<mask>               assists on/off: 1 collision, 2 emergency/early steer-around, 4 wall keeping, 8 hill hold,
                           16 ABS master, 32 stuck cut-off, 64 launch assist
    Y,<mask>               ABS per drive mode: bit 0 Eco, bit 1 Normal, bit 2 Sport (default 6: Normal and Sport)
    Q,<0-100>              traction control strength (a mode change resets it: Eco 0, Normal 30, Sport 80)
    O,<0|1>                sonars on/off (off: no distance reading, no collision system, no autonomous modes)
    E,...                  ignored (auto top speed is kept in the app)
    U,<ms>                 how long the steering takes to return to center after release (0 = instant)
    J,<50-200>             following gap scale in %
    Z                      re-zero gyro drift and yaw (car must be stationary)
    C,<1|0|2>              1 = level the tilt readings and zero the gyro, saved to EEPROM (car still, on a flat surface)
                           0 = clear the saved level trim
                           2 = motion and braking calibration: with a wall 280-400 cm straight ahead the car drives
                               about 2 m at full power, then brakes hard. It learns the MPU's forward direction,
                               its speed scale and its real braking rate (saved)
    N                      flip the yaw direction (saved to EEPROM)
    P                      ping (heartbeat only)
  Car -> app (one line every 50 ms: S every 100 ms, I every 200 ms, M and W in between):
    S,<distCenter>,<distLeft>,<distRight>,<driveMode>,<autoMode>,<flags>
    I,<roll>,<pitch>,<yawDeg>,<gForce x10>,<battery volts x100>,<speed %>
    W,<launch push cm/s2>,<ms for the 70-190 cm stretch>,<cm/s per % x100 measured>,<braking rate measured>,<ms braking hard>
      (only after a calibration has run; these are the raw numbers, also when the run failed)
    M,<own speed cm/s>,<closing speed cm/s>,<yaw rate dps>,<expected yaw rate dps>,<slope deg x10>,<speed cap %>,<calState>,
      <forwardKnown>,<calWhy>,<braking rate cm/s2>,<cm/s per % x100>,<flags2>
  flags bitmask: 1 failsafe, 2 limp, 4 battery cutoff, 8 flipped, 16 front blocked,
                 32 crash, 64 hold speed, 128 stability assist, 256 heading hold,
                 512 stuck, 1024 emergency steering, 2048 hard braking, 4096 speed limited,
                 8192 hill hold, 16384 drifting or slipping, 32768 autonomous mode gave up
  flags2: 1 traction control working, 2 ABS pulsing, 4 steering around, 8 sonars on, 16 braking for a roll or push
  calState: 0 idle, 1 running, 2 done, 3 no wall 280-400 cm ahead, 4 aborted, 5 failed (see calWhy)
  calWhy: 1 too little push felt by the accelerometer, 2 did not cover 2 m in time (wheels slipping or too weak),
          3 speed scale out of range, 4 braking rate out of range, 5 safety stop (wall under 60 cm),
          6 stopped by throttle, brake, link loss or flip, 7 stopping distance too short to measure

  START-UP SELF-TEST (headlight flashes): one long flash = all good.
    2 short = IMU not found, 3 short = a sonar saw no echo, 4 short = battery voltage odd.

  BATTERY SENSE (optional, HAS_BATT_SENSE)
   The HX-2S-D20 BMS has no signal wire, so the Arduino cannot read it. To show the battery voltage,
   wire pack + (after the power switch) -> 10k -> A6 -> 10k -> GND, then set HAS_BATT_SENSE to 1.
   A full 2S pack (8.4 V) gives 4.2 V at A6. Do NOT set it to 1 with nothing on A6: a floating pin
   makes the car think the battery is dead (cutoff, throttle blocked, warning beeps).

  NOTES
   - There is no rear sensor, so reversing is blind. The app's Roam tries a forward turn first and reverses only
     briefly and slowly (with beeps), and gives up after two reverses in a row.
   - FLASH: the Nano has 30720 bytes. LIGHT_DRIVERS 1 (own I2C + servo code), cheap tan/atan and moving the autonomous
     modes into the app are what make this fit. If the IDE still says too big, tell me the 'Sketch uses N bytes' line.
   - The collision system is best effort: the sonar beams are narrow and soft or angled surfaces can miss the echo.
     Speeds are estimated. Calibrate > Motion and braking measures the real braking rate; BRAKE_DECEL and
     COMFORT_DECEL are only the starting guesses. Tune the gaps on your own floor.
   - SoftwareSerial (D9/D10) blocks interrupts while it sends or receives a byte, which can make the
     servo twitch and slows sonar timing. It is great for bench work with USB debugging. For the
     final car, set USE_HW_SERIAL_BT 1 (move the Bluetooth module back to the shield UART socket,
     and unplug it when uploading), or raise the HC-05 to 38400 baud and set BT_BAUD to match.
   - Steering: positive = right. If the wheels turn the wrong way, set SERVO_DIR to -1.
   - First tests: wheels off the ground, check each motor direction (INVERT_LEFT / INVERT_RIGHT).
*/

#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// =====================================================================
//  USER CONFIG
// =====================================================================
#define USE_HW_SERIAL_BT 1     // 1 = Bluetooth on the shield UART socket (D0/D1). Unplug the module before uploading.
                               // 0 = needs #include <SoftwareSerial.h> added at the top, which costs about 1 KB of flash (not included: flash is tight)
#define HAS_BUZZER       1     // active buzzer wired to D12 (0 = no buzzer)
#define BUZZER_TYPE      2     // 0 = active buzzer, beeps when I/O is HIGH
                               // 1 = active buzzer on a "low-level trigger" module (beeps when I/O is LOW)
                               // 2 = passive buzzer (only clicks on plain DC): the sketch makes a ~2.5 kHz tone itself
#define HAS_BATT_SENSE   0     // 1 = two 10k resistors wired from the pack + to A6 and from A6 to GND (see header).
                               // 0 = nothing on A6: battery readout, limp mode and cutoff are all switched off.
                               // Leaving A6 unwired with this set to 1 makes the car think the battery is dead.
#define LIGHT_DRIVERS    1     // 1 = tiny built-in I2C (MPU) and steering-servo drivers instead of the Wire and Servo libraries.
                               //     Saves about 2 KB of flash. The servo pulse comes from Timer1 on pin D4 (so D4 must stay the servo pin).
                               // (Do not add #include <Wire.h> or <Servo.h>: the Arduino IDE would link the Servo library and its Timer1 interrupt clashes with ours.)
const long BT_BAUD = 9600;     // 38400 is smoother if you set the HC-05 with AT+UART=38400,0,0


// Motor wiring tweaks
const bool SWAP_SIDES    = false;  // true if left and right are swapped
const bool INVERT_LEFT   = false;  // true if the left wheel spins backwards on forward
const bool INVERT_RIGHT  = false;
const int8_t SERVO_DIR   = 1;      // -1 if "steer right" turns the car left
const uint8_t SERVO_CENTER = 90;   // adjust to trim steering
const uint8_t SERVO_RANGE  = 40;   // degrees each side (50..130); was 35. Do not exceed what the linkage allows: if the servo
                                   // buzzes or strains at full lock, go back down a few degrees
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
const float WHEEL_LOCK_DEG = 34.0f;      // how far the front wheels really turn at full steering (30 at SERVO_RANGE 35). Measure it:
                                         // at full lock, turning-circle radius R (cm, from the middle of the rear axle)
                                         // gives WHEEL_LOCK_DEG = atan(17 / R) in degrees.

const uint8_t MOTOR_MIN_PWM = 35;        // lowest PWM that reliably turns the N20s

// Obstacles
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
const float ABS_SOLID_BELOW_CMS = 30.0f;   // ABS pulses the brake above this speed, full solid brake below it
const float PUSH_G            = 0.08f;     // a sudden push on the stationary car bigger than this (g) counts as a push
const float PUSH_BRAKE_CMS    = 5.0f;      // rolling toward something faster than this with nobody on the throttle = brake
const float TRACTION_LAG      = 0.055f;    // how fast the model of the car's own speed follows the throttle (per 10 ms)
const float LOW_SPEED_DIFF_BOOST = 1.5f;   // extra differential strength when crawling (2.5x at 10 % speed)
const float LAUNCH_GAIN       = 1.6f;      // % of extra throttle per degree of uphill slope
const float HILL_HOLD_DEG     = 5.0f;      // hold the brake on slopes steeper than this
const unsigned long STUCK_MS  = 1200;      // throttle on, no movement, for this long = stuck
const uint8_t MODE_TRACTION[3] = { 0, 30, 80 };   // traction control strength per drive mode (Eco, Normal, Sport)

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

#if !LIGHT_DRIVERS
Servo steeringServo;
#endif

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
float aFwdLP = 0;                    // forward acceleration, smoothed over ~30 ms (cm/s^2)
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
float evadeStrength = 100.0f; unsigned long evadeStart = 0;       // how hard the car steers on its own (100 = emergency)
float vSlow = 0;                                                   // closing speed over the last 0.5 s (slow but quiet, shows a gentle roll)
unsigned long pushUntil = 0;                                       // the accelerometer felt a push on the stationary car
bool coastBrake = false, absPulsing = false;
float brakeDecel = BRAKE_DECEL, comfortDecel = COMFORT_DECEL;      // real braking rate comes from Calibrate > Motion and braking
float brakeV0 = 0; unsigned long brakeT0 = 0; bool brakePrev = false;

// Switches and limits from the app
bool sonarOn = true;                                               // O: master switch for all three sonars
uint8_t absModes = 6;                                              // Y: ABS per drive mode (bit 0 Eco, 1 Normal, 2 Sport)
uint8_t tractionPct = MODE_TRACTION[1];                            // Q: traction control strength
float tractionCap = 100.0f;                                        // throttle limit while the wheels are spinning

// Motion calibration (C,2)
uint8_t calStep = 0;                 // 0 idle, 1 checking, 2 driving 2 m, 3 braking, 4 measuring
uint8_t calState = 0;                // 0 idle, 1 running, 2 done, 3 no wall 280-400 cm ahead, 4 aborted, 5 failed
uint8_t calWhy = 0;                  // reason for a failure, see the header
unsigned long calT0 = 0;
float calD0 = 0, calInt = 0, calMx = 0, calMy = 0; uint8_t calN = 0;
float calDa = 0, calIa = 0, calDb = 0, calIb = 0, calDs = 0; unsigned long calTa = 0, calTb = 0; uint8_t calFlags = 0;
float calFx = 1, calFy = 0; unsigned long calTpk = 0;
bool calHave = false; float calRk = 0, calRdec = 0; int calRpush = 0, calRrun = 0, calRbrk = 0;   // raw numbers of the last run, for Diagnostics (W line)     // forward axis from the launch, and the last moment the car was braking hard

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

static long lim(long v, long lo, long hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Small replacements for tan() and atan2(): the avr-libc versions pull in about 1 KB of flash.
// fastTan: Pade approximation, error under 0.1 % up to 45 degrees (x in radians).
static float fastTan(float x) {
  float x2 = x * x;
  return x * (15.0f - x2) / (15.0f - 6.0f * x2);
}
// atan2Deg: result in degrees, error under 0.3 degrees and exact near zero.
static float atan2Deg(float y, float x) {
  float ax = fabs(x), ay = fabs(y);
  if (ax < 1e-6f && ay < 1e-6f) return 0.0f;
  float r = (ax >= ay) ? ay / ax : ax / ay;
  float a = 57.2958f * r / (1.0f + 0.2808f * r * r);
  if (ax < ay) a = 90.0f - a;
  if (x < 0) a = 180.0f - a;
  return (y < 0) ? -a : a;
}

// ---- steering servo ----
#if LIGHT_DRIVERS
// Own servo driver: Timer1 counts 0.5 us ticks with a 20 ms period. The overflow-style interrupt (ICR1 = TOP) raises D4,
// the compare interrupt drops it. The new pulse length is only loaded at the start of a period, so it can never glitch.
static_assert(PIN_SERVO == 4, "LIGHT_DRIVERS drives the servo on D4 (PORTD bit 4); use LIGHT_DRIVERS 0 for another pin");
volatile uint16_t servoNextTicks = 3000;
ISR(TIMER1_CAPT_vect) { OCR1A = servoNextTicks; PORTD |= _BV(PD4); }
ISR(TIMER1_COMPA_vect) { PORTD &= ~_BV(PD4); }
static void servoBegin() {
  pinMode(PIN_SERVO, OUTPUT);
  TCCR1A = 0;
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);        // CTC with ICR1 as TOP, clock / 8
  ICR1 = 39999;                                         // 20 ms
  OCR1A = servoNextTicks;
  TIMSK1 = _BV(ICIE1) | _BV(OCIE1A);
}
#endif
static void steerWrite(int deg) {
  deg = (int)lim(deg, 0, 180);
#if LIGHT_DRIVERS
  uint16_t t = (uint16_t)(1088UL + (uint32_t)deg * 3712UL / 180UL);   // 544..2400 us, like Servo.write()
  uint8_t sreg = SREG; cli(); servoNextTicks = t; SREG = sreg;
#else
  steeringServo.write(deg);
#endif
}

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
  if (fabs(steerPct - lastSteer) > 0.25f) {                       // tan is slow on an AVR, only redo it when the steering moved
    lastSteer = steerPct;
    lastTan = fastTan(steerPct * 0.01f * WHEEL_LOCK_DEG * 0.0174533f);
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
  if (!sonarOn) return;
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

#if LIGHT_DRIVERS
// Minimal polled TWI (I2C) master at 400 kHz with timeouts, so a stuck bus can never freeze the car.
#define TWI_SPIN 12000
static bool twiWait() {
  uint16_t n = TWI_SPIN;
  while (!(TWCR & _BV(TWINT))) { if (--n == 0) return false; }
  return true;
}
static bool twiStart(uint8_t addrRw) {
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
  if (!twiWait()) return false;
  TWDR = addrRw;
  TWCR = _BV(TWINT) | _BV(TWEN);
  if (!twiWait()) return false;
  uint8_t st = TWSR & 0xF8;
  return st == 0x18 || st == 0x40;                   // address acknowledged (write / read)
}
static bool twiPut(uint8_t b) {
  TWDR = b;
  TWCR = _BV(TWINT) | _BV(TWEN);
  if (!twiWait()) return false;
  return (TWSR & 0xF8) == 0x28;
}
static bool twiGet(uint8_t *b, bool ack) {
  TWCR = _BV(TWINT) | _BV(TWEN) | (ack ? _BV(TWEA) : 0);
  if (!twiWait()) return false;
  *b = TWDR;
  return true;
}
static void twiStop(bool ok) {
  if (ok) {
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
    uint16_t n = TWI_SPIN;
    while ((TWCR & _BV(TWSTO)) && --n) {}
  } else {
    TWCR = 0; TWCR = _BV(TWEN);                      // reset the TWI unit after an error
  }
}
static void i2cBegin() {
  PORTC |= _BV(4) | _BV(5);                          // pull-ups on SDA / SCL, like Wire.begin()
  TWSR = 0;
  TWBR = ((F_CPU / 400000UL) - 16) / 2;
  TWCR = _BV(TWEN);
}
static void mpuWrite(uint8_t reg, uint8_t val) {
  bool ok = twiStart(MPU_ADDR << 1) && twiPut(reg) && twiPut(val);
  twiStop(ok);
}
static bool mpuReadBlock(uint8_t reg, uint8_t *buf, uint8_t n) {
  bool ok = twiStart(MPU_ADDR << 1) && twiPut(reg) && twiStart((MPU_ADDR << 1) | 1);
  for (uint8_t i = 0; ok && i < n; i++) ok = twiGet(&buf[i], i < n - 1);
  twiStop(ok);
  return ok;
}
#else
static void i2cBegin() {
  Wire.begin();
  Wire.setClock(400000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);
#endif
}
static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
static bool mpuReadBlock(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
#endif

uint8_t mpuRead8(uint8_t reg) {
  uint8_t v;
  return mpuReadBlock(reg, &v, 1) ? v : 0xFF;
}

bool imuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gz) {
  uint8_t b[14];
  if (!mpuReadBlock(0x3B, b, 14)) return false;      // accel x/y/z, temperature, gyro x/y/z
  ax = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  ay = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
  az = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
  gz = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
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
struct MotionCal { uint8_t magic; int16_t cosX1000; int16_t sinX1000; int16_t kX1000; uint16_t brakeDecel; };
const uint8_t MCAL_MAGIC = 0xB2;
const int MCAL_ADDR = 16;

void saveMotionCal() {
  MotionCal m;
  m.magic = MCAL_MAGIC;
  m.cosX1000 = (int16_t)(fwdCos * 1000.0f);
  m.sinX1000 = (int16_t)(fwdSin * 1000.0f);
  m.kX1000 = (int16_t)(kSpeed * 1000.0f);
  m.brakeDecel = (uint16_t)brakeDecel;
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
  if (m.brakeDecel >= 150 && m.brakeDecel <= 1500) { brakeDecel = m.brakeDecel; comfortDecel = brakeDecel * 0.4f; }
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
      float r = atan2Deg(fy, fz);
      float p = atan2Deg(-fx, sqrt(fy * fy + fz * fz));
      if (fabs(r) < 25.0f && fabs(p) < 25.0f) { rollTrim = r; pitchTrim = p; saveCal(); }
    }
  }
  yawDeg = 0; hhActive = false;
}

bool imuInit() {
  i2cBegin();
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
  roll  = atan2Deg(fy, fz) - rollTrim;
  pitch = atan2Deg(-fx, sqrt(fy * fy + fz * fz)) - pitchTrim;
  gMag  = sqrt(fx * fx + fy * fy + fz * fz);

  // ---- motion estimate: acceleration without gravity, forward speed, slope ----
  static bool restInit = false;
  if (!restInit) { restX = fx; restY = fy; restInit = true; }
  float rr = (fabs(speedCur) < 1.0f) ? 0.01f : 0.0005f;   // ~1 s average when stationary, ~20 s while driving (so a long acceleration is not mistaken for gravity)
  restX += (fx - restX) * rr;                        // how gravity falls on x and y
  restY += (fy - restY) * rr;
  aDynX = fx - restX; aDynY = fy - restY;
  {                                                  // a sudden push on the stationary car (works without calibration)
    static uint8_t pushCnt = 0;
    float am = sqrt(aDynX * aDynX + aDynY * aDynY);
    if (fabs(speedCur) < 1.0f && !hardBrake && !brakeInput && millis() > brakeUntil && am > PUSH_G &&
        (!fwdValid || aDynX * fwdCos + aDynY * fwdSin > 0.03f)) {
      if (++pushCnt >= 2) { pushUntil = millis() + 1500UL; pushCnt = 0; }
    } else pushCnt = 0;
  }
  if (fwdValid) {
    float aMag = sqrt(aDynX * aDynX + aDynY * aDynY);
    aFwd = aDynX * fwdCos + aDynY * fwdSin;
    aFwdLP += (aFwd * 981.0f - aFwdLP) * 0.3f;
    vImu += aFwd * 981.0f * dt;                      // g -> cm/s^2
    vImu -= vImu * 0.4f * dt;                        // leak, so sensor drift can't build up
    if (speedCur == 0 && aMag < 0.03f && fabs(gyroZ) < 2.0f) vImu -= vImu * 8.0f * dt;   // standing still: pull to zero
    float rf = clampf(restX * fwdCos + restY * fwdSin, -0.7f, 0.7f);                    // gravity along the car's forward axis
    slopeDeg = atan2Deg(rf, sqrt(1.0f - rf * rf));
  } else { aFwd = 0; aFwdLP = 0; vImu = 0; slopeDeg = 0; }

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
//  MOTION, COLLISION SYSTEM AND ACC
// =====================================================================
// Yaw rate (deg/s, + = left turn) the car should have for this steering and speed:  w = v * tan(d) / L
float expectedYawDps(float steerPct, float speedPct) {
  float d = steerPct * 0.01f * WHEEL_LOCK_DEG * 0.0174533f;
  return -(speedPct * kSpeed / WHEELBASE_CM) * fastTan(d) * 57.2958f;
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

// Works on the car's real speed (throttle, coasting, rolling, a gentle push), not on the throttle.
// Outputs: capPct (speed limit), hardBrake, evading/evadeDir/evadeStrength, blockedFront, vOwn.
void collisionUpdate(float steerIn, float tgtT, unsigned long now) {
  float dC = distF(0), dL = distF(1), dR = distF(2);
  float vModel = speedCur > 0 ? speedCur * kSpeed : 0;
  float vInertial = vImu > 0 ? vImu : 0;
  vOwn = vModel > vInertial ? vModel : vInertial;
  float vClose = vSonar > 0 ? vSonar : 0;
  if (vClose < vOwn * 0.85f) vClose = vOwn * 0.85f;           // the sonar rate is noisy: never let it sit below the car's own speed

  // slow closing speed: the distance half a second ago minus the distance now. Averaged, so a gentle roll shows up
  // even though the quick estimate above is far too noisy for that.
  static float hist[7]; static unsigned long histAt = 0;
  if (now - histAt >= 100UL) {
    histAt = now;
    for (uint8_t i = 6; i > 0; i--) hist[i] = hist[i - 1];
    hist[0] = dC < 400.0f ? dC : 400.0f;
  }
  vSlow = clampf((hist[5] + hist[6]) - (hist[0] + hist[1]), -250.0f, 250.0f);     // (average old - average new) / 0.5 s
  if (hist[5] >= 400.0f || hist[6] >= 400.0f) vSlow = 0;                          // it was out of range 0.5 s ago: that is not a closing speed

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
    float vmax = sqrt(2.0f * comfortDecel * (avail > 0 ? avail : 0));
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
    if (aReq > brakeDecel * 0.8f) want = true;
    else if (speedCur < 8.0f && aReq > comfortDecel * 1.2f) want = true;
  }
  // nobody on the throttle and the car is rolling toward something (a gentle push, a slope, a bump): stop it in front of it
  coastBrake = false;
  if (collisionOn && !wide && fabs(speedCur) < 8.0f && fabs(tgtT) < 3.0f) {
    float vRoll = maxf(vSlow, vOwn > 5.0f ? vOwn : 0.0f);
    float g2 = (GAP_BASE_CM + GAP_PER_CMS * vRoll) * gapScale + vRoll * SENSOR_LATENCY_S + 4.0f;
    if (vRoll > PUSH_BRAKE_CMS && dC < g2) coastBrake = true;
    if (pushUntil > now && dC < 40.0f) coastBrake = true;       // the accelerometer felt a push and something is close ahead
  }
  if (coastBrake) want = true;
  if (want) {
    if (!hardBrake && vOwn > 10.0f) obstacleFlashUntil = now + 700;
    hardBrake = true; hardBrakeUntil = now + 300;
  } else if (hardBrake && now > hardBrakeUntil && (vClose < 8.0f || aReq < comfortDecel * 0.5f)) {
    hardBrake = false;
  }
  if (!collisionOn) hardBrake = false;

  // steering for the driver: early when there is room to go around, hardest when braking alone can't stop the car
  int8_t fs = freeSide(dC, dL, dR);
  float u = aReq / brakeDecel; if (u > 1.0f) u = 1.0f;
  bool unavoidable = collisionOn && evadeOn && vClose > EVADE_MIN_CMS && aReq > brakeDecel && dC > 10.0f;
  float trig = gap + lat + (vClose * vClose) / (2.0f * comfortDecel) * 1.3f + 20.0f;
  bool early = collisionOn && evadeOn && autoMode == 0 && !wide && vOwn > EVADE_MIN_CMS && vClose > 18.0f && dC > 12.0f && dC < trig;
  bool driverSteersClear = (w >= 0.5f) && fs != 0 && ((steerIn > 0) == (fs > 0));
  if (evading) {
    bool driverOverride = (steerIn * evadeDir) < -60.0f;             // pushing the pad hard the other way takes it back
    if (driverOverride) evadeLockout = now + 1000;
    bool opened = evadeStrength < 99.0f && (now - evadeStart > 450UL) && (wide || dC > trig * 1.4f);   // gentle swing only: hand back once the way is open
    if (driverOverride || opened || now > evadeUntil) { evading = false; evadeDir = 0; }
  } else if ((unavoidable || early) && fs != 0 && !driverSteersClear && now > evadeLockout) {
    evading = true; evadeDir = fs; evadeStart = now;
    evadeUntil = now + EVADE_MS + (unavoidable ? 0UL : 200UL);
    evadeStrength = 55.0f + 45.0f * u;                               // 55 % for an early swing, 100 % when it is the last chance
  }

  blockedFront = collisionOn && !wide && (hardBrake || capPct < 8.0f);
}

// Stuck = throttle on but the car isn't going anywhere (blocked by something the sonar can't see, wheels in the air, ...)
void stuckUpdate(unsigned long now) {
  static unsigned long driveSince = 0, lastMoving = 0, sampleAt = 0, staticSince = 0;
  static float lastD = 0; static bool fromStill = false;
  if (fabs(speedCur) > 5.0f || fabs(vImu) > 8.0f) lastMoving = now;
  if (!stuckOn || stuck || calStep != 0) { driveSince = 0; staticSince = 0; return; }
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

// Car-drives-itself calibration (C,2): a ~2 m run at full power toward a wall ~3 m away, then a hard stop.
//   launch (first 0.5 s)  -> which way is forward for the accelerometer
//   steady stretch 70-190 cm -> real speed per % of throttle (from the sonar distance and the throttle history)
//   hard stop             -> real braking rate (speed from the steady stretch, stopping distance from the sonar)
void calToBrake(unsigned long now, float trav, uint8_t why) {
  calDs = trav; if (why && !calWhy) calWhy = why;
  calStep = 3; calT0 = now; calTpk = 0;
  float mag = sqrt(calMx * calMx + calMy * calMy);
  if (calN >= 5 && mag > 0.0f) { calFx = calMx / mag; calFy = calMy / mag; }
}

void motionCalTick(unsigned long now, float &tgtT, float &tgtS, bool &braking) {
  if (calStep == 0) return;
  tgtS = 0;
  float d = distF(0);
  if (abs(cmdThrottle) > 10 || brakeInput || flipped || linkLost) { calStep = 0; calState = 4; calWhy = 6; tgtT = 0; return; }
  if (calStep == 1) {                                          // still, and a wall 280..400 cm ahead?
    tgtT = 0;
    if (now - calT0 > 500UL) {
      if (!sonarOn || d < 280.0f || d >= 400.0f) { calState = 3; calStep = 0; return; }
      calD0 = d; calInt = 0; calMx = 0; calMy = 0; calN = 0; calFlags = 0; calWhy = 0;
      calStep = 2; calT0 = now;
    }
  } else if (calStep == 2) {                                   // full power straight ahead
    tgtT = 100.0f;
    calInt += speedCur * 0.01f;                                // % x seconds
    unsigned long t = now - calT0;
    float trav = calD0 - d;
    if (t > 40UL && t < 600UL) {
      float m = sqrt(aDynX * aDynX + aDynY * aDynY);
      if (m > 0.03f && calN < 250) { calMx += aDynX; calMy += aDynY; calN++; }
    }
    if (!(calFlags & 1) && trav >= 70.0f)  { calFlags |= 1; calDa = trav; calIa = calInt; calTa = t; }
    if (!(calFlags & 2) && trav >= 190.0f) { calFlags |= 2; calDb = trav; calIb = calInt; calTb = t; }
    if (trav >= 200.0f)  calToBrake(now, trav, 0);
    else if (d < 60.0f)  calToBrake(now, trav, 5);             // safety: wall too close, stop at once
    else if (t > 4500UL) calToBrake(now, trav, 2);             // wheels slipping or far too slow
  } else if (calStep == 3) {                                   // hard stop and settle
    tgtT = 0; braking = true;
    if (!(calFlags & 4)) {                                     // time of the first burst of hard braking only (rebounds come later)
      if ((-(aDynX * calFx + aDynY * calFy) * 981.0f) > 60.0f) calTpk = now;
      else if (calTpk != 0) calFlags |= 4;                     // the first sample below the limit ends the burst
    }
    if (now - calT0 > 1100UL) calStep = 4;
  } else if (calStep == 4) {
    tgtT = 0;
    calStep = 0;
    float dEnd = distF(0);
    float mag = sqrt(calMx * calMx + calMy * calMy);
    calHave = true; calRk = 0; calRdec = 0; calRbrk = 0;
    calRpush = calN ? (int)(mag / calN * 981.0f) : 0;                                  // mean launch push, cm/s2
    calRrun = (calFlags & 2) ? (int)(calTb - calTa) : 0;                               // time for the 70..190 cm stretch, ms
    if (calWhy == 0 && (calN < 5 || mag < 0.04f * calN)) calWhy = 1;
    if (calWhy == 0 && !(calFlags & 2)) calWhy = 2;
    float k = 0, vC = 0, decel = 0;
    if (calWhy == 0) {
      float dd = calDb - calDa, di = calIb - calIa, dt = (calTb - calTa) * 0.001f;
      if (di < 5.0f || dt < 0.2f) calWhy = 3;
      else {
        k = dd / di; vC = dd / dt;                            // cm per (% x s), cruise speed in cm/s
        calRk = k;
        if (k < 0.3f || k > 2.0f) calWhy = 3;
      }
    }
    if (calWhy == 0) {
      // braking rate = cruise speed / time spent braking hard (accelerometer). The sonar only gives a stopping distance, which is
      // blurred by its own delay and reads far too long for a car that stops in 10 cm, so it is the fallback.
      float tS = (calTpk > calT0) ? (calTpk - calT0) * 0.001f : 0.0f;
      float stopD = (calD0 - dEnd) - calDs;                    // distance covered while braking
      if (tS > 0.04f && tS < 1.5f) decel = vC / tS;
      else if (stopD >= 3.0f) decel = vC * vC / (2.0f * stopD);
      else calWhy = 7;
      calRdec = decel; calRbrk = (int)(tS * 1000.0f);
      if (calWhy == 0 && (decel < 100.0f || decel > 3000.0f)) calWhy = 4;
    }
    if (calWhy != 0) { calState = (calWhy == 5) ? 4 : 5; return; }
    fwdCos = calMx / mag; fwdSin = calMy / mag; fwdValid = true;
    kSpeed = k; vImu = 0;
    brakeDecel = clampf(decel * 0.8f, 150.0f, 1500.0f);        // keep a margin: floors and tyres vary
    comfortDecel = brakeDecel * 0.4f;
    saveMotionCal();
    calState = 2;
  }
}

// Traction control: if the throttle says "go" but the accelerometer says the car is not picking up speed, the wheels are
// spinning. Cut the power to just above what the car is really doing and give it back gradually as it grips.
// Strength 0..100: stronger = deeper cut (lower floor and allowance) and slower return.
void tractionControl(float &tgtT) {
  static float vExp = 0; static unsigned long spinSince = 0, holdUntil = 0;
  float vCmd = fabs(speedCur) * kSpeed;
  float aExp = (vCmd - vExp) * TRACTION_LAG * 100.0f;                   // cm/s^2 the car should be picking up right now
  vExp += (vCmd - vExp) * TRACTION_LAG;
  bool on = tractionPct > 0 && fwdValid && imuOk && calStep == 0 && autoMode == 0 && fabs(tgtT) > 20.0f;
  if (!on) { tractionCap = 100.0f; spinSince = 0; return; }
  unsigned long now = millis();
  float st = (float)tractionPct;
  float aMeas = (tgtT < 0) ? -aFwdLP : aFwdLP;                          // cm/s^2 it is really picking up
  bool spin = aExp > 80.0f && aMeas < 0.35f * aExp;                     // asked to accelerate hard, hardly accelerating
  if (spin) { if (spinSince == 0) spinSince = now; } else spinSince = 0;
  float vMeas = (tgtT < 0) ? -vImu : vImu;
  if (vMeas < 0) vMeas = 0;
  if (spinSince != 0 && now - spinSince >= (unsigned long)(40.0f + (100.0f - st) * 0.6f)) {
    float target = maxf(vMeas / kSpeed + (35.0f - 0.27f * st), 55.0f - 0.15f * st);
    tractionCap = maxf(target, tractionCap - 10.0f);
    holdUntil = now + 120UL;                                            // keep the cut for a moment: the lower power makes the check look healthy at once
  } else if (now >= holdUntil) {
    tractionCap += 1.2f - 0.007f * st;                                  // give the power back gradually...
    float follow = vMeas / kSpeed + 25.0f;                              // ...and faster as the car picks up speed
    if (tractionCap < follow) tractionCap = follow;
    if (tractionCap > 100.0f) tractionCap = 100.0f;
  }
  if (tgtT > tractionCap) tgtT = tractionCap;
  else if (tgtT < -tractionCap) tgtT = -tractionCap;
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
    } else {                                                   // Roam / Follow / Corridor: the app does the driving over D,
      tgtS = cmdSteer;                                         // the car still brakes, steers around and cuts out by itself
      tgtT = clampf((float)cmdThrottle, -maxPct, maxPct);
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
  collisionUpdate(tgtS, tgtT, now);
  if (tgtT > capPct) tgtT = capPct;
  tractionControl(tgtT);
  stuckUpdate(now);
  if (stuck) { tgtT = 0; }
  bool brakeActive = brakeInput || hardBrake || calBraking;
  if (brakeActive && !brakePrev) { brakeT0 = now; brakeV0 = maxf(fabs(speedCur) * kSpeed, vOwn); }   // how fast we were going when the brake came on
  brakePrev = brakeActive;

  // ---- ramp ----
  if (brakeActive || now < brakeUntil) {
    speedCur = 0;
    hhActive = false;
  } else {
    float step = (rampMs == 0 || calStep == 2) ? 100.0f : 1000.0f / rampMs;       // % per 10 ms (the calibration launch is a hard kick)
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
  if (evading) { steerOut = evadeDir * evadeStrength; steerEased = steerOut; }
  steerNow = steerOut;
  steerWrite((int)(SERVO_CENTER + SERVO_DIR * steerOut * SERVO_RANGE / 100.0f));

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
  absPulsing = false;
  if (brakeActive || now < brakeUntil) {
    bool on = true;                                                // solid short brake...
    if (evading) on = ((uint16_t)now % 60) < 40;                             // ...but pulsed while swerving, so the car can still steer
    else if (absOn && ((absModes >> driveMode) & 1) && brakeActive && calStep == 0) {
      // ABS: pulse while the car is still fast (or the gyro sees a spin), full solid brake once it has slowed down.
      float vB = brakeV0 - brakeDecel * (float)(now - brakeT0) * 0.001f;     // speed estimate while braking
      if (fwdValid && vImu > vB) vB = vImu;
      if (vB > ABS_SOLID_BELOW_CMS || fabs(gyroZ) > ABS_YAW_DPS) { on = ((uint16_t)now % 60) < 40; absPulsing = true; }
    }
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
  uint16_t phase = (uint16_t)micros() % 2500;                  // 2.5 ms period, duty % of it
  digitalWrite(PIN_HEADLIGHT, (phase < (uint16_t)duty * 25) ? HIGH : LOW);
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
  uint16_t n16 = (uint16_t)now;                 // 16-bit maths is much smaller on an AVR; a blink glitching once a minute is invisible
  bool warn = false, warnOn = false, hazardOnly = false;

  if (flipped)                          { warn = true; warnOn = (n16 % 120) < 60; }                 // strobe
  else if (linkLost || cutoff)          { warn = true; warnOn = (n16 % 200) < 100; }                // fast blink
  else if (evading)                     { warn = true; warnOn = (n16 % 120) < 60; }                 // swerving: strobe
  else if (now < obstacleFlashUntil)    { warn = true; warnOn = (n16 % 200) < 100; }                // 3 quick flashes
  else if (stuck || autoGaveUp)         { warn = true; warnOn = (n16 % 400) < 200; }                // stuck: slow urgent blink
  else if (limp)                        { warn = true; uint16_t m = n16 % 2000; warnOn = (m < 80) || (m >= 200 && m < 280); }  // double blink
  else if (hazard)                      { warn = true; hazardOnly = true; warnOn = (n16 % 1000) < 500; }

  bool reverseBeep = (autoMode != 0 && speedCur < -2.0f && (n16 % 500) < 250);   // the car is backing up blind
  if (warn) headlightWrite(warnOn, 100);                // warnings always full brightness
  else      headlightWrite(lightsUser, headBrightness);
  buzzerWrite(hornOn || reverseBeep || (warn && warnOn && !hazardOnly));
}

// =====================================================================
//  BLUETOOTH
// =====================================================================
void handleLine(char *line) {
  char cmd = line[0];
  long v[3] = { 0, 0, 0 };
  char *p = line;
  for (uint8_t i = 0; i < 3; i++) {
    p = strchr(p, ',');
    if (!p) break;
    v[i] = atol(++p);
  }
  long a = v[0], b = v[1], c = v[2];

  bool known = true;
  switch (cmd) {
    case 'D': cmdThrottle = clampPct(a); cmdSteer = clampPct(b); brakeInput = (c != 0); break;
    case 'X': cmdThrottle = 0; holdSpeed = false; emergencyStop(300); break;
    case 'M': driveMode = (uint8_t)lim(a, 0, 2); rampMs = MODES[driveMode].rampMs; tractionPct = MODE_TRACTION[driveMode]; break;
    case 'A': autoMode = sonarOn ? (uint8_t)lim(a, 0, 3) : 0; holdSpeed = false;      // A,0,1 = the app's autopilot gave up
              autoGaveUp = (b != 0);
              if (autoMode) { cmdThrottle = 0; cmdSteer = 0; }                         // the app takes over the drive line
              break;
    case 'K':
      if (a && autoMode == 0 && cmdThrottle > 0) { holdSpeed = true; holdValue = cmdThrottle; }   // ACC holds forward speed only
      else holdSpeed = false;
      break;
    case 'L': lightsUser = (a != 0); break;
    case 'B': headBrightness = (uint8_t)lim(a, 0, 100); break;
    case 'H': hazard = (a != 0); break;
    case 'V': hornOn = (a != 0); break;
    case 'R': rampMs = (uint16_t)lim(a, 0, 3000); break;
    case 'S': stabAssist = (a != 0); break;
    case 'G': headingHold = (a != 0); hhActive = false; break;
    case 'F': diffGain = (float)lim(a, 0, 150) / 100.0f; break;   // differential strength in %
    case 'T':                                                                         // assist switches
      collisionOn = (a & 1) != 0; evadeOn = (a & 2) != 0; wallKeepOn = (a & 4) != 0; hillHoldOn = (a & 8) != 0;
      absOn = (a & 16) != 0; stuckOn = (a & 32) != 0; launchOn = (a & 64) != 0;
      if (!stuckOn) stuck = false;
      break;
    case 'Q': tractionPct = (uint8_t)lim(a, 0, 100); break;
    case 'Y': absModes = (uint8_t)(a & 7); break;
    case 'O':
      sonarOn = (a != 0);
      if (!sonarOn) {                                                                 // no sonar: no obstacles, no autonomy
        for (uint8_t i = 0; i < 3; i++) { distNow[i] = 999; distPrev[i] = 999; }
        vSonar = 0; vSlow = 0; lastCenterMs = 0; autoMode = 0; calStep = 0;
      } else { sonarPhase = 0; sonarT = micros(); }
      break;
    case 'E': break;                                                                  // top speed of the auto modes now lives in the app
    case 'U': steerReturnMs = (uint16_t)lim(a, 0, 2000); break;
    case 'J': gapScale = (float)lim(a, 50, 200) / 100.0f; break;
    case 'Z': if (speedCur == 0 && imuOk) imuCalibrate(false); break;
    case 'C':
      if (strchr(line, ',') == NULL) break;                            // ignore a damaged "C" with no value
      if (a == 0) { rollTrim = 0; pitchTrim = 0; saveCal(); }          // C,0 clears the saved level trim
      else if (a == 2) {                                               // C,2 motion and braking calibration: the car drives itself ~2 m
        if (calStep == 0 && speedCur == 0 && imuOk && autoMode == 0 && !holdSpeed && !linkLost && !flipped && !brakeInput) {
          calStep = 1; calState = 1; calWhy = 0; calT0 = millis();
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

// Numbers go out as plain text: tx() sends one value plus a separator ('\n' ends the line).
static void tx(long v, char sep) { BTPORT.print(v); BTPORT.write((uint8_t)sep); }
static void txh(char h) { BTPORT.write((uint8_t)h); BTPORT.write((uint8_t)','); }

// One line every 50 ms. The sonar line (S) goes out every 100 ms so the app can steer the car in Roam / Follow / Corridor.
void telemetryUpdate() {
  unsigned long now = millis();
  if (now - lastTelMs < 50) return;
  lastTelMs = now;
  if (!linkSeen) return;
  static uint8_t phase = 0;
  uint8_t ph = phase;
  phase = (phase + 1) & 7;

  if ((ph & 1) == 0) {                                                   // S: every 100 ms
    uint16_t flags = (linkLost ? 1 : 0) | (limp ? 2 : 0) | (cutoff ? 4 : 0) | (flipped ? 8 : 0) |
                     (blockedFront ? 16 : 0) | (crashFlag ? 32 : 0) | (holdSpeed ? 64 : 0) |
                     (stabAssist ? 128 : 0) | (headingHold ? 256 : 0) | (stuck ? 512 : 0) |
                     (evading ? 1024 : 0) | (hardBrake || brakeInput ? 2048 : 0) |
                     (capPct < 99.0f ? 4096 : 0) | (hillHolding ? 8192 : 0) | (driftFlag ? 16384 : 0) |
                     (autoGaveUp ? 32768U : 0);
    txh('S'); tx(distF(0), ','); tx(distF(1), ','); tx(distF(2), ','); tx(driveMode, ','); tx(autoMode, ','); tx(flags, '\n');
  } else if (ph == 1 || ph == 5) {                                       // I: every 200 ms
    txh('I'); tx((int)roll, ','); tx((int)pitch, ','); tx((int)yawDeg, ','); tx((int)(gMag * 10.0f), ',');
    tx((int)(battV * 100.0f), ','); tx((int)speedCur, '\n');
  } else if (ph == 7 && calHave) {                                       // W: numbers of the last calibration run
    txh('W'); tx(calRpush, ','); tx(calRrun, ','); tx((int)(calRk * 100.0f), ','); tx((int)calRdec, ','); tx(calRbrk, '\n');
  } else {                                                               // M: motion line
    txh('M'); tx((int)vOwn, ','); tx((int)vSonar, ','); tx((int)gyroZ, ','); tx((int)expectedYawDps(steerNow, speedCur), ',');
    tx((int)(slopeDeg * 10.0f), ','); tx((int)capPct, ','); tx(calState, ','); tx(fwdValid ? 1 : 0, ','); tx(calWhy, ',');
    tx((int)brakeDecel, ','); tx((int)(kSpeed * 100.0f), ',');
    tx((tractionCap < 95.0f ? 1 : 0) | (absPulsing ? 2 : 0) | (evading && evadeStrength < 99.0f ? 4 : 0) |
       (sonarOn ? 8 : 0) | (coastBrake ? 16 : 0), '\n');
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

#if LIGHT_DRIVERS
  servoBegin();
#else
  steeringServo.attach(PIN_SERVO);
#endif
  steerWrite(SERVO_CENTER);

  for (uint8_t i = 0; i < 20; i++) { batteryUpdate(); delay(5); }
  imuOk = imuInit();
  rampMs = MODES[driveMode].rampMs;
  tractionPct = MODE_TRACTION[driveMode];
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
