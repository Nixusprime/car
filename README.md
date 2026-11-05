# RC Car

Android controller for the Arduino Nano RC car, plus the car's firmware.

- `app/` Android app (Kotlin, no external libraries). Landscape, classic Bluetooth (HC-05 / HC-06).
- `firmware/rc_car/rc_car.ino` the Arduino sketch.
- `.github/workflows/build.yml` builds the APK on every push and publishes it as the **latest** release.

## Get the app on your phone

1. Push to `main` (or run the workflow from the Actions tab).
2. When the build is green, open the repo's **Releases** page, tap **Latest build**, and download `rc-car.apk`.
3. Install it (allow "install unknown apps" for your browser the first time).

## First run

1. Pair the car's Bluetooth module in Android settings (PIN `1234`).
2. Open the app, tap **Connect**, choose the module. The app reconnects to it automatically next time.

## Controls

Everything fits on one screen, nothing scrolls. Tap the round **i** next to Connect for the full in-app manual.

- **Throttle pad (left)**: your thumb's landing point is zero (a ring marks it). Up = forward, down = reverse. A small dead zone ignores accidental slips. **Pad travel** in Settings sets how far you move for full throttle.
- **Brake**: while throttling, slide your thumb left past the red brake line. Both motors are shorted for the hardest stop and held while you stay left. The throttle stays at zero until you lift or return to the ring. The **BRAKE** button under the steering pad does the same while pressed. Braking cancels Cruise and any Auto mode.
- **Steering pad (right)**: also a floating zero. When you let go, the steering eases back to center (Settings > Steering return) to spare the servo. **Tilt steer** uses the phone like a wheel.
- **Cruise**: arm it, push to a speed, let go. The car keeps that speed and slows to keep a gap behind things (Assists > Following gap). Touch the throttle or brake to cancel.
- Top right: **Mode**, **Auto**, **Assists**, **Settings** open popups. **Settings** also has **Calibrate** and **Diagnostics**.

## What the car does by itself

- **Differential**: wheel speeds follow the real geometry (wheelbase 17 cm, track 11 cm) and the car turns tighter at low speed.
- **Collision protection**: uses the sonars and the car's own speed (gyro/accelerometer plus sonar), also when you let go and it is rolling. Smooth speed limit, early steer-around when there is free space (then hands back), hard brake when a crash is unavoidable, emergency steering as a last resort (push the pad hard the other way to take over). It also brakes when the car is rolling or pushed toward something with nobody on the throttle.
- **Traction control** (launch only, per drive mode: Eco off, Normal 30%, Sport 80%) and **ABS** (per drive mode, pulses at speed, full brake once slow), both from the accelerometer. Strength and modes are set in Assists.
- **Sonars on/off** (Assists): off means no collision protection, wall keeping, Cruise gap or Auto modes.
- **Top speed** slider per Auto mode (Roam, Follow, Corridor) in the Auto popup.
- **Hill hold**, **launch assist**, **stuck cut-off**, **ABS guard**, **wall keeping**, **slip correction**: all switchable in Assists.
- **Autonomous modes** run on the phone (the car streams sonar data about 10 times a second, the app drives; the car's own collision system and failsafe still apply, and no phone means the car stops). They avoid reversing: tight forward turn first, then a short slow reverse with beeps, then stop and ask for help.
- The buzzer is a passive one, so the sketch makes its own 2.5 kHz tone (`BUZZER_TYPE 2`).

## Calibrate

Settings > Calibrate.

1. **Level and zero**: car on a flat surface and still. Roll and pitch should drop to about 0. Turn the car left by hand: yaw should go up; if not, tap **Flip yaw direction**.
2. **Motion and braking calibration**: put the car on the floor with a flat wall 280 to 400 cm (about 3 m) straight ahead and nothing in the way. The car drives about 2 m at full power, then brakes hard. It learns which way is forward for the motion sensor, its real speed and its real braking rate (used by the collision system and ABS). Hold BRAKE to stop it. If it fails the app says which check missed, and Diagnostics lists every number measured (launch push, time for the 70-190 cm stretch, speed scale, braking rate).

Values are saved in the car (EEPROM), so this is a one-time job unless the board is moved.

Optional: measure the full-lock turning circle radius R (cm, of the car's center line) and set `WHEEL_LOCK_DEG` in the sketch to `atan(17 / R)` in degrees (default 34). The steering servo travels `SERVO_RANGE` degrees each way (now 40): lower it if the servo strains at full lock.

## Tuning notes

Run the motion and braking calibration first: it replaces `BRAKE_DECEL` and `COMFORT_DECEL` with measured values. The sketch is sized for the Nano's 30720 bytes: the Auto modes now run in the app (`Autopilot.kt`), and `LIGHT_DRIVERS 1` uses small built-in I2C and servo drivers (set it to 0 to go back to the Wire and Servo libraries). The braking numbers (`BRAKE_DECEL`, `COMFORT_DECEL`, `SENSOR_LATENCY_S`, `GAP_BASE_CM` at the top of the sketch) are estimates. Test on your floor at low speed against a soft wall and adjust: if the car stops too far from things, raise `BRAKE_DECEL`/`COMFORT_DECEL`; if it hits, lower them. The sonars have narrow beams and there is none at the back, so the system is best effort, not a guarantee.

## Battery reading

The BMS has no signal wire, so the battery number needs two 10k resistors from pack + to A6 to GND, and `HAS_BATT_SENSE 1` in the sketch. Until then the app shows `n/a` and the car ignores battery voltage.

## Why there is no delay

- Only the latest stick position is ever sent. Old positions are dropped, never queued.
- The sender thread wakes the moment a value changes and sends at most every 25 ms.
- The drive line repeats every 100 ms when nothing changes, which also keeps the car's failsafe fed.
- If the HC-05 is still at 9600 baud it is the slowest part of the chain. Setting it to 38400 (`AT+UART=38400,0,0`) and `BT_BAUD` in the sketch to match makes it feel noticeably snappier.

## Protocol

Text lines ending in a newline. App to car: `D,<throttle>,<steer>,<brake 0|1>` (-100..100), `M`, `A`, `K`, `L`, `B`, `H`, `V`, `R`, `S`, `G`, `Z`, `C` (1 level, 0 clear, 2 motion and braking), `N`, `F`, `T` (assist mask), `Y` (ABS per mode), `Q` (traction %), `O` (sonars), `E` (Auto top speed), `U` (steering return ms), `J` (following gap %), `X`, `P`. Car to app: `S,...`, `I,...`, `M,...` telemetry and `W,...` (last calibration run). The full list is at the top of `firmware/rc_car/rc_car.ino`.
