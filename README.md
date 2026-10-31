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

Everything fits on one screen, nothing scrolls.

- Left pad: throttle (up = forward, down = reverse). Springs back to zero.
- Right pad: steering. Springs back to center. Horn and Brake sit right under it.
- Middle: connection, battery, speed, tilt, a top-down picture of the car with the three sonar distances, alerts, and the **Lights / Hazard / Hold speed / Tilt steer** toggles.
- Top right: **Mode**, **Auto**, **Assists** and **Settings** open small popups.
- **Tilt steering**: hold the phone upright like a wheel. **Reverse tilt** (in Assists) swaps the direction.
- **Hold speed**: arm it, drive, then let go of the throttle and the car keeps that speed. Touch the throttle again to release.
- **Settings** has headlight brightness, the acceleration ramp, two sliders that move the throttle pad to the right and the steering pad up (remembered), and **Calibrate**.

## Calibrate

Settings > Calibrate the car's sensors.

1. Car on a flat surface and still, tap **Level and zero**. Roll and pitch should drop to about 0.
2. Turn the car to the left by hand. Yaw should go up. If it goes down, tap **Flip yaw direction**.

The values are saved in the car (EEPROM), so this is a one-time job unless the board is moved.

## Battery reading

The BMS has no signal wire, so the battery number needs two 10k resistors from pack + to A6 to GND, and `HAS_BATT_SENSE 1` in the sketch. Until then the app shows `n/a` and the car ignores battery voltage.

## Why there is no delay

- Only the latest stick position is ever sent. Old positions are dropped, never queued.
- The sender thread wakes the moment a value changes and sends at most every 25 ms.
- The drive line repeats every 100 ms when nothing changes, which also keeps the car's failsafe fed.
- If the HC-05 is still at 9600 baud it is the slowest part of the chain. Setting it to 38400 (`AT+UART=38400,0,0`) and `BT_BAUD` in the sketch to match makes it feel noticeably snappier.

## Protocol

Text lines ending in a newline. App to car: `D,<throttle>,<steer>` (-100..100), `M`, `A`, `K`, `L`, `B`, `H`, `V`, `R`, `S`, `G`, `Z`, `C`, `N`, `X`, `P`. Car to app: `S,...` and `I,...` telemetry. The full list is at the top of `firmware/rc_car/rc_car.ino`.
