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

- Left pad: throttle (up = forward, down = reverse). Springs back to zero.
- Right pad: steering. Springs back to center.
- **Tilt steering**: hold the phone upright like a wheel. Use **Reverse tilt** if it feels backwards.
- **Hold speed on release**: arm it, drive, then let go of the throttle and the car keeps that speed. Touch the throttle again to release.
- Drive modes, autonomous modes, lights, hazard, horn, brake, stability assist, heading hold, headlight brightness and acceleration ramp are all in the middle panel.

## Why there is no delay

- Only the latest stick position is ever sent. Old positions are dropped, never queued.
- The sender thread wakes the moment a value changes and sends at most every 25 ms.
- The drive line repeats every 100 ms when nothing changes, which also keeps the car's failsafe fed.
- If the HC-05 is still at 9600 baud it is the slowest part of the chain. Setting it to 38400 (`AT+UART=38400,0,0`) and `BT_BAUD` in the sketch to match makes it feel noticeably snappier.

## Protocol

Text lines ending in a newline. App to car: `D,<throttle>,<steer>` (-100..100), `M`, `A`, `K`, `L`, `B`, `H`, `V`, `R`, `S`, `G`, `Z`, `X`, `P`. Car to app: `S,...` and `I,...` telemetry. The full list is at the top of `firmware/rc_car/rc_car.ino`.
