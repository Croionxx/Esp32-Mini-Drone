# ESP32-C3 Mini Drone Flight Controller v1.0

A lightweight, dependency-free flight controller for a brushless micro-drone ("tiny whoop" class), built on the ESP32-C3. Runs a deterministic 500 Hz control loop, fuses IMU data with a complementary filter, and is piloted over Wi-Fi/UDP from a phone joystick app — no separate RC receiver needed.

## Features

- 500 Hz hardware-timer flight loop (no FreeRTOS, single-core cooperative scheduler)
- Complementary filter (α = 0.98) for roll/pitch attitude estimation
- Cascade PID control: outer angle-P loop → inner rate-PID loop
- Two flight modes: **ANGLE** (self-levelling) and **ACRO** (direct rate)
- Wi-Fi access point + UDP joystick control (ASCII protocol, port 4210)
- UDP + Serial telemetry at 1 Hz (port 4211 / 115200 baud)
- Arm/disarm state machine requiring a sustained arm gesture (no accidental spin-up)
- Kill switch and link-loss failsafe (auto-disarm)
- Battery voltage monitoring with low-voltage warning
- Buzzer + status LED feedback for IMU/ESC ready, arming, and battery state

## Hardware

| Part | Spec |
|---|---|
| MCU | ESP32-C3 Super Mini (or any ESP32-C3 DevKit) |
| IMU | MPU6050 (I²C, address `0x68`) |
| ESC | 4-in-1 BLHeli_S, 5–10 A, standard 1000–2000 µs PWM |
| Motors | 1103 brushless, ~8000 KV, X-configuration |
| Battery | 1S LiPo, 450–650 mAh |
| Props | 40–65 mm, matched to motor recommendation |

## Wiring / Pin Map

| Signal | GPIO | Notes |
|---|---|---|
| I²C SDA | 4 | MPU6050 |
| I²C SCL | 5 | MPU6050 |
| Motor 0 — Front-Left (CW) | 6 | ESC channel 0 |
| Motor 1 — Front-Right (CCW) | 7 | ESC channel 1 |
| Motor 2 — Rear-Left (CCW) | 8 | ESC channel 2 |
| Motor 3 — Rear-Right (CW) | 9 | ESC channel 3 |
| Buzzer | 0 | Active-high |
| Status LED | 2 | Active-high |
| Battery sense (ADC1) | 1 | Through resistor divider, 100kΩ / 47kΩ |

```
        ── FRONT ──
  M0 (CW) ●        ● M1 (CCW)
            \    /
            /    \
  M2 (CCW) ●        ● M3 (CW)
        ── BACK ──
```

## Software Requirements

- Arduino IDE 2.x
- `arduino-esp32` board package **v3.x** (the code uses the v3.x LEDC/timer API: `ledcAttach()`, `timerBegin(freq)`, `timerAlarm()`). If you're on v2.x, you'll need to swap in the older `ledcSetup`/`ledcAttachPin`/`timerAlarmWrite` calls.

No external libraries are required — the MPU6050 driver talks directly to I²C registers.

## Setup & Flashing

1. Wire the hardware per the pin map above.
2. Open the sketch in Arduino IDE, select your ESP32-C3 board.
3. Edit `WIFI_SSID` / `WIFI_PASSWORD` if desired (defaults to `Drone_FC_01` / `dronefly1`).
4. **Remove the propellers.**
5. Flash and open Serial Monitor at 115200 baud.
6. On boot the firmware will: init the MPU6050, calibrate gyro bias (keep the drone still and level), arm the ESCs, start the Wi-Fi AP, and start the UDP listener on port 4210.

## Control Protocol (UDP, port 4210)

ASCII, newline-terminated:

```
T:<0-1000>,R:<-500-500>,P:<-500-500>,Y:<-500-500>,F:<flags>
```

- `T` Throttle, `R` Roll, `P` Pitch, `Y` Yaw
- `F` flags bitmask: bit0 = ARM, bit1 = MODE (0=angle, 1=acro), bit2 = KILL

Example — armed, 40% throttle, slight roll right:
```
T:400,R:30,P:0,Y:0,F:1
```

## Telemetry (Serial @115200 + UDP port 4211, 1 Hz)

```
ATT:<roll>,<pitch>,<yaw>|RC:<t>,<r>,<p>,<y>|MOT:<m0..m3>|VBAT:<v>V|<ARM_STATE>|<MODE>|FS:<0/1>
```

## Flight Modes

- **ANGLE** — pilot stick commands a target lean angle; outer P-loop converts angle error to a rate setpoint, inner PID tracks that rate. Self-levelling, beginner-friendly.
- **ACRO** — pilot stick commands rotation rate directly. No self-levelling, full aerobatic authority.

## Arm / Disarm State Machine

```
DISARMED → (arm flag + throttle < 5%) → ARMING
ARMING   → hold for 1s → ARMED
ARMING   → arm released or throttle raised → DISARMED
ARMED    → arm flag removed → DISARMED
ANY      → kill switch OR UDP silence > 300ms → DISARMED
```

## First-Flight Checklist

1. **No props fitted.** Power on, watch Serial for attitude angles.
2. Tilt drone right → roll should read positive. Tilt nose up → pitch positive. Yaw CW → yaw positive. If any axis is inverted, flip the matching `*_SIGN` `#define`.
3. Connect your phone to SSID `Drone_FC_01`, open a UDP joystick app pointed at port 4210.
4. Arm: send the arm flag with throttle < 5% and hold for 1 second.
5. Raise throttle slowly — motors should respond symmetrically.
6. Test the kill switch — it should cut motors instantly. Always verify this before flying.
7. Only fit propellers after steps 1–6 pass.
8. Tune PIDs: Kp first, add Ki once stable, Kd last.

## PID Tuning

Default gains are starting points only — **every airframe needs its own tuning**.

- Outer loop (angle → rate): pure P, `ROLL_ANGLE_KP` / `PITCH_ANGLE_KP`
- Inner loop (rate → motor): full PID per axis, `*_RATE_KP/KI/KD`
- Tuning order: raise Kp until oscillation starts, back off ~30%, then slowly add Ki to kill steady-state offset, then add Kd last (often left at 0 for small whoops).

## Safety Notes

- Always test without propellers first.
- Always verify the kill switch before every flight.
- The failsafe disarms automatically if UDP packets stop for >300 ms — don't rely on this as your primary stop method; use the kill switch.
- LiPo batteries are this build's main fire/injury risk — use a balance charger, never leave charging unattended, and respect the low-voltage warning.

## Configuration Reference

All tunables live in one `§2 CONFIGURATION` block at the top of the sketch — Wi-Fi credentials, GPIO pins, ESC PWM settings, IMU full-scale ranges and axis signs, filter coefficient, loop rates, PID gains, RC deadzone, failsafe timing, and battery divider values. Edit there; the rest of the code reads from those defines.

## License

No license specified — add one (e.g. MIT) before sharing or open-sourcing this project.
