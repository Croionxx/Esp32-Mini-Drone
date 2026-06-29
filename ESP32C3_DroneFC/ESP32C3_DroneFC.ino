/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║             ESP32-C3  MINI DRONE  FLIGHT CONTROLLER  v1.0               ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  Target board : ESP32-C3 Super Mini (or any ESP32-C3 DevKit)            ║
 * ║  IDE          : Arduino IDE 2.x  +  arduino-esp32 board package v2.x   ║
 * ║  NOTE (v3.x)  : If you are on arduino-esp32 v3.x, the LEDC and timer   ║
 * ║                 APIs changed. See migration comments in setup().         ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  HARDWARE BOM                                                            ║
 * ║    MCU      ESP32-C3 Super Mini                                          ║
 * ║    IMU      MPU6050 (I²C, address 0x68)                                 ║
 * ║    ESC      4-in-1 BLHeli_S  5–10 A   (standard 1000–2000 µs PWM)      ║
 * ║    Motors   1103 brushless  ~8000 KV  (X-configuration)                 ║
 * ║    Battery  1S LiPo 450–650 mAh                                         ║
 * ║    Props    40–65 mm matching motor recommendation                       ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  MOTOR LAYOUT  (top view, X-config)                                     ║
 * ║                                                                          ║
 * ║              ── FRONT ──                                                 ║
 * ║        M0 (CW) ●         ● M1 (CCW)                                     ║
 * ║                  \     /                                                 ║
 * ║                  /     \                                                 ║
 * ║       M2 (CCW) ●         ● M3 (CW)                                      ║
 * ║              ── BACK ──                                                  ║
 * ║                                                                          ║
 * ║  Motor index → LEDC channel → GPIO pin:                                 ║
 * ║    M0 Front-Left  CW  → ch0 → GPIO7                                     ║
 * ║    M1 Front-Right CCW → ch1 → GPIO8                                     ║
 * ║    M2 Rear-Left   CCW → ch2 → GPIO6                                     ║
 * ║    M3 Rear-Right  CW  → ch3 → GPIO9                                     ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  CONTROL ARCHITECTURE                                                    ║
 * ║    500 Hz flight loop  (hardware timer ISR flag, deterministic)          ║
 * ║    100 Hz UDP polling  (Wi-Fi joystick input)                            ║
 * ║     20 Hz slow tasks   (battery, LED, telemetry)                         ║
 * ║                                                                          ║
 * ║    Sensor fusion  : Complementary filter  α = 0.98                      ║
 * ║    Control law    : Cascade PID                                          ║
 * ║      Outer loop   : Angle P only  →  rate setpoint  [°/s]               ║
 * ║      Inner loop   : Full PID on rate error            → motor mix        ║
 * ║    Flight modes   : ANGLE (self-levelling)  |  ACRO (rate direct)       ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  UDP COMMAND PROTOCOL  (ASCII, newline-terminated, port 4210)           ║
 * ║    Format : T:<0–1000>,R:<-500–500>,P:<-500–500>,Y:<-500–500>,F:<n>    ║
 * ║    Fields : T=Throttle  R=Roll  P=Pitch  Y=Yaw                          ║
 * ║    Flags  : bit0=ARM  bit1=MODE(0=angle/1=acro)  bit2=KILL              ║
 * ║    Example: T:400,R:30,P:0,Y:0,F:1   (armed, 40% throttle, slight roll) ║
 * ║                                                                          ║
 * ║  TELEMETRY  (Serial 115200 baud + UDP port 4211, 1 Hz)                  ║
 * ║    ATT:<r>,<p>,<y>|RC:<t>,<r>,<p>,<y>|MOT:<m0..m3>|VBAT:<v>V|...      ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  FIRST-FLIGHT CHECKLIST                                                  ║
 * ║    1. NO PROPS. Power on. Watch Serial for attitude angles.              ║
 * ║    2. Tilt drone right → ATT roll should go POSITIVE.                   ║
 * ║       Tilt nose up    → ATT pitch should go POSITIVE.                   ║
 * ║       Yaw CW          → ATT yaw should go POSITIVE.                     ║
 * ║       If any axis is inverted, flip the matching SIGN define below.     ║
 * ║    3. Connect phone to SSID "Drone_FC_01", open UDP joystick app.       ║
 * ║    4. Arm: send arm flag + throttle < 5% and hold for 1 second.        ║
 * ║    5. Raise throttle slowly. Motors should respond symmetrically.       ║
 * ║    6. Kill switch immediately cuts motors. Always test it first.        ║
 * ║    7. Fit props only after steps 1–6 pass.                              ║
 * ║    8. Tune PIDs: start with only Kp, add Ki when stable, Kd last.      ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

// ════════════════════════════════════════════════════════════════════════════
//  §1  INCLUDES
// ════════════════════════════════════════════════════════════════════════════
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// ════════════════════════════════════════════════════════════════════════════
//  §2  CONFIGURATION  ← edit all tuneable parameters here, nowhere else
// ════════════════════════════════════════════════════════════════════════════

// ── Wi-Fi ──────────────────────────────────────────────────────────────────
#define WIFI_SSID           "Drone_FC_01"
#define WIFI_PASSWORD       "dronefly1"
#define UDP_RX_PORT         4210      // Joystick → ESP32
#define UDP_TX_PORT         4211      // ESP32 → GCS (telemetry)

// ── GPIO pins ───────────────────────────────────────────────────────────────
//   Adjust these to match your exact ESP32-C3 board breakout.
#define PIN_SDA             4
#define PIN_SCL             5
#define PIN_MOTOR_0         7         // Front-Left  (CW)   — GPIO7 confirmed FL on bench test
#define PIN_MOTOR_1         8         // Front-Right (CCW)  — GPIO8 confirmed FR on bench test
#define PIN_MOTOR_2         6         // Rear-Left   (CCW)  — GPIO6 confirmed RL on bench test
#define PIN_MOTOR_3         9         // Rear-Right  (CW)   — GPIO9 confirmed RR on bench test (unchanged)
#define PIN_BUZZER          0
#define PIN_LED             2
#define PIN_BAT_ADC         1         // Must be an ADC1 pin on ESP32-C3

// ── ESC / PWM ───────────────────────────────────────────────────────────────
//   BLHeli_S supports 50–400 Hz standard PWM.
//   250 Hz offers better response than 50 Hz without risking ESC compatibility.
#define ESC_PWM_HZ          250
#define ESC_PWM_BITS        16        // 16-bit duty resolution
#define ESC_PULSE_MIN_US    1000      // Motors off  (1000 µs)
#define ESC_PULSE_MAX_US    2000      // Full power  (2000 µs)
#define ESC_PULSE_ARM_US    900       // Arm pulse   (< min, sent at startup)
#define ESC_PERIOD_US       (1000000UL / ESC_PWM_HZ)  // 4000 µs at 250 Hz

// ── IMU (MPU6050) ──────────────────────────────────────────────────────────
#define MPU_I2C_ADDR        0x68
#define I2C_CLK_HZ          400000   // 400 kHz fast-mode

// Full-scale range selectors (0 = smallest/most precise, 3 = widest)
#define GYRO_FS_SEL         1         // 0=±250  1=±500  2=±1000  3=±2000 °/s
#define ACCEL_FS_SEL        1         // 0=±2 g  1=±4 g  2=±8 g   3=±16 g

// Lookup tables for sensitivity (LSB / unit)
static const float GYRO_SENS[]  = { 131.0f, 65.5f, 32.8f, 16.4f   };
static const float ACCEL_SENS[] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };

#define GYRO_SCALE          (1.0f / GYRO_SENS[GYRO_FS_SEL])    // °/s per LSB
#define ACCEL_SCALE         (1.0f / ACCEL_SENS[ACCEL_FS_SEL])  // g   per LSB
#define IMU_CALIB_SAMPLES   2000      // Samples for gyro bias (drone must be still)

// ── IMU AXIS SIGNS ─────────────────────────────────────────────────────────
//   Physical mounting determines which direction is positive.
//   TEST WITHOUT PROPS: tilt drone right → roll must be positive.
//   Tilt nose up → pitch must be positive.  Spin CW → yaw must be positive.
//   Flip any of these to -1 if an axis is backwards.
#define GYRO_X_SIGN         (+1)      // Roll-rate  sign
#define GYRO_Y_SIGN         (+1)      // Pitch-rate sign
#define GYRO_Z_SIGN         (+1)      // Yaw-rate   sign
#define ACCEL_X_SIGN        (+1)
#define ACCEL_Y_SIGN        (+1)
#define ACCEL_Z_SIGN        (+1)

// ── Complementary filter ───────────────────────────────────────────────────
//   α = 0.98 means 98 % gyro, 2 % accelerometer per cycle.
//   Increase α (→1) for faster gyro tracking; decrease for less drift.
#define CF_ALPHA            0.98f

// ── Loop rates ─────────────────────────────────────────────────────────────
#define FLIGHT_LOOP_HZ      500       // Control loop  (500 Hz, hardware timer)
#define UDP_POLL_MS         10        // UDP polling   (100 Hz)
#define SLOW_LOOP_MS        50        // Slow tasks    ( 20 Hz)
static const float DT = 1.0f / FLIGHT_LOOP_HZ;

// ── Cascade PID — OUTER (angle P → rate setpoint) ─────────────────────────
//   Error: desired_angle − current_angle  [°]
//   Output: rate_setpoint  [°/s]
//   NO integral, NO derivative in this outer loop.
#define ROLL_ANGLE_KP       6.0f
#define PITCH_ANGLE_KP      6.0f
#define MAX_RATE_ROLL       250.0f    // Cap on generated rate setpoint [°/s]
#define MAX_RATE_PITCH      250.0f

// ── Cascade PID — INNER (rate PID → motor correction) ─────────────────────
//   Error: rate_setpoint − gyro_rate  [°/s]
//   Output: normalised motor correction  [−1 … +1], scaled by OUT_LIMIT
//
//   *** THESE GAINS MUST BE TUNED ON YOUR SPECIFIC BUILD ***
//   Tuning order: start all I and D at zero. Raise Kp until oscillation,
//   then back off ~30 %. Add Ki slowly to remove steady-state offset.
//   Add Kd last; it is often zero for small whoops.
#define ROLL_RATE_KP        0.0013f
#define ROLL_RATE_KI        0.0004f
#define ROLL_RATE_KD        0.0006f

#define PITCH_RATE_KP       0.0013f
#define PITCH_RATE_KI       0.0004f
#define PITCH_RATE_KD       0.0006f

// Yaw: rate-only PID (no outer angle loop — no magnetometer)
#define YAW_RATE_KP         0.0020f
#define YAW_RATE_KI         0.0008f
#define YAW_RATE_KD         0.0f

// Per-axis output clamp (max control authority added to throttle per axis)
#define ROLL_OUT_LIMIT      0.30f
#define PITCH_OUT_LIMIT     0.30f
#define YAW_OUT_LIMIT       0.25f
#define RATE_INTEG_LIMIT    0.10f     // Anti-windup clamp on integral term

// ── Angle-mode pilot limits ────────────────────────────────────────────────
#define MAX_ANGLE_ROLL_DEG  25.0f
#define MAX_ANGLE_PITCH_DEG 25.0f
#define MAX_YAW_RATE_DPS    120.0f    // Yaw rate at full stick [°/s]

// ── Acro-mode pilot limits ─────────────────────────────────────────────────
#define ACRO_MAX_RATE_RP    400.0f    // Full stick roll/pitch rate [°/s]
#define ACRO_MAX_RATE_YAW   200.0f

// ── Throttle ───────────────────────────────────────────────────────────────
#define MOTOR_IDLE_FRAC     0.05f     // Idle speed when armed (prevents cut-out)
#define MOTOR_MIN_FRAC      0.0f
#define MOTOR_MAX_FRAC      1.0f

// ── RC dead-zone ───────────────────────────────────────────────────────────
//   Tiny joystick drift will not cause corrections inside this band.
#define RC_DEADZONE         0.03f     // ±3 % of full-stick

// ── Safety ─────────────────────────────────────────────────────────────────
#define FAILSAFE_MS         300       // UDP silence → failsafe & disarm
#define ARM_HOLD_MS         1000      // Must hold arm+low-throttle this long

// ── Battery monitoring ─────────────────────────────────────────────────────
#define BAT_CELLS           1
#define BAT_WARN_V_CELL     3.50f     // Warn below this per cell [V]
#define BAT_R_TOP           100000.0f // Voltage divider top   resistor [Ω]
#define BAT_R_BOT            47000.0f // Voltage divider bottom resistor [Ω]
#define ADC_VREF            3.3f      // ESP32-C3 ADC reference [V]
#define ADC_BITS            12        // Resolution

// ════════════════════════════════════════════════════════════════════════════
//  §3  TYPES
// ════════════════════════════════════════════════════════════════════════════

enum FlightMode { MODE_ANGLE = 0, MODE_ACRO = 1 };
enum ArmState   { DISARMED  = 0, ARMING   = 1, ARMED = 2 };

struct RcCommand {
    float      throttle;   // [0, 1]
    float      roll;       // [−1, +1]
    float      pitch;      // [−1, +1]
    float      yaw;        // [−1, +1]
    bool       arm;
    bool       kill;
    FlightMode mode;
};

struct ImuData {
    float ax, ay, az;     // accelerometer [g]
    float gx, gy, gz;     // gyroscope     [°/s]  (bias already removed)
};

struct Attitude {
    float roll;            // [°]  + = right lean
    float pitch;           // [°]  + = nose up
    float yaw;             // [°]  + = CW from above,  wraps ±180
};

struct PidState {
    float integral;
    float prevError;
};

struct MotorCmds {
    float m[4];            // [0, 1] indexed 0–3, matching MOTOR_PINS[0–3]
};

// ════════════════════════════════════════════════════════════════════════════
//  §4  GLOBALS
// ════════════════════════════════════════════════════════════════════════════

// --- Timer ---
hw_timer_t *g_timer    = nullptr;
volatile bool g_doFlight = false;    // Set by ISR, cleared in loop()

// --- Flight state ---
RcCommand  g_rc       = {};
ImuData    g_imu      = {};
Attitude   g_att      = {};
MotorCmds  g_motors   = {};

ArmState   g_armState  = DISARMED;
FlightMode g_mode      = MODE_ANGLE;

// --- Cascade PID states ---
//   Outer loop (angle → rate):  pure P, no persistent state needed,
//   but kept as PidState for uniform API.
PidState g_outRoll = {}, g_outPitch = {};
//   Inner loop (rate → motor):
PidState g_inRoll  = {}, g_inPitch  = {}, g_inYaw = {};

// --- IMU calibration ---
float g_biasGx = 0, g_biasGy = 0, g_biasGz = 0;

// --- Battery ---
float g_batV = 0.0f;

// --- Timing ---
uint32_t g_lastUdpPollMs = 0;
uint32_t g_lastSlowMs    = 0;
uint32_t g_lastRxMs      = 0;   // Last good UDP packet — failsafe reference

// --- Wi-Fi / UDP ---
WiFiUDP  g_udp;
IPAddress g_remoteIp;
char     g_rxBuf[128];
char     g_txBuf[256];

// ════════════════════════════════════════════════════════════════════════════
//  §5  HARDWARE TIMER ISR
//      Placed in IRAM for deterministic latency on ESP32-C3 (RISC-V).
//      Only sets a flag — all heavy work runs in loop() context.
// ════════════════════════════════════════════════════════════════════════════

void IRAM_ATTR onFlightTimer() {
    g_doFlight = true;
}

// ════════════════════════════════════════════════════════════════════════════
//  §6  ESC / LEDC HELPERS  (arduino-esp32 v3.x API)
//
//  v2.x used explicit channel numbers:
//    ledcSetup(ch, freq, bits) → ledcAttachPin(pin, ch) → ledcWrite(ch, duty)
//
//  v3.x is pin-centric — channels are assigned automatically by the driver:
//    ledcAttach(pin, freq, bits)  ← combines setup + attach in one call
//    ledcWrite(pin, duty)         ← pin is the key, not channel number
//
//  MOTOR_PINS[] is the single source of truth that maps
//  motor index 0–3 to the GPIO it is wired to.
// ════════════════════════════════════════════════════════════════════════════

// Central motor-to-pin map (index 0–3 matches MotorCmds.m[0–3])
static const uint8_t MOTOR_PINS[4] = {
    PIN_MOTOR_0,   // M0  Front-Left  (CW)
    PIN_MOTOR_1,   // M1  Front-Right (CCW)
    PIN_MOTOR_2,   // M2  Rear-Left   (CCW)
    PIN_MOTOR_3    // M3  Rear-Right  (CW)
};

// Convert pulse width [µs] to 16-bit LEDC duty count
static inline uint32_t usToDuty(uint32_t us) {
    const uint32_t maxCnt = (1UL << ESC_PWM_BITS) - 1;
    return (uint32_t)(((uint64_t)us * maxCnt) / ESC_PERIOD_US);
}

// Attach all four motor pins to the LEDC peripheral and configure frequency
void escInit() {
    for (int i = 0; i < 4; i++) {
        // v3.x: single call configures frequency, resolution, and attaches pin
        ledcAttach(MOTOR_PINS[i], ESC_PWM_HZ, ESC_PWM_BITS);
    }
}

// Write a raw pulse width [µs] to one ESC, identified by its GPIO pin
static void escWriteUs(uint8_t pin, uint32_t us) {
    us = constrain(us, (uint32_t)ESC_PULSE_ARM_US, (uint32_t)ESC_PULSE_MAX_US);
    ledcWrite(pin, usToDuty(us));
}

// Write a normalised throttle [0.0, 1.0] to one ESC, identified by GPIO pin
static void escWriteNorm(uint8_t pin, float norm) {
    norm = constrain(norm, 0.0f, 1.0f);
    uint32_t us = (uint32_t)(ESC_PULSE_MIN_US
                  + norm * (ESC_PULSE_MAX_US - ESC_PULSE_MIN_US));
    ledcWrite(pin, usToDuty(us));
}

// Send the arm/disarm (below-minimum) pulse to all four ESCs
void allEscOff() {
    const uint32_t d = usToDuty(ESC_PULSE_ARM_US);
    for (int i = 0; i < 4; i++) ledcWrite(MOTOR_PINS[i], d);
}

// Push a full MotorCmds struct to all four ESCs
void applyMotors(const MotorCmds &m) {
    for (int i = 0; i < 4; i++) escWriteNorm(MOTOR_PINS[i], m.m[i]);
}

// ════════════════════════════════════════════════════════════════════════════
//  §7  MPU6050 DRIVER  (no external library — direct register I²C)
// ════════════════════════════════════════════════════════════════════════════

static bool mpuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool mpuBurstRead(uint8_t startReg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_I2C_ADDR, len);
    if (Wire.available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool mpuInit() {
    // 0x6B  PWR_MGMT_1 = 0x00 → wake from sleep
    if (!mpuWriteReg(0x6B, 0x00)) return false;
    delay(100);
    // Use PLL with X-gyro for better clock stability
    mpuWriteReg(0x6B, 0x01);
    // 0x1A  CONFIG: DLPF_CFG = 3 → ~42 Hz gyro bandwidth, 1 kHz sample rate
    mpuWriteReg(0x1A, 0x03);
    // 0x1B  GYRO_CONFIG: full-scale select
    mpuWriteReg(0x1B, (uint8_t)(GYRO_FS_SEL  << 3));
    // 0x1C  ACCEL_CONFIG: full-scale select
    mpuWriteReg(0x1C, (uint8_t)(ACCEL_FS_SEL << 3));
    // 0x19  SMPLRT_DIV = 0 → sample at full 1 kHz (DLPF active)
    mpuWriteReg(0x19, 0x00);
    delay(50);
    return true;
}

// Read accelerometer + gyroscope in one 14-byte burst.
// Returns false on I²C error; caller should skip the current cycle.
bool mpuRead(ImuData &out) {
    uint8_t buf[14];
    if (!mpuBurstRead(0x3B, buf, 14)) return false;

    auto to16 = [](uint8_t hi, uint8_t lo) -> int16_t {
        return (int16_t)((hi << 8) | lo);
    };

    float ax = to16(buf[0],  buf[1])  * ACCEL_SCALE * ACCEL_X_SIGN;
    float ay = to16(buf[2],  buf[3])  * ACCEL_SCALE * ACCEL_Y_SIGN;
    float az = to16(buf[4],  buf[5])  * ACCEL_SCALE * ACCEL_Z_SIGN;
    // buf[6:7] = temperature → skipped
    float gx = to16(buf[8],  buf[9])  * GYRO_SCALE  * GYRO_X_SIGN - g_biasGx;
    float gy = to16(buf[10], buf[11]) * GYRO_SCALE  * GYRO_Y_SIGN - g_biasGy;
    float gz = to16(buf[12], buf[13]) * GYRO_SCALE  * GYRO_Z_SIGN - g_biasGz;

    out.ax = ax; out.ay = ay; out.az = az;
    out.gx = gx; out.gy = gy; out.gz = gz;
    return true;
}

// Gyro bias calibration — drone must be flat and stationary.
void mpuCalibrate() {
    ImuData tmp;
    double sumGx = 0, sumGy = 0, sumGz = 0;

    Serial.println(F("[IMU] Calibrating — keep drone still and level..."));

    // Discard 200 warm-up samples
    for (int i = 0; i < 200; i++) { mpuRead(tmp); delayMicroseconds(1000); }

    for (int i = 0; i < IMU_CALIB_SAMPLES; i++) {
        mpuRead(tmp);
        // Accumulate raw (before bias removal), adding current bias back
        sumGx += tmp.gx + g_biasGx;
        sumGy += tmp.gy + g_biasGy;
        sumGz += tmp.gz + g_biasGz;
        delayMicroseconds(500);
        if (i % 400 == 0) {
            Serial.print('.');
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        }
    }
    g_biasGx = (float)(sumGx / IMU_CALIB_SAMPLES);
    g_biasGy = (float)(sumGy / IMU_CALIB_SAMPLES);
    g_biasGz = (float)(sumGz / IMU_CALIB_SAMPLES);
    Serial.printf("\n[IMU] Bias  gx=%.4f  gy=%.4f  gz=%.4f  °/s\n",
                  g_biasGx, g_biasGy, g_biasGz);
}

// ════════════════════════════════════════════════════════════════════════════
//  §8  COMPLEMENTARY FILTER
//
//  Why complementary and not Kalman?
//  Kalman is optimal for Gaussian noise but overkill here:
//    • The complementary filter requires no matrix operations.
//    • At 500 Hz the gyro drift over 2 ms is negligible.
//    • It is deterministic — critical for real-time control.
//  α = 0.98 is the standard starting point for small drones.
// ════════════════════════════════════════════════════════════════════════════

void updateAttitude(const ImuData &imu, Attitude &att, float dt) {
    // Tilt angles from accelerometer (valid only during near-zero acceleration)
    //   roll  = atan2( ay,  az )          — rotation about X-axis
    //   pitch = atan2(-ax, √(ay²+az²) )  — rotation about Y-axis
    float rollAcc  =  atan2f(imu.ay, imu.az) * (180.0f / (float)M_PI);
    float pitchAcc = -atan2f(imu.ax,
                     sqrtf(imu.ay * imu.ay + imu.az * imu.az))
                     * (180.0f / (float)M_PI);

    // Fuse: gyro integration corrected by accelerometer
    att.roll  = CF_ALPHA * (att.roll  + imu.gx * dt) + (1.0f - CF_ALPHA) * rollAcc;
    att.pitch = CF_ALPHA * (att.pitch + imu.gy * dt) + (1.0f - CF_ALPHA) * pitchAcc;

    // Yaw: integrate only — no magnetometer correction available
    att.yaw += imu.gz * dt;
    if (att.yaw >  180.0f) att.yaw -= 360.0f;
    if (att.yaw < -180.0f) att.yaw += 360.0f;
}

// ════════════════════════════════════════════════════════════════════════════
//  §9  PID CONTROLLER
//
//  Uses derivative-on-error (not on measurement) for simplicity on v1.0.
//  Improvement: switch to derivative-on-measurement by keeping prevMeasured
//  and using  dError = -(measured − prevMeasured) / dt  to avoid
//  "derivative kick" when the setpoint changes suddenly.
// ════════════════════════════════════════════════════════════════════════════

float pidUpdate(PidState &s,
                float setpoint, float measured,
                float kp, float ki, float kd,
                float dt,
                float integLimit, float outLimit) {
    float error  = setpoint - measured;
    float dError = (error - s.prevError) / dt;

    s.integral  += error * dt;
    s.integral   = constrain(s.integral, -integLimit, integLimit);

    float out = kp * error + ki * s.integral + kd * dError;
    out = constrain(out, -outLimit, outLimit);

    s.prevError = error;
    return out;
}

void pidReset(PidState &s) { s.integral = 0; s.prevError = 0; }

void resetAllPids() {
    pidReset(g_outRoll);  pidReset(g_outPitch);
    pidReset(g_inRoll);   pidReset(g_inPitch);   pidReset(g_inYaw);
}

// ════════════════════════════════════════════════════════════════════════════
//  §10  MOTOR MIXER  (X-configuration)
//
//  Sign conventions (all positive values):
//    +roll  → drone leans RIGHT  → left  motors (M0, M2) faster
//    +pitch → drone tilts FWD   → rear  motors (M2, M3) faster
//    +yaw   → drone yaws CW     → CCW   motors (M1, M2) faster
//
//  Mixing matrix:
//         Throttle  Roll  Pitch   Yaw
//    M0     +1       +1    −1     −1      Front-Left  (CW)
//    M1     +1       −1    −1     +1      Front-Right (CCW)
//    M2     +1       +1    +1     +1      Rear-Left   (CCW)
//    M3     +1       −1    +1     −1      Rear-Right  (CW)
//
//  De-saturation: if any motor would exceed [0,1], the entire set is
//  shifted uniformly so the constraint is met while preserving attitude
//  control authority as much as possible.
// ════════════════════════════════════════════════════════════════════════════

void motorMix(float thr, float roll, float pitch, float yaw, MotorCmds &out) {
    out.m[0] = thr + roll - pitch - yaw;
    out.m[1] = thr - roll - pitch + yaw;
    out.m[2] = thr + roll + pitch + yaw;
    out.m[3] = thr - roll + pitch - yaw;

    // Find min and max across all channels
    float hi = out.m[0], lo = out.m[0];
    for (int i = 1; i < 4; i++) {
        if (out.m[i] > hi) hi = out.m[i];
        if (out.m[i] < lo) lo = out.m[i];
    }

    // Shift down if ceiling exceeded
    if (hi > 1.0f) { float d = hi - 1.0f; for (int i=0;i<4;i++) out.m[i] -= d; }
    // Shift up  if floor violated
    if (lo < 0.0f) {                       for (int i=0;i<4;i++) out.m[i] -= lo; }

    // Hard clamp (should rarely trigger after desaturation)
    for (int i = 0; i < 4; i++)
        out.m[i] = constrain(out.m[i], MOTOR_MIN_FRAC, MOTOR_MAX_FRAC);
}

// ════════════════════════════════════════════════════════════════════════════
//  §11  RC INPUT PARSER
//
//  Format: T:<0–1000>,R:<-500–500>,P:<-500–500>,Y:<-500–500>,F:<flags>\n
//  Values normalised internally to [0,1] (throttle) and [−1,+1] (axes).
//  Flags bitmask: bit0=ARM  bit1=MODE(0=angle,1=acro)  bit2=KILL
//
//  To connect a different UDP app, replace parsePacket() with the app's
//  packet parser. Only this one function needs to change.
// ════════════════════════════════════════════════════════════════════════════

static float applyDeadzone(float v, float dz) {
    return (fabsf(v) < dz) ? 0.0f : v;
}

void parsePacket(const char *buf, RcCommand &cmd) {
    int T=0, R=0, P=0, Y=0, F=0;
    sscanf(buf, "T:%d,R:%d,P:%d,Y:%d,F:%d", &T, &R, &P, &Y, &F);

    cmd.throttle = constrain(T / 1000.0f, 0.0f, 1.0f);
    cmd.roll     = applyDeadzone(constrain(R / 500.0f, -1.0f, 1.0f), RC_DEADZONE);
    cmd.pitch    = applyDeadzone(constrain(P / 500.0f, -1.0f, 1.0f), RC_DEADZONE);
    cmd.yaw      = applyDeadzone(constrain(Y / 500.0f, -1.0f, 1.0f), RC_DEADZONE);
    cmd.arm      = (F & 0x01) != 0;
    cmd.mode     = ((F & 0x02) != 0) ? MODE_ACRO : MODE_ANGLE;
    cmd.kill     = (F & 0x04) != 0;
}

void pollUdp() {
    int len = g_udp.parsePacket();
    if (len > 0 && len < (int)sizeof(g_rxBuf) - 1) {
        g_remoteIp = g_udp.remoteIP();
        g_udp.read(g_rxBuf, len);
        g_rxBuf[len] = '\0';
        parsePacket(g_rxBuf, g_rc);
        g_lastRxMs = millis();
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  §12  FAILSAFE
// ════════════════════════════════════════════════════════════════════════════

bool isFailsafe() {
    // True when no valid UDP packet received within FAILSAFE_MS
    return (millis() - g_lastRxMs) > (uint32_t)FAILSAFE_MS;
}

void applyFailsafeCmd(RcCommand &cmd) {
    cmd.throttle = 0.0f;
    cmd.roll     = 0.0f;
    cmd.pitch    = 0.0f;
    cmd.yaw      = 0.0f;
    cmd.arm      = false;
}

// ════════════════════════════════════════════════════════════════════════════
//  §13  BATTERY MONITORING
//      Voltage divider: V_batt → R_top → node → R_bot → GND
//      V_adc = V_batt × R_bot / (R_top + R_bot)
//      ESP32-C3 ADC has ±5 % error without calibration.
//      For accurate readings, use espressif/esp_adc_cal or a known reference.
// ════════════════════════════════════════════════════════════════════════════

float readBatVoltage() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += analogRead(PIN_BAT_ADC);
    float vAdc = (float)(sum >> 4) * ADC_VREF / (float)((1 << ADC_BITS) - 1);
    return vAdc * (BAT_R_TOP + BAT_R_BOT) / BAT_R_BOT;
}

bool isBatLow() {
    return g_batV > 0.1f && g_batV < (BAT_WARN_V_CELL * BAT_CELLS);
}

// ════════════════════════════════════════════════════════════════════════════
//  §14  BUZZER  &  STATUS LED
// ════════════════════════════════════════════════════════════════════════════

void buzz(uint16_t onMs) {
    digitalWrite(PIN_BUZZER, HIGH); delay(onMs);
    digitalWrite(PIN_BUZZER, LOW);  delay(40);
}
void buzzerBeep(uint8_t n) { for (uint8_t i = 0; i < n; i++) buzz(80); }

void updateLed() {
    static uint32_t lastMs = 0;
    static bool     state  = false;
    uint32_t now = millis();

    // Blink period encodes state (faster = more urgent)
    uint16_t period;
    if      (g_armState == ARMED   && isBatLow())    period =   80;  // fast: low bat
    else if (g_armState == ARMED)                     period =  800;  // slow: armed OK
    else if (g_armState == ARMING)                    period =  250;  // medium: arming
    else                                              period = 2000;  // very slow: idle

    if (now - lastMs >= period) {
        lastMs = now;
        state  = !state;
        digitalWrite(PIN_LED, state);
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  §15  ARM / DISARM STATE MACHINE
//
//  DISARMED  ──(arm signal + throttle < 5 %)──►  ARMING
//  ARMING    ──(hold for ARM_HOLD_MS)──────────►  ARMED
//  ARMING    ──(arm released OR throttle raised)►  DISARMED
//  ARMED     ──(arm signal removed)────────────►  DISARMED
//  ANY       ──(kill switch OR failsafe)────────►  DISARMED
//
//  This requires a deliberate and sustained action to arm, preventing
//  accidental motor start from a glitch or dropped packet.
// ════════════════════════════════════════════════════════════════════════════

void handleArmDisarm() {
    static uint32_t armStartMs = 0;

    // ── Kill switch: instant, unconditional disarm ─────────────────────────
    if (g_rc.kill) {
        if (g_armState != DISARMED)
            Serial.println(F("[ARM] Kill switch → DISARMED"));
        g_armState = DISARMED;
        resetAllPids();
        allEscOff();
        return;
    }

    // ── Failsafe: disarm if link is lost while flying ──────────────────────
    if (isFailsafe() && g_armState == ARMED) {
        Serial.printf("[ARM] FAILSAFE (no UDP for %lu ms) → DISARMED\n",
                      millis() - g_lastRxMs);
        g_armState = DISARMED;
        resetAllPids();
        allEscOff();
        buzzerBeep(3);      // Three beeps = failsafe event
        return;
    }

    // ── Normal state machine ───────────────────────────────────────────────
    switch (g_armState) {

        case DISARMED:
            if (g_rc.arm && g_rc.throttle < 0.05f) {
                g_armState = ARMING;
                armStartMs = millis();
                Serial.println(F("[ARM] Arming… hold arm + low throttle for 1 s"));
            }
            break;

        case ARMING:
            if (!g_rc.arm || g_rc.throttle >= 0.05f) {
                g_armState = DISARMED;
                Serial.println(F("[ARM] Arm cancelled"));
            } else if (millis() - armStartMs >= ARM_HOLD_MS) {
                g_armState = ARMED;
                Serial.println(F("[ARM] *** ARMED — throttle up carefully ***"));
                buzzerBeep(2);   // Two beeps = armed
            }
            break;

        case ARMED:
            if (!g_rc.arm) {
                g_armState = DISARMED;
                resetAllPids();
                allEscOff();
                Serial.println(F("[ARM] Disarmed by pilot"));
            }
            break;
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  §16  FLIGHT CONTROL LOOP  (runs at FLIGHT_LOOP_HZ via timer flag)
//
//  Execution budget at 500 Hz: 2000 µs per cycle.
//  Typical I²C read at 400 kHz (14 bytes): ~400 µs.
//  Remaining for filter + PID + mixer + ESC: ~1600 µs → comfortable.
// ════════════════════════════════════════════════════════════════════════════

void runFlightLoop() {

    // ── 1. Read IMU ────────────────────────────────────────────────────────
    //   On I²C error, skip this cycle rather than use stale data.
    //   A single missed cycle at 500 Hz is invisible to the pilot.
    if (!mpuRead(g_imu)) return;

    // ── 2. Attitude estimation ─────────────────────────────────────────────
    updateAttitude(g_imu, g_att, DT);

    // ── 3. If not armed, keep ESCs in arm-pulse state and exit ────────────
    if (g_armState != ARMED) {
        allEscOff();
        return;
    }

    // ── 4. Compute axis corrections ────────────────────────────────────────
    float rollOut, pitchOut, yawOut;

    if (g_mode == MODE_ANGLE) {
        // ── ANGLE MODE (self-levelling) ────────────────────────────────────
        // Outer P loop: angle error → rate setpoint
        float rollRateSP  = ROLL_ANGLE_KP  * (g_rc.roll  * MAX_ANGLE_ROLL_DEG  - g_att.roll);
        float pitchRateSP = PITCH_ANGLE_KP * (g_rc.pitch * MAX_ANGLE_PITCH_DEG - g_att.pitch);
        float yawRateSP   = g_rc.yaw * MAX_YAW_RATE_DPS;

        rollRateSP  = constrain(rollRateSP,  -MAX_RATE_ROLL,  MAX_RATE_ROLL);
        pitchRateSP = constrain(pitchRateSP, -MAX_RATE_PITCH, MAX_RATE_PITCH);

        // Inner rate PID: rate error → motor correction
        rollOut  = pidUpdate(g_inRoll,  rollRateSP,  g_imu.gx,
                             ROLL_RATE_KP,  ROLL_RATE_KI,  ROLL_RATE_KD,
                             DT, RATE_INTEG_LIMIT, ROLL_OUT_LIMIT);

        pitchOut = pidUpdate(g_inPitch, pitchRateSP, g_imu.gy,
                             PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD,
                             DT, RATE_INTEG_LIMIT, PITCH_OUT_LIMIT);

        yawOut   = pidUpdate(g_inYaw,   yawRateSP,   g_imu.gz,
                             YAW_RATE_KP,  YAW_RATE_KI,  YAW_RATE_KD,
                             DT, RATE_INTEG_LIMIT, YAW_OUT_LIMIT);

    } else {
        // ── ACRO MODE (rate-direct, no self-levelling) ─────────────────────
        // RC stick commands gyroscope rate directly.  No outer angle loop.
        float rollRateSP  = g_rc.roll  * ACRO_MAX_RATE_RP;
        float pitchRateSP = g_rc.pitch * ACRO_MAX_RATE_RP;
        float yawRateSP   = g_rc.yaw   * ACRO_MAX_RATE_YAW;

        rollOut  = pidUpdate(g_inRoll,  rollRateSP,  g_imu.gx,
                             ROLL_RATE_KP,  ROLL_RATE_KI,  ROLL_RATE_KD,
                             DT, RATE_INTEG_LIMIT, ROLL_OUT_LIMIT);

        pitchOut = pidUpdate(g_inPitch, pitchRateSP, g_imu.gy,
                             PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD,
                             DT, RATE_INTEG_LIMIT, PITCH_OUT_LIMIT);

        yawOut   = pidUpdate(g_inYaw,   yawRateSP,   g_imu.gz,
                             YAW_RATE_KP,  YAW_RATE_KI,  YAW_RATE_KD,
                             DT, RATE_INTEG_LIMIT, YAW_OUT_LIMIT);
    }

    // ── 5. Throttle floor — prevent motors from cutting out when armed ─────
    float throttle = g_rc.throttle;
    if (throttle < MOTOR_IDLE_FRAC) throttle = MOTOR_IDLE_FRAC;

    // ── 6. Mix & output ────────────────────────────────────────────────────
    motorMix(throttle, rollOut, pitchOut, yawOut, g_motors);
    applyMotors(g_motors);
}

// ════════════════════════════════════════════════════════════════════════════
//  §17  TELEMETRY
//  Serial always active (for tuning with Serial Plotter / monitor).
//  UDP telemetry is sent back to the controller's IP on UDP_TX_PORT.
// ════════════════════════════════════════════════════════════════════════════

void sendTelemetry() {
    static const char *const ARM_STR[]  = { "DISARMED", "ARMING", "ARMED" };
    static const char *const MODE_STR[] = { "ANGLE", "ACRO" };

    snprintf(g_txBuf, sizeof(g_txBuf),
        "ATT:%.1f,%.1f,%.1f|"
        "RC:%.2f,%.2f,%.2f,%.2f|"
        "MOT:%.2f,%.2f,%.2f,%.2f|"
        "VBAT:%.2fV|%s|%s|FS:%d\n",
        g_att.roll, g_att.pitch, g_att.yaw,
        g_rc.throttle, g_rc.roll, g_rc.pitch, g_rc.yaw,
        g_motors.m[0], g_motors.m[1], g_motors.m[2], g_motors.m[3],
        g_batV,
        ARM_STR[(int)g_armState],
        MODE_STR[(int)g_mode],
        (int)isFailsafe()
    );

    Serial.print(g_txBuf);

    // Send UDP only if we have a known remote IP (controller has spoken first)
    if ((uint32_t)g_remoteIp != 0) {
        g_udp.beginPacket(g_remoteIp, UDP_TX_PORT);
        g_udp.write((const uint8_t *)g_txBuf, strlen(g_txBuf));
        g_udp.endPacket();
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  §18  SETUP
// ════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n╔══════════════════════════════╗"));
    Serial.println(F("║  ESP32-C3 Drone FC  v1.0     ║"));
    Serial.println(F("╚══════════════════════════════╝"));

    // ── GPIO ──────────────────────────────────────────────────────────────
    pinMode(PIN_LED,    OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    analogReadResolution(ADC_BITS);
    digitalWrite(PIN_LED,    LOW);
    digitalWrite(PIN_BUZZER, LOW);

    // ── I²C ───────────────────────────────────────────────────────────────
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_CLK_HZ);

    // ── MPU6050 ───────────────────────────────────────────────────────────
    Serial.print(F("[IMU] Init ... "));
    {
        int tries = 0;
        while (!mpuInit() && ++tries < 5) {
            Serial.print(F("retry "));
            delay(500);
        }
        if (tries >= 5) {
            Serial.println(F("FATAL: MPU6050 not found! Check wiring and I²C address."));
            // Rapid LED blink = fatal error; only a power-cycle recovers
            while (true) {
                digitalWrite(PIN_LED, HIGH); delay(80);
                digitalWrite(PIN_LED, LOW);  delay(80);
            }
        }
    }
    Serial.println(F("OK"));
    mpuCalibrate();
    buzzerBeep(1);   // One beep = IMU ready

    // ── ESC initialisation ────────────────────────────────────────────────
    Serial.println(F("[ESC] Initialising LEDC channels..."));
    escInit();
    allEscOff();     // Send below-minimum pulse to allow BLHeli_S to arm
    Serial.println(F("[ESC] Sending arm pulse for 2 s (BLHeli_S requirement)..."));
    delay(2000);
    Serial.println(F("[ESC] Done"));
    buzzerBeep(1);   // One beep = ESC ready

    // ── Wi-Fi Access Point ────────────────────────────────────────────────
    Serial.printf("[WiFi] Starting AP: \"%s\" ...\n", WIFI_SSID);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] AP IP : %s\n", WiFi.softAPIP().toString().c_str());
    g_udp.begin(UDP_RX_PORT);
    Serial.printf("[WiFi] UDP RX port %d  TX port %d\n", UDP_RX_PORT, UDP_TX_PORT);

    // ── Initialise RC to safe state ───────────────────────────────────────
    memset(&g_rc, 0, sizeof(g_rc));
    g_lastRxMs = millis();   // Prevent false failsafe before first packet

    // ── PID reset ─────────────────────────────────────────────────────────
    resetAllPids();

    // ── Hardware timer for flight loop (arduino-esp32 v3.x API) ─────────────
    //
    //  v3.x mental model:
    //    timerBegin(frequency)  — you specify desired TICK FREQUENCY in Hz
    //                             (driver computes the prescaler for you)
    //    timerAttachInterrupt() — no longer takes an 'edge' boolean
    //    timerAlarm()           — replaces timerAlarmWrite() + timerAlarmEnable()
    //                             4th arg = reload_count (0 = infinite auto-reload)
    //
    //  Example at 500 Hz control loop:
    //    Tick frequency = 1 000 000 Hz  →  1 µs per tick
    //    Alarm ticks    = 1 000 000 / 500 = 2000  →  ISR fires every 2 ms
    {
        const uint32_t TICK_HZ  = 1000000UL;                // 1 MHz → 1 µs/tick
        const uint64_t ALARM    = TICK_HZ / FLIGHT_LOOP_HZ; // ticks per control cycle
        g_timer = timerBegin(TICK_HZ);
        timerAttachInterrupt(g_timer, &onFlightTimer);
        timerAlarm(g_timer, ALARM, /*autoreload=*/true, /*reload_count=*/0);
    }
    Serial.printf("[Timer] Flight loop at %d Hz (%.0f µs/cycle)\n",
                  FLIGHT_LOOP_HZ, 1e6f / FLIGHT_LOOP_HZ);

    // ── Ready ──────────────────────────────────────────────────────────────
    buzzerBeep(2);   // Two beeps = fully ready
    Serial.println(F("\n[FC] Ready. Connect phone to AP, send UDP packets to arm."));
    Serial.println(F("[FC] Serial telemetry at 1 Hz. Plotter: ATT roll/pitch/yaw."));
    Serial.println(F("─────────────────────────────────────────────────────────\n"));
}

// ════════════════════════════════════════════════════════════════════════════
//  §19  MAIN LOOP
//
//  The loop() function is the task scheduler.
//  Three rate tiers controlled by timer flag and millis() gating.
//
//  Why not FreeRTOS tasks?
//    ESP32-C3 is single-core. Task switching overhead and unpredictable
//    preemption would jitter the control loop more than a simple
//    cooperative scheduler like this.
// ════════════════════════════════════════════════════════════════════════════

void loop() {
    uint32_t now = millis();

    // ── 500 Hz: Flight control (gate on timer ISR flag) ───────────────────
    if (g_doFlight) {
        g_doFlight = false;
        g_mode     = g_rc.mode;   // Apply any mode switch from RC
        runFlightLoop();
    }

    // ── 100 Hz: UDP polling + failsafe + arm state machine ────────────────
    if (now - g_lastUdpPollMs >= UDP_POLL_MS) {
        g_lastUdpPollMs = now;
        pollUdp();
        if (isFailsafe()) applyFailsafeCmd(g_rc);
        handleArmDisarm();
    }

    // ── 20 Hz: Battery, LED, telemetry ───────────────────────────────────
    if (now - g_lastSlowMs >= SLOW_LOOP_MS) {
        g_lastSlowMs = now;
        g_batV = readBatVoltage();
        updateLed();

        // Telemetry at 1 Hz (every 20th slow iteration = 1000 ms)
        static uint8_t telemTick = 0;
        if (++telemTick >= 20) {
            telemTick = 0;
            sendTelemetry();
        }
    }
}
