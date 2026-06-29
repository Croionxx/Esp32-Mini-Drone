/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║      ESP32-C3  MOTOR TEST  —  Custom MOSFET Brushed-Motor Driver        ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  *** REMOVE PROPELLERS BEFORE RUNNING THIS TEST ***                     ║
 * ║                                                                          ║
 * ║  HARDWARE — this is NOT an RC ESC. There is no pulse-width protocol     ║
 * ║  to satisfy, no arming sequence, no "below-minimum disarm pulse."       ║
 * ║  Each GPIO drives an SI2300 N-MOSFET gate directly. The MOSFET drain    ║
 * ║  switches power straight to a 6mm coreless brushed motor. 1N4148        ║
 * ║  flyback diodes absorb the inductive kick when the gate turns off,      ║
 * ║  and 10kΩ gate pull-down resistors hold each MOSFET off whenever the    ║
 * ║  GPIO is floating (e.g. during reset, before the pin is configured).   ║
 * ║                                                                          ║
 * ║  Control model: the gate just wants a plain 0–100% duty-cycle PWM.      ║
 * ║    0%   duty → gate fully off → motor off                              ║
 * ║    100% duty → gate fully on  → full power                             ║
 * ║  There is no "minimum valid pulse width" concept here at all — that     ║
 * ║  was the RC-ESC model, and it doesn't apply to this board. Any nonzero  ║
 * ║  duty is genuinely nonzero power on a small low-resistance motor.       ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  PIN MAPPING (confirmed by bench test)                                  ║
 * ║    GPIO6 → RL motor gate                                                ║
 * ║    GPIO7 → FL motor gate                                                ║
 * ║    GPIO8 → FR motor gate                                                ║
 * ║    GPIO9 → RR motor gate                                                ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  HOW TO USE                                                              ║
 * ║    1. No props. Power on. Open Serial Monitor @ 115200 baud.            ║
 * ║    2. All four motors initialize to a TRUE 0% duty — confirm none are   ║
 * ║       spinning before you type anything. If one is, see note below.    ║
 * ║    3. Commands (type + Enter):                                         ║
 * ║         0/1/2/3 = spin RL/FL/FR/RR only, at TEST_DUTY, for 1.5 s        ║
 * ║         a       = spin all four together                               ║
 * ║         s       = stop immediately                                     ║
 * ║         +/-     = adjust TEST_DUTY by 5%  (capped 0–50%)                ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

// ── Pin mapping (confirmed by bench test) ───────────────────────────────────
#define PIN_FL   7   // GPIO7 — confirmed FL
#define PIN_FR   8   // GPIO8 — confirmed FR
#define PIN_RL   6   // GPIO6 — confirmed RL
#define PIN_RR   9   // GPIO9 — confirmed RR

// Array order: key '0'→RL  key '1'→FL  key '2'→FR  key '3'→RR
// (Happens to land on sequential GPIO 6,7,8,9 once the labels above are correct.)
static const uint8_t MOTOR_PINS[4]  = { PIN_RL, PIN_FL, PIN_FR, PIN_RR };
static const char   *MOTOR_NAMES[4] = { "RL",   "FL",   "FR",   "RR"   };

// ── PWM config — direct duty-cycle drive, no ESC protocol involved ─────────
//   20 kHz is above the audible range (no motor whine) and gives the
//   SI2300 plenty of switching margin for a small coreless motor's
//   inductance. 10-bit resolution = 1024 duty steps, smooth at low power.
#define PWM_FREQ_HZ   20000
#define PWM_BITS      10
#define PWM_MAX       ((1u << PWM_BITS) - 1)   // 1023

// ── Test parameters ─────────────────────────────────────────────────────────
float g_testDuty = 0.15f;        // 15% — enough to confirm spin, not violent
#define TEST_RUN_MS    1500

// ════════════════════════════════════════════════════════════════════════════
//  Driver helpers
// ════════════════════════════════════════════════════════════════════════════

void escInit() {
    for (int i = 0; i < 4; i++) {
        ledcAttach(MOTOR_PINS[i], PWM_FREQ_HZ, PWM_BITS);
        ledcWrite(MOTOR_PINS[i], 0);    // force a genuine 0% duty immediately —
                                         // no "almost off" floor, no ambiguity
    }
}

void writeDuty(uint8_t pin, float norm) {
    norm = constrain(norm, 0.0f, 1.0f);
    ledcWrite(pin, (uint32_t)(norm * PWM_MAX));
}

void allMotorsOff() {
    for (int i = 0; i < 4; i++) ledcWrite(MOTOR_PINS[i], 0);
}

// ════════════════════════════════════════════════════════════════════════════
//  Test actions
// ════════════════════════════════════════════════════════════════════════════

void spinOne(uint8_t idx) {
    if (idx > 3) return;
    Serial.printf("[TEST] %s @ %.0f%% duty for %d ms\n",
                  MOTOR_NAMES[idx], g_testDuty * 100.0f, TEST_RUN_MS);
    writeDuty(MOTOR_PINS[idx], g_testDuty);
    delay(TEST_RUN_MS);
    ledcWrite(MOTOR_PINS[idx], 0);
    Serial.println(F("[TEST] Stopped."));
}

void spinAll() {
    Serial.printf("[TEST] ALL motors @ %.0f%% duty for %d ms\n",
                  g_testDuty * 100.0f, TEST_RUN_MS);
    for (int i = 0; i < 4; i++) writeDuty(MOTOR_PINS[i], g_testDuty);
    delay(TEST_RUN_MS);
    allMotorsOff();
    Serial.println(F("[TEST] Stopped."));
}

void stopAll() {
    allMotorsOff();
    Serial.println(F("[TEST] All motors stopped (0% duty)."));
}

void printMenu() {
    Serial.println(F("\n──────────────────────────────────────────"));
    Serial.printf( "  Test duty: %.0f%%\n", g_testDuty * 100.0f);
    Serial.println(F("  0=RL 1=FL 2=FR 3=RR   a=all   s=stop"));
    Serial.println(F("  + / - = adjust test duty"));
    Serial.println(F("──────────────────────────────────────────"));
}

// ════════════════════════════════════════════════════════════════════════════
//  Setup / Loop
// ════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n=== ESP32-C3 Motor Test — Custom MOSFET Driver ==="));
    Serial.println(F("*** Confirm propellers are REMOVED before continuing ***"));

    escInit();   // all four pins forced to 0% duty here

    Serial.println(F("[INIT] All motors at 0% duty."));
    Serial.println(F("[CHECK] None of the motors should be spinning right now."));
    Serial.println(F("        If one is, stop power immediately — see wiring note."));

    Serial.println(F("\n[READY] Type a command and press Enter."));
    printMenu();
}

void loop() {
    if (!Serial.available()) return;

    char c = Serial.read();
    if (c == '\n' || c == '\r') return;

    switch (c) {
        case '0': spinOne(0); break;
        case '1': spinOne(1); break;
        case '2': spinOne(2); break;
        case '3': spinOne(3); break;
        case 'a': case 'A': spinAll(); break;
        case 's': case 'S': stopAll(); break;
        case '+':
            g_testDuty = constrain(g_testDuty + 0.05f, 0.0f, 0.5f);
            Serial.printf("[CFG] Test duty = %.0f%%\n", g_testDuty * 100.0f);
            break;
        case '-':
            g_testDuty = constrain(g_testDuty - 0.05f, 0.0f, 0.5f);
            Serial.printf("[CFG] Test duty = %.0f%%\n", g_testDuty * 100.0f);
            break;
        default:
            printMenu();
            break;
    }
}
