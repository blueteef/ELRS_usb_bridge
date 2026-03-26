/* ============================================================
 * SBUS -> USB HID Gamepad Bridge
 * ESP32-C3 Super Mini (Tenstar Robot)
 * ============================================================
 * RC receiver SBUS out -> GPIO 3
 * Auto-senses inverted (standard) or TTL (non-inverted) SBUS.
 * Outputs as a USB HID gamepad (composite USB: HID + CDC serial).
 *
 * Wiring:
 *   Receiver SBUS  -> GPIO 3
 *   Receiver GND   -> Drain of 2N7000 (Source -> System GND)
 *   Receiver 5V    -> 5V (unswitched)
 *   2N7000 Gate    -> GPIO 5  (HIGH = RX powered on)
 *   Wake switch    -> GPIO 4  (other leg to GND; active LOW)
 *   WS2812 DIN     -> GPIO 2
 *
 * Axis mapping (USB HID has 6 axes):
 *   ch[0] -> X    ch[1] -> Y    ch[2] -> Z
 *   ch[3] -> Rz   ch[4] -> Rx   ch[5] -> Ry
 *   ch[6] -> Button 9  (high/low threshold)
 *   ch[7] -> Button 10 (high/low threshold)
 * Button mapping:
 *   ch[8-15] -> Buttons 1-8
 *
 * LED (GPIO 2, WS2812):
 *   Amber (slow blink) : scanning for signal
 *   Amber (solid)      : signal locked, USB not yet enumerated
 *   Green              : signal + USB ready, normal operation
 *   Red                : failsafe active
 *   Off                : sleeping
 *   Orange (slow blink): diagnostic / upload mode
 *
 * Button behaviour:
 *   Short press (sleeping)  : wake, RX on, resume scanning
 *   Long press (3s, awake)  : enter diagnostic mode (RX off,
 *                             HID stopped, USB serial active).
 *                             Press RST to return to normal.
 *
 * Power management:
 *   After IDLE_TIMEOUT_MS of no stick/button activity, the MOSFET
 *   cuts power to the RX and the ESP32 enters light sleep. USB
 *   stays enumerated. Press the wake switch to resume.
 * ============================================================ */

#include <Arduino.h>
#include <USB.h>
#include <USBHIDGamepad.h>
#include <Adafruit_NeoPixel.h>
#include "sbus.h"
#include <esp_sleep.h>
#include <driver/gpio.h>

// --- USB HID Gamepad ---
USBHIDGamepad gamepad;

// --- External WS2812 LED ---
#define RGB_PIN  2
Adafruit_NeoPixel rgb(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// --- SBUS input ---
#define RX_PIN 3
bfs::SbusRx sbus_inv(&Serial1, RX_PIN, -1, true);   // standard SBUS (inverted)
bfs::SbusRx sbus_ttl(&Serial1, RX_PIN, -1, false);  // SBUS TTL (non-inverted)
bfs::SbusData sbusData;

// --- Power control ---
#define MOSFET_PIN  5   // Gate of 2N7000; HIGH = RX on
#define WAKE_PIN    4   // Switch to GND; wakes from sleep / triggers diag mode

// --- Timeouts ---
#define IDLE_TIMEOUT_MS     (5UL * 60UL * 1000UL)  // 5 min idle → sleep
#define DIAG_HOLD_MS        3000                    // hold duration for diag mode

// SBUS units from center (992) to count as stick activity.
#define ACTIVITY_THRESHOLD  50

// --- RTC memory: survives software reset, cleared on power cycle ---
RTC_DATA_ATTR static uint8_t bootToDiag = 0;

static unsigned long lastActivity = 0;

// --- State machine ---
enum State { SCAN_INV, SCAN_TTL, ACTIVE_INV, ACTIVE_TTL };
State currentState = SCAN_INV;
unsigned long lastValidSignal = 0;
unsigned long stateTimer = 0;

// Map SBUS value (172-1811) to int8_t axis (-128 to 127).
inline int8_t sbusToAxis(uint16_t val) {
    val = constrain(val, 172, 1811);
    return (int8_t)map((long)val, 172, 1811, -128, 127);
}

void setLED(uint8_t r, uint8_t g, uint8_t b) {
    rgb.setPixelColor(0, rgb.Color(r, g, b));
    rgb.show();
}

// Returns true if any axis or button shows meaningful user input.
bool isChannelActive(const bfs::SbusData &d) {
    for (int i = 0; i < 8; i++) {
        if (abs((int)d.ch[i] - 992) > ACTIVITY_THRESHOLD) return true;
    }
    for (int i = 8; i < 16; i++) {
        if (d.ch[i] > 1000) return true;
    }
    return false;
}

// Diagnostic mode: RX off, HID idle, USB serial active.
// Slow orange blink. Exit by pressing RST.
void runDiagMode() {
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, LOW);
    pinMode(WAKE_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);

    rgb.begin();
    rgb.setBrightness(60);

    Serial.println("=== DIAGNOSTIC / UPLOAD MODE ===");
    Serial.println("HID not started. RX off. USB serial active.");
    Serial.println("Press RST to return to normal operation.");

    while (true) {
        setLED(255, 80, 0);
        delay(600);
        setLED(0, 0, 0);
        delay(400);
    }
}

// Trigger a restart into diagnostic mode via RTC flag.
void enterDiagMode() {
    Serial.println("Entering diagnostic mode...");
    Serial.flush();
    bootToDiag = 1;
    esp_restart();
}

// Cut RX power and enter light sleep. Returns only when the wake
// switch is pressed; execution continues as if nothing happened.
void goToSleep() {
    Serial.println("Idle — cutting RX power, entering light sleep.");
    Serial.flush();

    Serial1.end();
    setLED(0, 0, 0);
    digitalWrite(MOSFET_PIN, LOW);

    gpio_wakeup_enable((gpio_num_t)WAKE_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_light_sleep_start();         // returns on switch press

    // Wait for button release (software debounce).
    while (digitalRead(WAKE_PIN) == LOW) delay(10);
    delay(50);

    Serial.println("Wake switch pressed — RX on, scanning.");
    digitalWrite(MOSFET_PIN, HIGH);
    currentState    = SCAN_INV;
    lastValidSignal = millis();
    lastActivity    = millis();
}

void setup() {
    if (bootToDiag) {
        bootToDiag = 0;
        runDiagMode();   // never returns
    }

    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, HIGH);
    pinMode(WAKE_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);
    Serial.println("SBUS USB HID Bridge starting...");

    rgb.begin();
    rgb.setBrightness(60);
    rgb.clear();
    rgb.show();

    gamepad.begin();
    USB.begin();

    lastActivity = millis();
    stateTimer   = millis();
}

void loop() {
    bool newFrame = false;
    bool failsafe = false;

    // --- Long-press detection (diag mode trigger) ---
    static unsigned long buttonHeldSince = 0;
    static bool buttonTracked = false;
    if (digitalRead(WAKE_PIN) == LOW) {
        if (!buttonTracked) {
            buttonHeldSince = millis();
            buttonTracked   = true;
        } else if (millis() - buttonHeldSince >= DIAG_HOLD_MS) {
            enterDiagMode();
        }
    } else {
        buttonTracked = false;
    }

    switch (currentState) {

        case SCAN_INV:
            sbus_inv.Begin();
            stateTimer = millis();
            while (millis() - stateTimer < 600) {
                if (sbus_inv.Read()) {
                    currentState    = ACTIVE_INV;
                    lastValidSignal = millis();
                    lastActivity    = millis();
                    Serial.println("Locked: SBUS Inverted");
                    break;
                }
                delay(5);
            }
            if (currentState != ACTIVE_INV) {
                Serial1.end();
                currentState = SCAN_TTL;
            }
            break;

        case SCAN_TTL:
            sbus_ttl.Begin();
            stateTimer = millis();
            while (millis() - stateTimer < 600) {
                if (sbus_ttl.Read()) {
                    currentState    = ACTIVE_TTL;
                    lastValidSignal = millis();
                    lastActivity    = millis();
                    Serial.println("Locked: SBUS TTL");
                    break;
                }
                delay(5);
            }
            if (currentState != ACTIVE_TTL) {
                Serial1.end();
                currentState = SCAN_INV;
            }
            break;

        case ACTIVE_INV:
            if (sbus_inv.Read()) {
                sbusData        = sbus_inv.data();
                newFrame        = true;
                lastValidSignal = millis();
                failsafe        = sbusData.failsafe;
                if (!failsafe && isChannelActive(sbusData))
                    lastActivity = millis();
            }
            if (millis() - lastValidSignal > 2000) {
                Serial.println("Signal lost, scanning...");
                Serial1.end();
                currentState = SCAN_INV;
            }
            break;

        case ACTIVE_TTL:
            if (sbus_ttl.Read()) {
                sbusData        = sbus_ttl.data();
                newFrame        = true;
                lastValidSignal = millis();
                failsafe        = sbusData.failsafe;
                if (!failsafe && isChannelActive(sbusData))
                    lastActivity = millis();
            }
            if (millis() - lastValidSignal > 2000) {
                Serial.println("Signal lost, scanning...");
                Serial1.end();
                currentState = SCAN_INV;
            }
            break;
    }

    // --- USB HID output ---
    if (newFrame && !failsafe && gamepad.ready()) {
        hid_gamepad_report_t report = {0};
        report.x  = sbusToAxis(sbusData.ch[0]);
        report.y  = sbusToAxis(sbusData.ch[1]);
        report.z  = sbusToAxis(sbusData.ch[2]);
        report.rz = sbusToAxis(sbusData.ch[3]);
        report.rx = sbusToAxis(sbusData.ch[4]);
        report.ry = sbusToAxis(sbusData.ch[5]);

        // ch[6-7] as buttons 9-10; ch[8-15] as buttons 1-8
        uint32_t buttons = 0;
        for (int b = 0; b < 8; b++) {
            if (sbusData.ch[b + 8] > 1000) buttons |= (1u << b);
        }
        if (sbusData.ch[6] > 1000) buttons |= (1u << 8);
        if (sbusData.ch[7] > 1000) buttons |= (1u << 9);
        report.buttons = buttons;

        gamepad.sendReport(&report);
    }

    // --- LED status ---
    // All LED logic in one place; called every loop iteration.
    bool scanning = (currentState == SCAN_INV || currentState == SCAN_TTL);
    if (scanning) {
        // Amber slow blink: looking for signal
        if ((millis() / 500) % 2)
            setLED(200, 100, 0);
        else
            setLED(0, 0, 0);
    } else {
        // Active state
        if (failsafe)
            setLED(255, 0, 0);       // red:   failsafe
        else if (!gamepad.ready())
            setLED(200, 100, 0);     // amber: signal ok, USB not ready
        else
            setLED(0, 200, 0);       // green: all good
    }

    // --- Idle / sleep check ---
    if (millis() - lastActivity > IDLE_TIMEOUT_MS) {
        goToSleep();
    }
}
