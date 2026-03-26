/* ============================================================
 * SBUS -> BLE HID Gamepad Bridge
 * ESP32-C3 Super Mini (Tenstar Robot)
 * ============================================================
 * RC receiver SBUS out -> GPIO 3
 * Auto-senses inverted (standard) or TTL (non-inverted) SBUS.
 * Outputs 8 axes + 8 buttons as a BLE HID gamepad.
 *
 * Wiring:
 *   Receiver SBUS  -> GPIO 3
 *   Receiver GND   -> Drain of 2N7000 (Source -> System GND)
 *   Receiver 5V    -> 5V (unswitched)
 *   2N7000 Gate    -> GPIO 5  (HIGH = RX powered on)
 *   Wake switch    -> GPIO 4  (other leg to GND; active LOW)
 *   WS2812 DIN     -> GPIO 2
 *
 * SBUS ch[0-7]  -> axes  (X, Y, Z, Rz, Rx, Ry, Slider1, Slider2)
 * SBUS ch[8-15] -> buttons 1-8 (high = pressed)
 *
 * LED (GPIO 2, WS2812):
 *   Amber (slow blink) : scanning for signal
 *   Amber (solid)      : signal locked, BLE not connected
 *   Green              : signal + BLE connected, normal operation
 *   Red                : failsafe active
 *   Off                : sleeping
 *   Orange (slow blink): diagnostic mode
 *
 * Button behaviour:
 *   Short press (sleeping)  : wake, RX on, resume scanning
 *   Long press (3s, awake)  : enter diagnostic mode (RX off,
 *                             BLE stopped, USB serial active).
 *                             Press RST to return to normal.
 *
 * Power management:
 *   After IDLE_TIMEOUT_MS of no stick/button activity, the MOSFET
 *   cuts power to the RX and the ESP32 enters light sleep. BLE
 *   stays alive. Press the wake switch to resume.
 * ============================================================ */

#include <Arduino.h>
#include <HWCDC.h>
#include <Adafruit_NeoPixel.h>
#include "sbus.h"
#include <BleGamepad.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

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
#define IDLE_TIMEOUT_MS     (5UL * 60UL * 1000UL)  // 5 min idle -> sleep
#define DIAG_HOLD_MS        3000                    // hold duration for diag mode

// SBUS units from center (992) to count as stick activity.
#define ACTIVITY_THRESHOLD  50

// --- RTC memory: survives software reset, cleared on power cycle ---
RTC_DATA_ATTR static uint8_t bootToDiag = 0;

static unsigned long lastActivity = 0;

// --- BLE Gamepad ---
BleGamepad bleGamepad("RC Joystick", "ESP32-C3", 100);
BleGamepadConfiguration bleGamepadConfig;

// --- State machine ---
enum State { SCAN_INV, SCAN_TTL, ACTIVE_INV, ACTIVE_TTL };
State currentState = SCAN_INV;
unsigned long lastValidSignal = 0;
unsigned long stateTimer = 0;

// Map SBUS value (172-1811) to signed 16-bit axis (-32768 to 32767).
inline int16_t sbusToAxis(uint16_t val) {
    val = constrain(val, 172, 1811);
    return (int16_t)map((long)val, 172, 1811, -32768, 32767);
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

// Diagnostic mode: RX off, BLE stopped, USB serial active.
// Slow orange blink. Exit by pressing RST.
void runDiagMode() {
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, LOW);
    pinMode(WAKE_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);

    rgb.begin();
    rgb.setBrightness(217);

    Serial.println("=== DIAGNOSTIC MODE ===");
    Serial.println("BLE not started. RX off. USB serial active.");
    Serial.println("Press RST to return to normal operation.");

    while (true) {
        setLED(255, 80, 0);
        delay(600);
        setLED(0, 0, 0);
        delay(400);
    }
}

// Trigger a restart into diagnostic mode via RTC flag.
// Cut RX power immediately before restarting so it never lingers.
void enterDiagMode() {
    Serial1.end();
    digitalWrite(MOSFET_PIN, LOW);
    bootToDiag = 1;
    esp_restart();
}

// Cut RX power and enter light sleep. Returns only when the wake
// switch is pressed; execution continues as if nothing happened.
void goToSleep() {
    Serial.println("Idle - cutting RX power, entering light sleep.");
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

    Serial.println("Wake switch pressed - RX on, scanning.");
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
    Serial.println("SBUS BLE Bridge starting...");

    rgb.begin();
    rgb.setBrightness(217);
    rgb.clear();
    rgb.show();

    bleGamepadConfig.setAutoReport(false);
    bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
    bleGamepadConfig.setButtonCount(8);
    bleGamepadConfig.setWhichAxes(true, true, true, true, true, true, true, true);
    bleGamepadConfig.setAxesMax(32767);
    bleGamepadConfig.setAxesMin(-32768);
    bleGamepad.begin(&bleGamepadConfig);

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
                if (digitalRead(WAKE_PIN) == LOW) {
                    if (!buttonTracked) { buttonHeldSince = millis(); buttonTracked = true; }
                    else if (millis() - buttonHeldSince >= DIAG_HOLD_MS) enterDiagMode();
                } else { buttonTracked = false; }
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
                if (digitalRead(WAKE_PIN) == LOW) {
                    if (!buttonTracked) { buttonHeldSince = millis(); buttonTracked = true; }
                    else if (millis() - buttonHeldSince >= DIAG_HOLD_MS) enterDiagMode();
                } else { buttonTracked = false; }
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

    // --- BLE output ---
    if (newFrame && !failsafe && bleGamepad.isConnected()) {
        bleGamepad.setAxes(
            sbusToAxis(sbusData.ch[0]),
            sbusToAxis(sbusData.ch[1]),
            sbusToAxis(sbusData.ch[2]),
            sbusToAxis(sbusData.ch[3]),
            sbusToAxis(sbusData.ch[4]),
            sbusToAxis(sbusData.ch[5]),
            sbusToAxis(sbusData.ch[6]),
            sbusToAxis(sbusData.ch[7])
        );
        for (int b = 0; b < 8; b++) {
            if (sbusData.ch[b + 8] > 1000)
                bleGamepad.press(b + 1);
            else
                bleGamepad.release(b + 1);
        }
        bleGamepad.sendReport();
    }

    // --- LED status ---
    bool scanning = (currentState == SCAN_INV || currentState == SCAN_TTL);
    if (scanning) {
        if ((millis() / 500) % 2)
            setLED(200, 100, 0);   // amber blink: scanning
        else
            setLED(0, 0, 0);
    } else {
        if (failsafe)
            setLED(255, 0, 0);             // red:   failsafe
        else if (!bleGamepad.isConnected())
            setLED(200, 100, 0);           // amber: signal ok, BLE not connected
        else
            setLED(0, 200, 0);             // green: all good
    }

    // --- Idle / sleep check ---
    if (millis() - lastActivity > IDLE_TIMEOUT_MS) {
        goToSleep();
    }
}
