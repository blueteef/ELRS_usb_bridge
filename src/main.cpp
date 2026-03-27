/* ============================================================
 * SBUS -> BLE HID Gamepad Bridge
 * ESP32-C3 Super Mini (Tenstar Robot)
 * ============================================================
 * Wiring:
 *   Receiver SBUS  -> GPIO 3
 *   Receiver GND   -> Drain of 2N7000 (Source -> System GND)
 *   Receiver 5V    -> 5V (always on)
 *   2N7000 Gate    -> GPIO 5  (HIGH = RX on)
 *   Button         -> GPIO 4  (other leg to GND, active LOW)
 *   WS2812B DIN    -> GPIO 2
 *
 * SBUS ch[0-7]  -> HID axes  (X,Y,Z,Rz,Rx,Ry,Slider1,Slider2)
 * SBUS ch[8-15] -> HID buttons 1-8
 *
 * Button behaviour:
 *   RX on  + hold 3s -> cut RX, stay running (breathing blue LED)
 *   RX off + press   -> restore RX
 *   Sleeping + press -> wake, restore RX
 *
 * Auto-sleep: 5 min idle -> cut RX -> light sleep
 *
 * LED states:
 *   Breathing blue  : RX off (manual cut or pre-sleep)
 *   Amber blink     : RX on, scanning for SBUS
 *   Amber solid     : RX on, SBUS locked, BLE not connected
 *   Green solid     : RX on, SBUS locked, BLE connected
 *   Red solid       : RX on, failsafe active
 *   Off             : sleeping
 * ============================================================ */

#include <Arduino.h>
#include <HWCDC.h>
#include <Adafruit_NeoPixel.h>
#include "sbus.h"
#include <BleGamepad.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

// ---- Pins ----
#define PIN_SBUS    3
#define PIN_MOSFET  5
#define PIN_BUTTON  4
#define PIN_LED     2

// ---- Timing ----
#define IDLE_MS      (5UL * 60UL * 1000UL)  // idle -> sleep
#define HOLD_CUT_MS  3000                    // hold to cut RX
#define SCAN_WIN_MS  600                     // SBUS scan window
#define LOST_MS      2000                    // signal-lost timeout
#define ACT_THRESH   50                      // SBUS deviation from center (992)

// ---- Hardware ----
Adafruit_NeoPixel led(1, PIN_LED, NEO_GRB + NEO_KHZ800);
bfs::SbusRx       sbus_inv(&Serial1, PIN_SBUS, -1, true);
bfs::SbusRx       sbus_ttl(&Serial1, PIN_SBUS, -1, false);
bfs::SbusData     sbusData;
BleGamepad        ble("RC Joystick", "ESP32-C3", 100);
BleGamepadConfiguration bleCfg;

// ---- State ----
enum RxState   { RX_ON, RX_OFF };
enum SbusState { SCAN_INV, SCAN_TTL, ACTIVE_INV, ACTIVE_TTL };

RxState   rxState   = RX_ON;
SbusState sbusState = SCAN_INV;

unsigned long lastActivity    = 0;
unsigned long lastValidSig    = 0;
unsigned long wakeGraceEnd    = 0;   // long-press blocked after wake

// ---- LED phase ----
enum LedPhase { LED_BOOT, LED_RESTORE, LED_NORMAL };
LedPhase      ledPhase      = LED_BOOT;
unsigned long ledPhaseStart  = 0;

// ---- Helpers ----
void setLED(uint8_t r, uint8_t g, uint8_t b) {
    led.setPixelColor(0, led.Color(r, g, b));
    led.show();
}

inline int16_t sbusToAxis(uint16_t v) {
    return (int16_t)map((long)constrain(v, 172, 1811), 172, 1811, -32768, 32767);
}

bool isActive(const bfs::SbusData &d) {
    for (int i = 0; i < 8;  i++) if (abs((int)d.ch[i] - 992) > ACT_THRESH) return true;
    for (int i = 8; i < 16; i++) if (d.ch[i] > 1000) return true;
    return false;
}

// ---- RX power control ----
void cutRX() {
    Serial1.end();
    digitalWrite(PIN_MOSFET, LOW);
    rxState   = RX_OFF;
    sbusState = SCAN_INV;
    ledPhase  = LED_NORMAL;
}

void restoreRX() {
    digitalWrite(PIN_MOSFET, HIGH);
    rxState       = RX_ON;
    sbusState     = SCAN_INV;
    lastActivity  = millis();
    lastValidSig  = millis();
    ledPhase      = LED_NORMAL;   // scanning sweep starts immediately
}

// ---- Sleep ----
void enterSleep() {
    if (rxState == RX_ON) cutRX();
    setLED(0, 0, 0);

    // Wait for button release so LOW_LEVEL wakeup doesn't fire instantly.
    while (digitalRead(PIN_BUTTON) == LOW) delay(10);
    delay(50);

    // Hold pullup active during sleep (Arduino INPUT_PULLUP doesn't
    // guarantee this on ESP32-C3 in sleep domain).
    gpio_sleep_set_pull_mode((gpio_num_t)PIN_BUTTON, GPIO_PULLUP_ONLY);
    gpio_wakeup_enable((gpio_num_t)PIN_BUTTON, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_light_sleep_start();             // returns on button press

    // Debounce after wake.
    while (digitalRead(PIN_BUTTON) == LOW) delay(10);
    delay(50);

    wakeGraceEnd = millis() + 1500;      // suppress long-press for 1.5s
    restoreRX();
}

// ---- Setup ----
void setup() {
    pinMode(PIN_MOSFET, OUTPUT);
    digitalWrite(PIN_MOSFET, HIGH);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    Serial.begin(115200);

    led.begin();
    led.setBrightness(217);
    led.clear();
    led.show();

    bleCfg.setAutoReport(false);
    bleCfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);
    bleCfg.setButtonCount(8);
    bleCfg.setWhichAxes(true, true, true, true, true, true, true, true);
    bleCfg.setAxesMax(32767);
    bleCfg.setAxesMin(-32768);
    ble.begin(&bleCfg);

    lastActivity  = millis();
    lastValidSig  = millis();
    ledPhaseStart = millis();
}

// ---- Main loop ----
void loop() {

    // ============================================================
    // BUTTON
    // RX on  + hold HOLD_CUT_MS -> cutRX
    // RX off + short press       -> restoreRX
    // A "cut press" is flagged so its release doesn't immediately
    // restore power.
    // ============================================================
    static unsigned long btnHeldSince = 0;
    static bool          btnDown      = false;
    static bool          cutThisPress = false;

    bool pressed = (digitalRead(PIN_BUTTON) == LOW);

    if (pressed && !btnDown) {
        // Falling edge
        btnHeldSince  = millis();
        btnDown       = true;
        cutThisPress  = false;
    } else if (pressed && btnDown) {
        // Held: check for long-press cut (guard with !cutThisPress so it
        // only fires once — btnDown stays true so we don't re-detect a
        // falling edge while the button is still held)
        if (!cutThisPress &&
            rxState == RX_ON &&
            millis() > wakeGraceEnd &&
            millis() - btnHeldSince >= HOLD_CUT_MS) {
            cutRX();
            cutThisPress = true;
        }
    } else if (!pressed && btnDown) {
        // Rising edge (release)
        if (rxState == RX_OFF && !cutThisPress) {
            restoreRX();
        }
        btnDown      = false;
        cutThisPress = false;
    }

    // ============================================================
    // SBUS STATE MACHINE  (only runs while RX is on)
    // ============================================================
    bool newFrame = false;
    bool failsafe = false;

    if (rxState == RX_ON) {
        switch (sbusState) {

        case SCAN_INV: {
            sbus_inv.Begin();
            unsigned long t0 = millis();
            while (millis() - t0 < SCAN_WIN_MS) {
                if (sbus_inv.Read()) {
                    sbusState     = ACTIVE_INV;
                    ledPhase      = LED_RESTORE;
                    ledPhaseStart = millis();
                    lastValidSig  = lastActivity = millis();
                    break;
                }
                delay(5);
            }
            if (sbusState != ACTIVE_INV) { Serial1.end(); sbusState = SCAN_TTL; }
            break;
        }

        case SCAN_TTL: {
            sbus_ttl.Begin();
            unsigned long t0 = millis();
            while (millis() - t0 < SCAN_WIN_MS) {
                if (sbus_ttl.Read()) {
                    sbusState     = ACTIVE_TTL;
                    ledPhase      = LED_RESTORE;
                    ledPhaseStart = millis();
                    lastValidSig  = lastActivity = millis();
                    break;
                }
                delay(5);
            }
            if (sbusState != ACTIVE_TTL) { Serial1.end(); sbusState = SCAN_INV; }
            break;
        }

        case ACTIVE_INV:
            if (sbus_inv.Read()) {
                sbusData     = sbus_inv.data();
                newFrame     = true;
                lastValidSig = millis();
                failsafe     = sbusData.failsafe;
                if (!failsafe && isActive(sbusData)) lastActivity = millis();
            }
            if (millis() - lastValidSig > LOST_MS) {
                Serial1.end(); sbusState = SCAN_INV;
            }
            break;

        case ACTIVE_TTL:
            if (sbus_ttl.Read()) {
                sbusData     = sbus_ttl.data();
                newFrame     = true;
                lastValidSig = millis();
                failsafe     = sbusData.failsafe;
                if (!failsafe && isActive(sbusData)) lastActivity = millis();
            }
            if (millis() - lastValidSig > LOST_MS) {
                Serial1.end(); sbusState = SCAN_INV;
            }
            break;
        }
    }

    // ============================================================
    // BLE OUTPUT
    // ============================================================
    if (newFrame && !failsafe && ble.isConnected()) {
        ble.setAxes(
            sbusToAxis(sbusData.ch[0]), sbusToAxis(sbusData.ch[1]),
            sbusToAxis(sbusData.ch[2]), sbusToAxis(sbusData.ch[3]),
            sbusToAxis(sbusData.ch[4]), sbusToAxis(sbusData.ch[5]),
            sbusToAxis(sbusData.ch[6]), sbusToAxis(sbusData.ch[7])
        );
        for (int b = 0; b < 8; b++) {
            if (sbusData.ch[b + 8] > 1000) ble.press(b + 1);
            else                            ble.release(b + 1);
        }
        ble.sendReport();
    }

    // ============================================================
    // LED
    // ============================================================
    if (ledPhase == LED_BOOT) {
        // Fast RGB scanner (60ms/colour) -> white flash -> normal
        unsigned long t = millis() - ledPhaseStart;
        if (t >= 3000) {
            ledPhase = LED_NORMAL;
        } else if (t >= 2700) {
            setLED(200, 200, 200);
        } else {
            uint8_t c = (t / 60) % 3;
            if      (c == 0) setLED(220, 0,   0);
            else if (c == 1) setLED(0,   220, 0);
            else             setLED(0,   0,   220);
        }
    } else if (ledPhase == LED_RESTORE) {
        // White glitch strobe -> fast RGB scan -> white flash -> normal
        unsigned long t = millis() - ledPhaseStart;
        if (t < 400) {
            bool on = (millis() / 40) % 2;
            setLED(on ? 220 : 0, on ? 220 : 0, on ? 220 : 0);
        } else if (t < 2000) {
            uint8_t c = ((t - 400) / 60) % 3;
            if      (c == 0) setLED(220, 0,   0);
            else if (c == 1) setLED(0,   220, 0);
            else             setLED(0,   0,   220);
        } else if (t < 2200) {
            setLED(200, 200, 200);
        } else {
            ledPhase = LED_NORMAL;
        }
    } else {
        // LED_NORMAL: derived from state
        if (btnDown && rxState == RX_ON) {
            // Building urgency: intensity and strobe rate both ramp
            unsigned long held   = millis() - btnHeldSince;
            uint8_t intensity    = (uint8_t)min((unsigned long)255, held * 255UL / HOLD_CUT_MS);
            unsigned long period = max(60UL, 400UL - (340UL * held / HOLD_CUT_MS));
            bool on = (millis() % period) < (period / 2);
            setLED(on ? intensity : 0, 0, 0);
        } else if (rxState == RX_OFF) {
            // Breathing blue: 8s cycle, quadratic ease-in/out, 5%->75%
            const uint8_t B_MIN = 13, B_MAX = 220, B_RNG = B_MAX - B_MIN;
            unsigned long t = millis() % 8000;
            uint8_t blue;
            if (t < 3500) {
                uint32_t p = (uint32_t)t * 1000 / 3500;
                blue = B_MIN + (uint8_t)((uint32_t)p * p * B_RNG / 1000000UL);
            } else if (t < 7000) {
                uint32_t p = (uint32_t)(7000 - t) * 1000 / 3500;
                blue = B_MIN + (uint8_t)((uint32_t)p * p * B_RNG / 1000000UL);
            } else {
                blue = B_MIN;
            }
            setLED(0, 0, blue);
            delay(20);
        } else {
            // RX on
            bool scanning = (sbusState == SCAN_INV || sbusState == SCAN_TTL);
            if (scanning) {
                // Radar sweep: fast ramp in, slow fade out, 2s cycle
                unsigned long t = millis() % 2000;
                uint8_t g;
                if      (t < 300)  g = (uint8_t)(t * 220 / 300);
                else if (t < 1000) g = (uint8_t)((1000 - t) * 220 / 700);
                else               g = 0;
                setLED(0, g, 0);
            } else if (failsafe) {
                setLED(255, 0, 0);
            } else {
                // Slow green throb: 150-220 brightness, 2s cycle
                unsigned long t = millis() % 2000;
                uint32_t p = (t < 1000) ? t : (2000 - t);
                uint8_t g = 150 + (uint8_t)(p * 70 / 1000);
                setLED(0, g, 0);
            }
        }
    }

    // ============================================================
    // IDLE / SLEEP
    // ============================================================
    if (millis() - lastActivity > IDLE_MS) {
        enterSleep();
    }
}
