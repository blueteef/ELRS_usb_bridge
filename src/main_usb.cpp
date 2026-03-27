/* ============================================================
 * SBUS -> USB HID Gamepad Bridge
 * ESP32-S3 Super Mini (Tenstar Robot)
 * ============================================================
 * Wiring: identical to C3 BLE version (same pinout).
 *   Receiver SBUS  -> GPIO 3
 *   Receiver GND   -> Drain of 2N7000 (Source -> System GND)
 *   Receiver 5V    -> 5V (always on)
 *   2N7000 Gate    -> GPIO 5  (HIGH = RX on)
 *   Button         -> GPIO 4  (other leg to GND, active LOW)
 *   WS2812B DIN    -> GPIO 2
 *
 * SBUS ch[0-7]  -> HID axes  (X,Y,Z,Rz,Rx,Ry,Slider,Dial)
 * SBUS ch[8-15] -> HID buttons 1-8
 *
 * Shows up on Windows as "RC Joystick" — no drivers or pairing.
 * Uses the Arduino ESP32 built-in TinyUSB stack (no extra library).
 * ============================================================ */

#include <Arduino.h>
#include <USB.h>
#include <USBHID.h>
#include <Adafruit_NeoPixel.h>
#include "sbus.h"
#include <esp_sleep.h>
#include <driver/gpio.h>

// ---- Pins ----
#define PIN_SBUS    3
#define PIN_MOSFET  5
#define PIN_BUTTON  4
#define PIN_LED     2

// ---- Timing ----
#define IDLE_MS      (5UL * 60UL * 1000UL)
#define HOLD_CUT_MS  3000
#define SCAN_WIN_MS  600
#define LOST_MS      2000
#define ACT_THRESH   50

// ---- USB HID descriptor: 8 buttons + 8 signed 16-bit axes ----
static const uint8_t hid_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    // 8 buttons (1 bit each, packed into 1 byte)
    0x05, 0x09,        // Usage Page (Button)
    0x19, 0x01,        // Usage Min (1)
    0x29, 0x08,        // Usage Max (8)
    0x15, 0x00,        // Logical Min (0)
    0x25, 0x01,        // Logical Max (1)
    0x75, 0x01,        // Report Size (1)
    0x95, 0x08,        // Report Count (8)
    0x81, 0x02,        // Input (Data, Variable, Absolute)
    // 8 axes (16-bit signed)
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x30,        // X
    0x09, 0x31,        // Y
    0x09, 0x32,        // Z
    0x09, 0x35,        // Rz
    0x09, 0x33,        // Rx
    0x09, 0x34,        // Ry
    0x09, 0x36,        // Slider
    0x09, 0x37,        // Dial
    0x16, 0x00, 0x80,  // Logical Min (-32768)
    0x26, 0xFF, 0x7F,  // Logical Max (32767)
    0x75, 0x10,        // Report Size (16)
    0x95, 0x08,        // Report Count (8)
    0x81, 0x02,        // Input (Data, Variable, Absolute)
    0xC0               // End Collection
};

typedef struct __attribute__((packed)) {
    uint8_t  buttons;
    int16_t  axes[8];
} gamepad_report_t;

// ---- Custom HID device using Arduino ESP32 built-in stack ----
USBHID HID;

class GamepadHID : public USBHIDDevice {
public:
    uint16_t _onGetDescriptor(uint8_t *buf) override {
        memcpy(buf, hid_desc, sizeof(hid_desc));
        return sizeof(hid_desc);
    }
    bool send(gamepad_report_t *r) {
        return HID.SendReport(0, r, sizeof(*r), 10);
    }
    bool ready() { return HID.ready(); }
} gamepad;

// ---- Hardware ----
Adafruit_NeoPixel led(1, PIN_LED, NEO_GRB + NEO_KHZ800);
bfs::SbusRx       sbus_inv(&Serial1, PIN_SBUS, -1, true);
bfs::SbusRx       sbus_ttl(&Serial1, PIN_SBUS, -1, false);
bfs::SbusData     sbusData;

// ---- State ----
enum RxState   { RX_ON, RX_OFF };
enum SbusState { SCAN_INV, SCAN_TTL, ACTIVE_INV, ACTIVE_TTL };

RxState   rxState   = RX_ON;
SbusState sbusState = SCAN_INV;

unsigned long lastActivity = 0;
unsigned long lastValidSig = 0;
unsigned long wakeGraceEnd = 0;

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

// ---- RX power ----
void cutRX() {
    Serial1.end();
    digitalWrite(PIN_MOSFET, LOW);
    rxState   = RX_OFF;
    sbusState = SCAN_INV;
    ledPhase  = LED_NORMAL;
}

void restoreRX() {
    digitalWrite(PIN_MOSFET, HIGH);
    rxState      = RX_ON;
    sbusState    = SCAN_INV;
    lastActivity = millis();
    lastValidSig = millis();
    ledPhase     = LED_NORMAL;   // scanning sweep starts immediately
}

// ---- Sleep ----
void enterSleep() {
    if (rxState == RX_ON) cutRX();
    setLED(0, 0, 0);

    while (digitalRead(PIN_BUTTON) == LOW) delay(10);
    delay(50);

    gpio_sleep_set_pull_mode((gpio_num_t)PIN_BUTTON, GPIO_PULLUP_ONLY);
    gpio_wakeup_enable((gpio_num_t)PIN_BUTTON, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_light_sleep_start();

    while (digitalRead(PIN_BUTTON) == LOW) delay(10);
    delay(50);

    wakeGraceEnd = millis() + 1500;
    restoreRX();
}

// ---- Setup ----
void setup() {
    pinMode(PIN_MOSFET, OUTPUT);
    digitalWrite(PIN_MOSFET, HIGH);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    // Register HID device before USB stack starts
    HID.addDevice(&gamepad, sizeof(hid_desc));
    USB.productName("RC Joystick");
    USB.manufacturerName("ESP32-S3");
    HID.begin();
    USB.begin();

    Serial.begin(115200);

    led.begin();
    led.setBrightness(217);
    led.clear();
    led.show();

    lastActivity  = millis();
    lastValidSig  = millis();
    ledPhaseStart = millis();
}

// ---- Main loop ----
void loop() {

    // ============================================================
    // BUTTON
    // ============================================================
    static unsigned long btnHeldSince = 0;
    static bool          btnDown      = false;
    static bool          cutThisPress = false;

    bool pressed = (digitalRead(PIN_BUTTON) == LOW);

    if (pressed && !btnDown) {
        btnHeldSince  = millis();
        btnDown       = true;
        cutThisPress  = false;
    } else if (pressed && btnDown) {
        if (!cutThisPress &&
            rxState == RX_ON &&
            millis() > wakeGraceEnd &&
            millis() - btnHeldSince >= HOLD_CUT_MS) {
            cutRX();
            cutThisPress = true;
        }
    } else if (!pressed && btnDown) {
        if (rxState == RX_OFF && !cutThisPress) {
            restoreRX();
        }
        btnDown      = false;
        cutThisPress = false;
    }

    // ============================================================
    // SBUS STATE MACHINE
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
            if (millis() - lastValidSig > LOST_MS) { Serial1.end(); sbusState = SCAN_INV; }
            break;

        case ACTIVE_TTL:
            if (sbus_ttl.Read()) {
                sbusData     = sbus_ttl.data();
                newFrame     = true;
                lastValidSig = millis();
                failsafe     = sbusData.failsafe;
                if (!failsafe && isActive(sbusData)) lastActivity = millis();
            }
            if (millis() - lastValidSig > LOST_MS) { Serial1.end(); sbusState = SCAN_INV; }
            break;
        }
    }

    // ============================================================
    // USB HID OUTPUT
    // ============================================================
    if (newFrame && !failsafe && gamepad.ready()) {
        gamepad_report_t report = {};
        for (int b = 0; b < 8; b++) {
            if (sbusData.ch[b + 8] > 1000) report.buttons |= (1 << b);
        }
        for (int a = 0; a < 8; a++) {
            report.axes[a] = sbusToAxis(sbusData.ch[a]);
        }
        gamepad.send(&report);
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
