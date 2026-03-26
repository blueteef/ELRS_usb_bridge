# SBUS → USB HID Gamepad Bridge
**Hardware:** ESP32-C3 Super Mini (Tenstar Robot)

Converts RC receiver SBUS output to a USB HID gamepad recognized natively by Windows. No drivers or pairing required. Plug in and it shows up as **"ESP32 Gamepad"** in Device Manager.

---

## Wiring

| Signal | ESP32-C3 Pin |
|--------|-------------|
| RC Receiver SBUS out | IO3 |
| 2N7000 MOSFET gate | IO5 |
| Wake / diag switch | IO4 (other leg to GND) |
| WS2812B data in | IO2 |

**Power switching (N-channel low-side):**
- RC Receiver VCC → 5V (always on)
- RC Receiver GND → 2N7000 Drain
- 2N7000 Source → System GND
- 2N7000 Gate → IO5 + 10k pulldown to GND

The 10k pulldown ensures the receiver stays off during sleep even if the GPIO floats.

---

## Channel Mapping

SBUS supports 16 channels. They are mapped as follows:

### Axes (6 total)
| SBUS Channel | HID Axis | Typical RC function |
|---|---|---|
| ch1 | X | Aileron / Roll |
| ch2 | Y | Elevator / Pitch |
| ch3 | Z | Throttle |
| ch4 | Rz | Rudder / Yaw |
| ch5 | Rx | Aux 1 |
| ch6 | Ry | Aux 2 |

### Buttons
| SBUS Channel | HID Button | Note |
|---|---|---|
| ch7 | Button 9 | Digital (high/low threshold) |
| ch8 | Button 10 | Digital (high/low threshold) |
| ch9–ch16 | Buttons 1–8 | Digital (high/low threshold) |

Axis resolution is 8-bit (–128 to 127). Both standard inverted SBUS and TTL (non-inverted) SBUS are auto-detected on startup.

---

## LED Status

| Colour | Pattern | Meaning |
|--------|---------|---------|
| Amber | Slow blink | Scanning for SBUS signal |
| Amber | Solid | Signal locked — waiting for USB host |
| Green | Solid | Normal operation — signal and USB ready |
| Red | Solid | Failsafe active (TX off or signal lost) |
| Orange | Slow blink | Diagnostic / upload mode |
| Off | — | Sleeping (RX powered off) |

---

## Button

The single button (IO4) has two functions depending on device state:

| State | Action | Result |
|-------|--------|--------|
| Sleeping | Short press | RX powers on, resumes scanning |
| Awake | Hold 3 seconds | Enter diagnostic mode |

---

## Power Management

After **5 minutes** of no stick or button activity, the device enters light sleep:

1. RX is powered off via MOSFET (reduces heat, saves power)
2. ESP32 enters light sleep (~1mA)
3. USB remains enumerated — the PC does not see a disconnect
4. Press the button to wake: RX powers back on and scanning resumes within ~1 second

Activity is defined as any axis channel deviating more than 50 SBUS units from center (992), or any button channel going high.

---

## Diagnostic Mode

Triggered by holding the button for 3 seconds while the device is awake.

- RX powered off
- USB HID gamepad stopped (no ghost inputs to PC)
- USB serial remains active at **115200 baud**
- LED blinks orange

**Use this mode for:**
- Reading serial debug output (SBUS lock messages, signal lost events)
- Firmware uploads (still requires holding hardware BOOT button + RST to enter bootloader)

**To exit:** Press the **RST** button on the ESP32. The device reboots into normal operation. Power cycling also exits diagnostic mode.

---

## Firmware Upload

1. Hold the **BOOT** button (IO9, onboard)
2. Press and release **RST**
3. Release **BOOT**
4. Upload from PlatformIO

If upload fails at 460800 baud, add `upload_speed = 115200` to `platformio.ini`.

---

## Verifying Gamepad Input (Windows)

1. Press **Win + R**, type `joy.cpl`, press Enter
2. Select **ESP32 Gamepad** → click **Properties**
3. Move sticks — axes should respond live
4. Activate switches — buttons should light up

---

## Configurable Constants (`src/main.cpp`)

| Constant | Default | Description |
|----------|---------|-------------|
| `IDLE_TIMEOUT_MS` | 5 minutes | Inactivity time before sleep |
| `DIAG_HOLD_MS` | 3000 ms | Button hold time to enter diag mode |
| `ACTIVITY_THRESHOLD` | 50 | SBUS units from center to count as active |
| `RGB_PIN` | 2 | WS2812B data pin |
| `MOSFET_PIN` | 5 | 2N7000 gate pin |
| `WAKE_PIN` | 4 | Wake / diag button pin |
