# WS2812B Single-Pixel Driver — No Library

**Platform:** ESP32 (Arduino core) | **Method:** Hardware RMT peripheral  
**Author:** [Shriyanshi Khandal] | **Roll No.:** [BTECH/25119/23]

---

## 1. Working Code

```cpp
/**
 * WS2812B Single-Pixel Driver — No external libraries
 * Platform : ESP32 (Arduino core)
 * Method   : ESP32 RMT (Remote Control Transceiver) peripheral
 */

#include <Arduino.h>
#include "driver/rmt.h"

#define RMT_TX_CHANNEL  RMT_CHANNEL_0
#define RMT_CLK_DIV     4          // 80 MHz / 4 = 20 MHz → 50 ns per tick

// WS2812B timing (from datasheet, in 50 ns ticks)
#define T0H_TICKS   8    // 400 ns
#define T0L_TICKS  17    // 850 ns
#define T1H_TICKS  16    // 800 ns
#define T1L_TICKS   9    // 450 ns
#define RESET_TICKS 1000 // 50 µs

static const rmt_item32_t BIT_0 = {
    .duration0 = T0H_TICKS, .level0 = 1,
    .duration1 = T0L_TICKS, .level1 = 0
};
static const rmt_item32_t BIT_1 = {
    .duration0 = T1H_TICKS, .level0 = 1,
    .duration1 = T1L_TICKS, .level1 = 0
};
static const rmt_item32_t RESET_ITEM = {
    .duration0 = RESET_TICKS, .level0 = 0,
    .duration1 = RESET_TICKS, .level1 = 0
};

static bool rmt_initialised = false;

static void rmt_init(uint8_t gpio_pin) {
    rmt_config_t cfg = {};
    cfg.rmt_mode            = RMT_MODE_TX;
    cfg.channel             = RMT_TX_CHANNEL;
    cfg.gpio_num            = (gpio_num_t)gpio_pin;
    cfg.clk_div             = RMT_CLK_DIV;
    cfg.mem_block_num       = 1;
    cfg.tx_config.loop_en             = false;
    cfg.tx_config.carrier_en          = false;
    cfg.tx_config.idle_output_en      = true;
    cfg.tx_config.idle_level          = RMT_IDLE_LEVEL_LOW;
    cfg.tx_config.carrier_duty_percent = 50;

    rmt_config(&cfg);
    rmt_driver_install(RMT_TX_CHANNEL, 0, 0);
    rmt_initialised = true;
}

void setPixelColor(uint8_t pin, uint8_t r, uint8_t g, uint8_t b) {
    if (!rmt_initialised) {
        rmt_init(pin);
    }

    rmt_item32_t items[25];
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;

    for (int i = 0; i < 24; i++) {
        items[i] = (grb & (1UL << (23 - i))) ? BIT_1 : BIT_0;
    }
    items[24] = RESET_ITEM;

    rmt_write_items(RMT_TX_CHANNEL, items, 25, true);
}

// Demo: cycle through colours
#define DATA_PIN 4

void setup() {
    Serial.begin(115200);
}

void loop() {
    setPixelColor(DATA_PIN, 255,   0,   0);  delay(500);  // Red
    setPixelColor(DATA_PIN,   0, 255,   0);  delay(500);  // Green
    setPixelColor(DATA_PIN,   0,   0, 255);  delay(500);  // Blue
    setPixelColor(DATA_PIN,   0,   0,   0);  delay(500);  // Off
}
```

**Hardware connection:**
```
ESP32 GPIO 4  ──────►  WS2812B DIN
ESP32 3.3 V   ──────►  WS2812B VCC  (or 5 V from USB if LED is 5 V-powered)
ESP32 GND     ──────►  WS2812B GND
```
> If the LED is powered at 5 V, add a 300–500 Ω series resistor on the data line to protect the ESP32 GPIO.

---

## 2. How It Works

### The WS2812B Protocol

The WS2812B is a single-wire, self-clocked LED. All information — colour and timing — is encoded in the **duration** of high and low pulses on a single data line.

#### Bit encoding

| Bit | High pulse (T_H) | Low pulse (T_L) |
|-----|-----------------|-----------------|
| `0` | 400 ns ± 150 ns | 850 ns ± 150 ns |
| `1` | 800 ns ± 150 ns | 450 ns ± 150 ns |

A bit is recognised as `1` when the high pulse is long; `0` when it is short.

#### Packet structure

Each pixel requires exactly **24 bits** sent **MSB first** in **GRB order** (not RGB):

```
[ G7 G6 G5 G4 G3 G2 G1 G0 | R7 R6 R5 R4 R3 R2 R1 R0 | B7 B6 B5 B4 B3 B2 B1 B0 ]
```

After the 24 bits, the data line must be held **LOW for ≥ 50 µs** — this is the **reset pulse** that latches the colour into the LED's internal PWM driver.

### Why RMT instead of bit-banging?

Bit-banging with `digitalWrite` / `delayMicroseconds` on an ESP32 is unreliable because:
- The ESP32 runs FreeRTOS, and ISRs can steal the CPU mid-pulse.
- `delayMicroseconds` is not cycle-accurate at sub-microsecond granularity.

The **RMT (Remote Control Transceiver)** peripheral solves both problems. It is an independent hardware block that reads a pre-loaded array of pulse descriptors from its internal FIFO and drives the GPIO pin autonomously — no CPU involvement during transmission.

### Clock configuration

The ESP32 RMT clock runs at 80 MHz. With a clock divider of **4**, each tick is:

```
1 / (80 MHz / 4) = 50 ns per tick
```

This makes the WS2812B timing targets easy integers:

| Parameter | Target    | Ticks (÷50 ns) |
|-----------|-----------|----------------|
| T0H       | 400 ns    | 8              |
| T0L       | 850 ns    | 17             |
| T1H       | 800 ns    | 16             |
| T1L       | 450 ns    | 9              |
| Reset     | ≥ 50 µs   | 1000           |

### RMT item format

Each `rmt_item32_t` encodes **two consecutive half-pulses**:

```c
typedef struct {
    uint16_t duration0 : 15;  // ticks for first half-pulse
    uint16_t level0    :  1;  // GPIO level (1 = high)
    uint16_t duration1 : 15;  // ticks for second half-pulse
    uint16_t level1    :  1;  // GPIO level (0 = low)
} rmt_item32_t;
```

One `rmt_item32_t` = one WS2812B bit.

### GRB packing

The 24-bit word is assembled as:

```c
uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
```

Bits are sent MSB first by testing bit 23 down to bit 0 in a loop.

### Platform choice — why ESP32?

| Factor | Reason |
|--------|--------|
| **RMT peripheral** | Hardware-accurate pulse generation; no jitter |
| **3.3 V GPIO** | WS2812B DIN threshold is ~0.7 × VDD; 3.3 V signal works with 5 V-powered LEDs |
| **Arduino core support** | Familiar `setup()`/`loop()` structure; ESP-IDF RMT driver included |
| **Widespread availability** | Common dev board; easy to replicate |

An AVR (Uno) would require `cli()`/`sei()` around a hand-crafted cycle-counted loop — fragile and non-portable. The ESP32 RMT approach is cleaner and more robust.

---

## 3. Research Path

### Step 1 — Understand the protocol

**Search terms:** `"WS2812B datasheet"`, `"WS2812B timing diagram"`

- **Source:** [WS2812B Datasheet v5.0 (Worldsemi)](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf)  
  - Section **"Data Transfer Time"** gave the exact T0H, T0L, T1H, T1L values and tolerances.  
  - Section **"Timing Waveform"** confirmed GRB bit order and the 50 µs reset requirement.

### Step 2 — Find a hardware approach for ESP32

**Search terms:** `"ESP32 WS2812B RMT example"`, `"ESP-IDF RMT peripheral WS2812"`

- **Source:** [Espressif ESP-IDF Programming Guide — RMT](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)  
  - Explained the `rmt_config_t` struct fields, clock divider arithmetic, and `rmt_item32_t` bit layout.  
  - Showed that `rmt_write_items()` with `wait_tx_done = true` blocks until transmission is complete — exactly what we need for single-call colour setting.

- **Source:** [Espressif GitHub — rmt_led_strip example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/led_strip)  
  - Confirmed the 80 MHz base clock and verified ÷4 divider gives 50 ns resolution.

### Step 3 — Verify bit-banging alternative (cross-check)

**Search terms:** `"WS2812B bit bang Arduino timing"`, `"NeoPixel without library"`

- **Source:** [Adafruit NeoPixel Überguide — Wiring & Power](https://learn.adafruit.com/adafruit-neopixel-uberguide/the-magic-of-neopixels)  
  - Confirmed that pure software bit-banging on an ESP32 requires `portDISABLE_INTERRUPTS()` and is fragile — reinforcing the RMT choice.

### Key insight

The WS2812B datasheet states tolerances of ±150 ns per pulse. At 50 ns/tick, our RMT values land well within spec. No additional calibration is needed.

---

## 4. LLM Chat Log

*(This section documents how an LLM was used as a learning and verification tool.)*

**Prompt 1:**  
> "The WS2812B datasheet shows T0H = 0.4 µs and T1H = 0.8 µs. If I set the ESP32 RMT clock divider to 4 (giving 50 ns per tick), how many ticks do I need for each timing parameter?"

**LLM response (paraphrased):**  
The LLM confirmed the arithmetic: 400 ns / 50 ns = 8 ticks for T0H, 850 ns / 50 ns = 17 ticks for T0L, etc. It also pointed out that the reset pulse needs ≥ 50 µs, so ≥ 1000 ticks.

**Prompt 2:**  
> "Why is the WS2812B data order GRB and not RGB?"

**LLM response (paraphrased):**  
The LLM explained that this is a vendor decision baked into the WS2812B silicon — the internal shift register simply maps the first 8 bits to the green channel driver. There is no functional reason; it is a legacy choice from the original design. I verified this against the datasheet's "Data Transfer" section, which explicitly labels the first 8 bits as Green.

**Prompt 3:**  
> "Is `rmt_write_items()` safe to call from Arduino's `loop()` without needing a mutex?"

**LLM response (paraphrased):**  
Yes, when `wait_tx_done = true` the call is synchronous and blocks until the RMT hardware finishes. In a single-pixel setup with no concurrent tasks touching the same channel, no mutex is needed. For multi-LED chained setups a mutex would be prudent.

> **Note:** All LLM responses were verified against the ESP-IDF documentation and the WS2812B datasheet before inclusion in the final implementation.

---

## File Structure

```
ws2812b_driver/
├── ws2812b.ino      ← Complete source code
└── README.md        ← This document
```

## Build Instructions

1. Open `ws2812b.ino` in **Arduino IDE 2.x**.
2. Install the **ESP32 board package** via Board Manager (Espressif Systems → esp32).
3. Select **Board:** `ESP32 Dev Module`.
4. Connect your ESP32, select the correct COM port, and click **Upload**.
5. Wire GPIO 4 → WS2812B DIN, 3.3 V/5 V → VCC, GND → GND.
6. The LED cycles through Red → Green → Blue → Off every 500 ms.
