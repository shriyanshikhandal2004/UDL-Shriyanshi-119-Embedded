/**
 * WS2812B Single-Pixel Driver — No external libraries
 * Platform : ESP32 (Arduino core)
 * Method   : ESP32 RMT (Remote Control Transceiver) peripheral
 *
 * The RMT peripheral generates precise pulse trains in hardware,
 * freeing the CPU from tight timing loops and making the signal
 * immune to interrupt jitter.
 */

#include <Arduino.h>
#include "driver/rmt.h"

// ── RMT configuration ─────────────────────────────────────────────────────────
#define RMT_TX_CHANNEL  RMT_CHANNEL_0
#define RMT_CLK_DIV     4          // 80 MHz / 4 = 20 MHz → 50 ns per tick

// WS2812B timing (from datasheet, all in 50 ns ticks @ 20 MHz)
// T0H = 400 ns → 8 ticks   T0L = 850 ns → 17 ticks
// T1H = 800 ns → 16 ticks  T1L = 450 ns →  9 ticks
// Reset = >50 µs → 1 000 ticks

#define T0H_TICKS   8
#define T0L_TICKS  17
#define T1H_TICKS  16
#define T1L_TICKS   9
#define RESET_TICKS 1000   // 50 µs

// ── RMT item helpers ──────────────────────────────────────────────────────────
// An rmt_item32_t encodes two back-to-back pulse segments:
//   { duration0, level0, duration1, level1 }
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

// ── One-time RMT channel initialisation ───────────────────────────────────────
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

// ── Public API ─────────────────────────────────────────────────────────────────
/**
 * setPixelColor() — Drive a single WS2812B pixel.
 *
 * @param pin  GPIO pin connected to the LED data-in line
 * @param r    Red   intensity (0–255)
 * @param g    Green intensity (0–255)
 * @param b    Blue  intensity (0–255)
 *
 * The WS2812B expects colour data in GRB order, MSB first.
 * Total payload: 24 bits (8 G + 8 R + 8 B) followed by a reset pulse.
 */
void setPixelColor(uint8_t pin, uint8_t r, uint8_t g, uint8_t b) {
    if (!rmt_initialised) {
        rmt_init(pin);
    }

    // Build the 24-bit GRB item array (+ 1 reset item)
    rmt_item32_t items[25];   // 24 data bits + 1 reset
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;

    for (int i = 0; i < 24; i++) {
        // Send MSB first: test bit 23 down to bit 0
        items[i] = (grb & (1UL << (23 - i))) ? BIT_1 : BIT_0;
    }
    items[24] = RESET_ITEM;

    // Transmit and block until done
    rmt_write_items(RMT_TX_CHANNEL, items, 25, true /* wait_tx_done */);
}

// ── Demo: cycle through R → G → B ─────────────────────────────────────────────
#define DATA_PIN 4   // GPIO 4 → WS2812B DIN

void setup() {
    Serial.begin(115200);
    Serial.println("WS2812B RMT driver ready");
}

void loop() {
    setPixelColor(DATA_PIN, 255,   0,   0);  delay(500);  // Red
    setPixelColor(DATA_PIN,   0, 255,   0);  delay(500);  // Green
    setPixelColor(DATA_PIN,   0,   0, 255);  delay(500);  // Blue
    setPixelColor(DATA_PIN, 128, 128,   0);  delay(500);  // Yellow
    setPixelColor(DATA_PIN,   0, 128, 128);  delay(500);  // Cyan
    setPixelColor(DATA_PIN, 128,   0, 128);  delay(500);  // Magenta
    setPixelColor(DATA_PIN, 255, 255, 255);  delay(500);  // White
    setPixelColor(DATA_PIN,   0,   0,   0);  delay(500);  // Off
}
