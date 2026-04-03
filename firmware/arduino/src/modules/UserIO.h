/**
 * @file UserIO.h
 * @brief User interface I/O management (buttons, LEDs, NeoPixels)
 *
 * This module manages all user-facing I/O:
 * - 10 user buttons (active low with internal pullup)
 * - 8 limit switches (shared pins with buttons 3-10)
 * - 5 discrete user/TLV-controlled LEDs
 * - 1 NeoPixel RGB LED for automatic system state indication
 *
 * LED Modes:
 * - OFF: LED is off
 * - ON: LED is constantly on at specified brightness
 * - PWM: LED is on with PWM dimming (0-255)
 * - BLINK: LED blinks at specified period
 * - BREATHE: LED fades in and out (breathing effect)
 *
 * NeoPixel System State Colors:
 * - INIT: yellow
 * - IDLE: green
 * - RUNNING: cyan
 * - ERROR: red
 * - ESTOP: red-magenta
 *
 * Usage:
 *   UserIO::init();
 *
 *   // In scheduler task @ 100Hz:
 *   UserIO::sampleInputs();
 *
 *   // In scheduler task @ 20Hz:
 *   UserIO::update();
 *
 *   // Query buttons:
 *   if (UserIO::isButtonPressed(0)) { ... }
 *
 *   // Set LED:
 *   UserIO::setLED(LED_RED, LED_BLINK, 255, 500, 250);
 *
 *   // Queue the NeoPixel for a state entry:
 *   UserIO::setPixelStateRunning();
 */

#ifndef USERIO_H
#define USERIO_H

#include <Arduino.h>
#include <stdint.h>
#include "../drivers/NeoPixelDriver.h"
#include "../messages/TLV_Payloads.h"  // For LEDMode enum
#include "../config.h"

// LEDMode enum is defined in TLV_Payloads.h:
// LED_OFF, LED_ON, LED_PWM, LED_BLINK, LED_BREATHE

// ============================================================================
// LED IDENTIFIERS
// ============================================================================

enum LEDId {
    LED_RED = 0,        // Discrete red LED (user-controlled)
    LED_GREEN = 1,      // Discrete green LED (user-controlled)
    LED_BLUE = 2,       // User LED blue
    LED_ORANGE = 3,     // User LED orange
    LED_PURPLE = 4,     // User LED purple (non-PWM)
    LED_COUNT = 5       // Total number of LEDs
};

// ============================================================================
// USER I/O CLASS (Static)
// ============================================================================

/**
 * @brief User interface I/O manager
 *
 * Static class providing:
 * - Button and limit switch reading
 * - LED control with multiple modes
 * - NeoPixel system state indication
 */
class UserIO {
public:
    /**
     * @brief Initialize all user I/O
     *
     * Configures buttons, LEDs, and NeoPixel.
     * Must be called once in setup() before using I/O.
     */
    static void init();

    /**
     * @brief Update LED animations and NeoPixel state
     *
     * Called from scheduler at 20Hz.
     * Updates LED patterns (blink, breathe) and NeoPixel animation only.
     */
    static void update();

    /**
     * @brief Sample user buttons and shared limit-switch GPIOs.
     *
     * Called from the 100 Hz soft sensor task so button and limit state caches
     * are refreshed together on the same cadence.
     */
    static void sampleInputs();

    /**
     * @brief Service the periodic user-I/O task.
     *
     * Runs LED animations and queued NeoPixel state rendering.
     */
    static void serviceTask();

    /**
     * @brief Re-apply the current LED hardware state after timer setup.
     *
     * Timer1/Timer3 configuration can reconnect OC output channels after
     * UserIO::init() has already driven LEDs low. Call this once after the
     * real-time timers are configured so the logical LED state matches the
     * actual hardware output at startup.
     */
    static void syncOutputs();

    // ========================================================================
    // BUTTON INTERFACE
    // ========================================================================

    /**
     * @brief Read all button GPIO states
     *
     * Updates volatile buttonStates_ and prevButtonStates_.
     * Kept as a narrow helper used by sampleInputs().
     */
    static void readButtons();

    /**
     * @brief Check if button is currently pressed
     *
     * Buttons are active low (pressed = LOW, released = HIGH).
     *
     * @param buttonId Button index (0-9)
     * @return True if button is pressed
     */
    static bool isButtonPressed(uint8_t buttonId);

    /**
     * @brief Get all button states as bitmask
     *
     * Bit 0 = button 0, bit 1 = button 1, etc.
     * Bit value 1 = pressed, 0 = released.
     *
     * @return 10-bit button state bitmask
     */
    static uint16_t getButtonStates();

    /**
     * @brief Check if button was just pressed (rising edge)
     *
     * Detects transition from released to pressed.
     *
     * @param buttonId Button index (0-9)
     * @return True if button was just pressed this update cycle
     */
    static bool wasButtonPressed(uint8_t buttonId);

    // ========================================================================
    // LIMIT SWITCH INTERFACE
    // ========================================================================

    /**
     * @brief Check if limit switch is triggered
     *
     * Limit switches share pins with buttons 3-10.
     * Active state depends on LIMIT_ACTIVE_LOW setting.
     *
     * @param limitId Limit switch index (0-7)
     * @return True if limit switch is triggered
     */
    static bool isLimitTriggered(uint8_t limitId);

    /**
     * @brief Get all limit switch states as bitmask
     *
     * Bit 0 = limit 0, bit 1 = limit 1, etc.
     * Bit value 1 = triggered, 0 = not triggered.
     *
     * @return 8-bit limit switch state bitmask
     */
    static uint8_t getLimitStates();

    // ========================================================================
    // LED CONTROL
    // ========================================================================

    /**
     * @brief Set LED mode and parameters
     *
     * @param ledId LED identifier (LED_RED, LED_GREEN, etc.)
     * @param mode LED mode (OFF, ON, PWM, BLINK, BREATHE)
     * @param brightness Brightness ceiling (0-255)
     * @param periodMs Period in milliseconds (used by BLINK/BREATHE modes)
     * @param dutyCycle Duty cycle in permille (0-1000). For BLINK this is the
     *        ON-time share of each period. For BREATHE this is the rise-time
     *        share; the remaining period is used for the fall.
     */
    static void setLED(LEDId ledId,
                       LEDMode mode,
                       uint8_t brightness = 255,
                       uint16_t periodMs = 1000,
                       uint16_t dutyCycle = 500);

    /**
     * @brief Turn off all LEDs
     */
    static void setAllLEDsOff();

    /**
     * @brief Get current effective brightness of a LED (0=off, 255=full)
     *
     * For LED_OFF: returns 0.
     * For LED_ON / LED_PWM: returns the configured brightness.
     * For LED_BLINK / LED_BREATHE: returns the configured brightness ceiling
     *   (not the instantaneous value — the LED may be between on/off states).
     *
     * @param ledId LED identifier (0-4, see LEDId enum)
     * @return Brightness 0–255, or 0 if ledId is out of range
     */
    static uint8_t getLEDBrightness(uint8_t ledId);

    // ========================================================================
    // NEOPIXEL SYSTEM STATE
    // ========================================================================

    /**
     * @brief Queue the INIT-state NeoPixel presentation.
     *
     * Safe to call from any context. Rendering happens later from update().
     */
    static void setPixelStateInit();

    /**
     * @brief Queue the IDLE-state NeoPixel presentation.
     */
    static void setPixelStateIdle();

    /**
     * @brief Queue the RUNNING-state NeoPixel presentation.
     */
    static void setPixelStateRunning();

    /**
     * @brief Queue the ERROR-state NeoPixel presentation.
     */
    static void setPixelStateError();

    /**
     * @brief Queue the ESTOP-state NeoPixel presentation.
     */
    static void setPixelStateEstop();

    /**
     * @brief Set custom NeoPixel color
     *
     * @param color 32-bit RGB color (0x00RRGGBB)
     */
    static void setNeoPixelColor(uint32_t color);

    /**
     * @brief Set NeoPixel color from RGB components
     *
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     */
    static void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set NeoPixel brightness
     *
     * @param brightness Brightness (0-255)
     */
    static void setNeoPixelBrightness(uint8_t brightness);

    /**
     * @brief Read the currently buffered NeoPixel color.
     *
     * Returns the RGB color last written into the NeoPixel driver buffer. This
     * is used for telemetry/UI mirroring and does not trigger show().
     *
     * @param index Pixel index
     * @return Packed 0x00RRGGBB color
     */
    static uint32_t getNeoPixelColor(uint8_t index);

    /**
     * @brief Enable or disable the automatic state-driven NeoPixel rendering.
     *
     * When enabled (default), update() renders the latest queued system state.
     * When disabled, manual setNeoPixelColor() calls stick until changed.
     *
     * Disable in test sketches or wherever manual NeoPixel control is needed.
     *
     * @param enable true = auto animation (production default), false = manual control
     */
    static void setNeoAutoAnimate(bool enable);

private:
    // NeoPixel driver
    static NeoPixelDriver neopixel_;

    // LED state tracking
    struct LEDState {
        LEDMode mode;
        uint8_t pin;
        uint8_t brightness;
        uint16_t periodMs;
        uint16_t dutyCycle;    // Permille timing parameter for blink/breathe
        uint32_t lastToggle;
        bool state;
        uint8_t breathePhase;   // For breathe mode (0-255)
    };
    static LEDState leds_[LED_COUNT];

    // Button state tracking (updated by sampleInputs(), read by soft tasks)
    static volatile uint16_t buttonStates_;
    static volatile uint16_t prevButtonStates_;

    // Limit switch state tracking
    static uint8_t limitStates_;

    // NeoPixel animation state machine
    static uint8_t              animPhase_;       // Reserved for future state animations
    static volatile SystemState pixelState_;      // Latest queued system state
    static volatile bool        pixelStateDirty_; // True when update() should re-render state
    static bool                 neoAutoAnimate_;  // true = state-driven, false = manual
    static uint32_t             neoPixelColors_[(NEOPIXEL_COUNT > 0) ? NEOPIXEL_COUNT : 1];

    // Initialization flag
    static bool initialized_;

    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    /**
     * @brief Update LED animations (blink, breathe)
     */
    static void updateLEDs();

    /**
     * @brief Update NeoPixel rendering for the queued system state.
     *
     * Called from update() at 20 Hz. State changes are queued by SystemManager;
     * this task performs the actual NeoPixel write so the state transition path
     * itself does not call show().
     */
    static void updateNeoPixelAnimation();

    /**
     * @brief Queue a new system-state presentation for the NeoPixel.
     *
     * This is the shared implementation behind the public setPixelState*()
     * entry points. It only updates cached state; rendering happens in update().
     */
    static void queuePixelState(SystemState state);

    /**
     * @brief Write a NeoPixel color to both the driver buffer and local cache.
     */
    static void setBufferedNeoPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Convert HSV to RGB (fast integer implementation, /256 approximation)
     *
     * @param h Hue 0-255 (0=red, 85=green, 170=blue)
     * @param s Saturation 0-255
     * @param v Value/brightness 0-255
     */
    static void hsvToRgb(uint8_t h, uint8_t s, uint8_t v,
                         uint8_t& r, uint8_t& g, uint8_t& b);

    /**
     * @brief Update single LED based on mode
     *
     * @param led LED state structure
     */
    static void updateLED(LEDState& led);
};

#endif // USERIO_H
