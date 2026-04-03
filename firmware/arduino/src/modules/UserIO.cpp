/**
 * @file UserIO.cpp
 * @brief Implementation of user I/O management
 *
 * LED_RED PWM (v0.9.0 profile)
 * ---------------------
 * LED_RED uses direct OCR writes (via the LED_RED_OCR / LED_RED_ICR macros
 * defined in pins.h) instead of analogWrite(). This resolves the previously
 * documented Timer3 CTC conflict in Rev. B and enables full PWM + breathing
 * effects on LED_RED in both hardware revisions.
 *
 *   Rev A: LED_RED on pin 11 — Timer1 OC1A — LED_RED_OCR = OCR1A
 *   Rev B: LED_RED on pin 5  — Timer3 OC3A — LED_RED_OCR = OCR3A
 *
 * See: firmware/docs/TIMER3_CONFLICT_ANALYSIS.md
 */

#include "UserIO.h"
#include "../pins.h"
#include "SensorManager.h"
#include "../SystemManager.h"

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

NeoPixelDriver UserIO::neopixel_;

UserIO::LEDState UserIO::leds_[LED_COUNT];

volatile uint16_t UserIO::buttonStates_     = 0;
volatile uint16_t UserIO::prevButtonStates_ = 0;
uint8_t               UserIO::limitStates_     = 0;
uint8_t               UserIO::animPhase_       = 0;
volatile SystemState  UserIO::pixelState_      = SYS_STATE_INIT;
volatile bool         UserIO::pixelStateDirty_ = true;
bool                  UserIO::neoAutoAnimate_  = true;
uint32_t              UserIO::neoPixelColors_[(NEOPIXEL_COUNT > 0) ? NEOPIXEL_COUNT : 1] = {0};

bool UserIO::initialized_ = false;

namespace {

void disconnectRedLedPwm()
{
#if defined(PIN_LED_RED_IS_OC1A)
    TCCR1A &= (uint8_t)~((1 << COM1A1) | (1 << COM1A0));
#elif defined(PIN_LED_RED_IS_OC3A)
    TCCR3A &= (uint8_t)~((1 << COM3A1) | (1 << COM3A0));
#endif
}

void connectRedLedPwm()
{
#if defined(PIN_LED_RED_IS_OC1A)
    TCCR1A = (uint8_t)((TCCR1A & (uint8_t)~(1 << COM1A0)) | (1 << COM1A1));
#elif defined(PIN_LED_RED_IS_OC3A)
    TCCR3A = (uint8_t)((TCCR3A & (uint8_t)~(1 << COM3A0)) | (1 << COM3A1));
#endif
}

void applyRedLedLevel(uint8_t brightness)
{
    if (brightness == 0U) {
        LED_RED_OCR = 0;
        disconnectRedLedPwm();
        digitalWrite(PIN_LED_RED, LOW);
        return;
    }

    if (brightness >= 255U) {
        LED_RED_OCR = LED_RED_ICR;
        disconnectRedLedPwm();
        digitalWrite(PIN_LED_RED, HIGH);
        return;
    }

    connectRedLedPwm();
    LED_RED_OCR = (uint16_t)(((uint32_t)brightness * (uint32_t)LED_RED_ICR) / 255UL);
}

} // namespace

// ============================================================================
// INITIALIZATION
// ============================================================================

void UserIO::init() {
    if (initialized_) return;

    // Initialize buttons (INPUT_PULLUP for active-low buttons)
    for (uint8_t i = 0; i < 10; i++) {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }

    // Initialize LEDs
    leds_[LED_RED].pin = PIN_LED_RED;
    leds_[LED_RED].mode = LED_OFF;
    pinMode(PIN_LED_RED, OUTPUT);
    applyRedLedLevel(0);

    leds_[LED_GREEN].pin = PIN_LED_GREEN;
    leds_[LED_GREEN].mode = LED_OFF;
    pinMode(PIN_LED_GREEN, OUTPUT);
    digitalWrite(PIN_LED_GREEN, LOW);

    leds_[LED_BLUE].pin = PIN_LED_BLUE;
    leds_[LED_BLUE].mode = LED_OFF;
    pinMode(PIN_LED_BLUE, OUTPUT);
    digitalWrite(PIN_LED_BLUE, LOW);

    leds_[LED_ORANGE].pin = PIN_LED_ORANGE;
    leds_[LED_ORANGE].mode = LED_OFF;
    pinMode(PIN_LED_ORANGE, OUTPUT);
    digitalWrite(PIN_LED_ORANGE, LOW);

    leds_[LED_PURPLE].pin = PIN_LED_PURPLE;
    leds_[LED_PURPLE].mode = LED_OFF;
    pinMode(PIN_LED_PURPLE, OUTPUT);
    digitalWrite(PIN_LED_PURPLE, LOW);

    // Initialize all LED states
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        leds_[i].brightness = 255;
        leds_[i].periodMs = 1000;
        leds_[i].dutyCycle = 500;
        leds_[i].lastToggle = 0;
        leds_[i].state = false;
        leds_[i].breathePhase = 0;
    }

    // Initialize NeoPixel
    neopixel_.init(PIN_NEOPIXEL, 1, 128);  // 1 LED, 50% brightness
    for (uint8_t i = 0; i < ((NEOPIXEL_COUNT > 0) ? NEOPIXEL_COUNT : 1); ++i) {
        neoPixelColors_[i] = 0;
    }
    queuePixelState(SystemManager::getState());

    // Prime input caches so button/limit status is valid immediately.
    sampleInputs();

    initialized_ = true;
    updateNeoPixelAnimation();

#ifdef DEBUG_USERIO
    DEBUG_SERIAL.println(F("[UserIO] Initialized"));
    DEBUG_SERIAL.println(F("  - 10 buttons configured"));
    DEBUG_SERIAL.println(F("  - 5 LEDs configured"));
    DEBUG_SERIAL.println(F("  - NeoPixel initialized"));
#endif
}

// ============================================================================
// UPDATE
// ============================================================================

void UserIO::update() {
    if (!initialized_) return;

    updateLEDs();
    updateNeoPixelAnimation();
}

void UserIO::sampleInputs() {
    if (!initialized_) return;

    readButtons();

    limitStates_ = 0;
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t activeState = LIMIT_ACTIVE_LOW ? LOW : HIGH;
        if (digitalRead(LIMIT_PINS[i]) == activeState) {
            limitStates_ |= (uint8_t)(1u << i);
        }
    }
}

void UserIO::serviceTask() {
    if (!initialized_) return;

    update();
}

void UserIO::syncOutputs() {
    if (!initialized_) return;

    for (uint8_t i = 0; i < LED_COUNT; i++) {
        LEDState& led = leds_[i];
        switch (led.mode) {
            case LED_OFF:
                if (led.pin == PIN_LED_RED) {
                    applyRedLedLevel(0);
                } else {
                    digitalWrite(led.pin, LOW);
                }
                led.state = false;
                break;

            case LED_ON:
            case LED_PWM:
                if (led.pin == PIN_LED_RED) {
                    applyRedLedLevel(led.brightness);
                    led.state = (led.brightness != 0U);
                } else if (led.mode == LED_PWM) {
                    analogWrite(led.pin, led.brightness);
                    led.state = (led.brightness != 0U);
                } else {
                    digitalWrite(led.pin, HIGH);
                    led.state = true;
                }
                break;

            case LED_BLINK:
            case LED_BREATHE:
                updateLED(led);
                break;

            default:
                break;
        }
    }
}

// ============================================================================
// BUTTON INTERFACE
// ============================================================================

bool UserIO::isButtonPressed(uint8_t buttonId) {
    if (buttonId >= 10) return false;

    return (buttonStates_ & (uint16_t)(1u << buttonId)) != 0;
}

uint16_t UserIO::getButtonStates() {
    return buttonStates_;
}

bool UserIO::wasButtonPressed(uint8_t buttonId) {
    if (buttonId >= 10) return false;

    // Snapshot volatile values once to avoid mid-read change
    uint16_t cur  = buttonStates_;
    uint16_t prev = prevButtonStates_;

    return ((cur & (uint16_t)(1u << buttonId)) != 0) &&
           ((prev & (uint16_t)(1u << buttonId)) == 0);
}

// ============================================================================
// LIMIT SWITCH INTERFACE
// ============================================================================

bool UserIO::isLimitTriggered(uint8_t limitId) {
    if (limitId >= 8) return false;

    return (limitStates_ & (1 << limitId)) != 0;
}

uint8_t UserIO::getLimitStates() {
    return limitStates_;
}

// ============================================================================
// LED CONTROL
// ============================================================================

void UserIO::setLED(LEDId ledId, LEDMode mode, uint8_t brightness, uint16_t periodMs, uint16_t dutyCycle) {
    if (ledId >= LED_COUNT) return;

    LEDState& led = leds_[ledId];
    led.mode = mode;
    led.brightness = brightness;
    led.periodMs = periodMs;
    led.dutyCycle = (dutyCycle > 1000U) ? 1000U : dutyCycle;
    led.lastToggle = millis();
    led.breathePhase = 0;

    // Immediately apply state for OFF, ON, and PWM modes
    if (mode == LED_OFF) {
        if (led.pin == PIN_LED_RED) {
            applyRedLedLevel(0);
        } else {
            digitalWrite(led.pin, LOW);
        }
        led.state = false;
    } else if (mode == LED_ON) {
        if (led.pin == PIN_LED_RED) {
            applyRedLedLevel(brightness);
            led.state = (brightness != 0U);
        } else {
            digitalWrite(led.pin, HIGH);
            led.state = true;
        }
    } else if (mode == LED_PWM) {
        if (led.pin == PIN_LED_RED) {
            applyRedLedLevel(brightness);
        } else {
            analogWrite(led.pin, brightness);
        }
        led.state = (brightness != 0U);
    } else if (mode == LED_BLINK || mode == LED_BREATHE) {
        // Animated modes should react immediately to a new command instead of
        // waiting for the next 20 Hz user-I/O tick.
        updateLED(led);
    }
}

void UserIO::setAllLEDsOff() {
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        setLED((LEDId)i, LED_OFF);
    }
}

uint8_t UserIO::getLEDBrightness(uint8_t ledId) {
    if (ledId >= LED_COUNT) return 0;
    const LEDState& led = leds_[ledId];
    if (led.mode == LED_OFF) return 0;
    return led.brightness;
}

// ============================================================================
// NEOPIXEL SYSTEM STATE
// ============================================================================

void UserIO::setPixelStateInit() {
    queuePixelState(SYS_STATE_INIT);
}

void UserIO::setPixelStateIdle() {
    queuePixelState(SYS_STATE_IDLE);
}

void UserIO::setPixelStateRunning() {
    queuePixelState(SYS_STATE_RUNNING);
}

void UserIO::setPixelStateError() {
    queuePixelState(SYS_STATE_ERROR);
}

void UserIO::setPixelStateEstop() {
    queuePixelState(SYS_STATE_ESTOP);
}

void UserIO::setNeoPixelColor(uint32_t color) {
    neoPixelColors_[0] = color;
    neopixel_.setPixel(0, color);
    neopixel_.show();
}

void UserIO::setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b) {
    setBufferedNeoPixel(0, r, g, b);
    neopixel_.show();
}

void UserIO::setNeoPixelBrightness(uint8_t brightness) {
    neopixel_.setBrightness(brightness);
    neopixel_.show();
}

uint32_t UserIO::getNeoPixelColor(uint8_t index) {
    if (index >= ((NEOPIXEL_COUNT > 0) ? NEOPIXEL_COUNT : 1)) {
        return 0;
    }
    return neoPixelColors_[index];
}

void UserIO::setNeoAutoAnimate(bool enable) {
    neoAutoAnimate_ = enable;
    if (enable) {
        pixelStateDirty_ = true;
    }
}

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

void UserIO::readButtons() {
    // Pure GPIO reads — no millis(), no blocking.
    // Snapshot previous before overwriting (used by wasButtonPressed()).
    prevButtonStates_ = buttonStates_;
    uint16_t states = 0;

    for (uint8_t i = 0; i < 10; i++) {
        // Buttons are active LOW (pressed = LOW)
        if (digitalRead(BUTTON_PINS[i]) == LOW) {
            states |= (uint16_t)(1u << i);
        }
    }
    buttonStates_ = states;
}

void UserIO::updateLEDs() {
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        updateLED(leds_[i]);
    }
}

void UserIO::updateLED(LEDState& led) {
    uint32_t now = millis();

    switch (led.mode) {
        case LED_OFF:
            // Already handled in setLED()
            break;

        case LED_ON:
        case LED_PWM:
            // State was applied once in setLED() — nothing more to do here.
            // LED_RED uses OCR1A/OCR3A (hardware PWM), which persists without
            // re-writing on every update tick.
            break;

        case LED_BLINK:
        {
            const uint32_t periodMs = (led.periodMs == 0U) ? 1U : led.periodMs;
            const uint16_t duty = (led.dutyCycle > 1000U) ? 1000U : led.dutyCycle;
            uint32_t elapsed = now - led.lastToggle;

            if (elapsed >= periodMs) {
                led.lastToggle += (elapsed / periodMs) * periodMs;
                elapsed = now - led.lastToggle;
            }

            const uint32_t onTimeMs = (periodMs * duty) / 1000U;
            const bool nextState =
                (duty >= 1000U) ? true :
                (duty == 0U) ? false :
                (elapsed < onTimeMs);

            if (nextState != led.state) {
                led.state = nextState;
                if (led.pin == PIN_LED_RED) {
                    applyRedLedLevel(led.state ? led.brightness : 0);
                } else {
                    digitalWrite(led.pin, led.state ? HIGH : LOW);
                }
            }
            break;
        }

        case LED_BREATHE: {
            // Asymmetric breathing:
            // dutyCycle = share of the period spent ramping up, remaining time
            // is spent ramping back down. Clamp away the 0/period singularities.
            const uint32_t periodMs = (led.periodMs < 2U) ? 2U : led.periodMs;
            uint16_t risePermille = led.dutyCycle;
            if (risePermille == 0U) risePermille = 1U;
            if (risePermille >= 1000U) risePermille = 999U;

            uint32_t riseMs = (periodMs * risePermille) / 1000U;
            if (riseMs == 0U) riseMs = 1U;
            if (riseMs >= periodMs) riseMs = periodMs - 1U;
            const uint32_t fallMs = periodMs - riseMs;

            uint32_t elapsed = now - led.lastToggle;
            if (elapsed >= periodMs) {
                led.lastToggle += (elapsed / periodMs) * periodMs;
                elapsed = now - led.lastToggle;
            }

            uint8_t brightness;
            if (elapsed < riseMs) {
                brightness = (uint8_t)((elapsed * 255U) / riseMs);
            } else {
                const uint32_t fallElapsed = elapsed - riseMs;
                brightness = (uint8_t)(255U - ((fallElapsed * 255U) / fallMs));
            }

            // Apply per-LED brightness ceiling
            brightness = (uint8_t)((brightness * led.brightness) / 255);
            led.state = (brightness != 0U);

            if (led.pin == PIN_LED_RED) {
                applyRedLedLevel(brightness);
            } else if (led.pin == PIN_LED_PURPLE) {
                digitalWrite(led.pin, (brightness > 128) ? HIGH : LOW);
            } else {
                analogWrite(led.pin, brightness);
            }
            break;
        }

        default:
            break;
    }
}

// ============================================================================
// NEOPIXEL ANIMATION
// ============================================================================

void UserIO::hsvToRgb(uint8_t h, uint8_t s, uint8_t v,
                      uint8_t& r, uint8_t& g, uint8_t& b) {
    if (s == 0) { r = g = b = v; return; }

    uint8_t region    = h / 43;                   // 0–5 (six hue sextants)
    uint8_t remainder = (uint8_t)((h - (uint8_t)(region * 43)) * 6);

    // Fast /256 approximation avoids costly division on AVR
    uint8_t p = (uint8_t)(((uint16_t)v * (255u - s)) >> 8);
    uint8_t q = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * remainder) >> 8))) >> 8);
    uint8_t t = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * (255u - remainder)) >> 8))) >> 8);

    switch (region) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
}

void UserIO::updateNeoPixelAnimation() {
    if (!neoAutoAnimate_) return;   // manual color control active — don't override

    if (!pixelStateDirty_) {
        return;
    }

    SystemState state = pixelState_;
    pixelStateDirty_ = false;
    animPhase_ = 0;

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    switch (state) {
        case SYS_STATE_INIT:
            r = 0; g = 80; b = 220;
            break;
        case SYS_STATE_IDLE:
            r = 200; g = 230; b = 0;
            break;
        case SYS_STATE_RUNNING:
            r = 0; g = 220; b = 40;
            break;
        case SYS_STATE_ERROR:
            r = 255; g = 0; b = 0;
            break;
        case SYS_STATE_ESTOP:
            r = 255; g = 0; b = 80;
            break;
        default:
            b = 30;
            break;
    }

    setBufferedNeoPixel(0, r, g, b);
    neopixel_.show();
}

void UserIO::queuePixelState(SystemState state) {
    pixelState_ = state;
    pixelStateDirty_ = true;
    animPhase_ = 0;
}

void UserIO::setBufferedNeoPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= ((NEOPIXEL_COUNT > 0) ? NEOPIXEL_COUNT : 1)) {
        return;
    }
    neoPixelColors_[index] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
    neopixel_.setPixel(index, r, g, b);
}
