/**********************************************************************************************************************
 *        ____                   _    _                       _
 *       / __ \                 | |  | |                     | |
 *      | |  | |_ __   ___ _ __ | |__| | ___  _ __ _ __   ___| |_
 *      | |  | | '_ \ / _ \ '_ \|  __  |/ _ \| '__| '_ \ / _ \ __|
 *      | |__| | |_) |  __/ | | | |  | | (_) | |  | | | |  __/ |_
 *       \____/| .__/ \___|_| |_|_|  |_|\___/|_|  |_| |_|\___|\__|
 *             | |
 *             |_|
 *   ----------------------------------------------------------------------------------
 *  
 * @file      TempMonitor.h
 * @author    Murtle
 * @date      28.Oct 2026
 * @version   V 1.0.0
 * @copyright Copyright 2016-2026 OpenHornet. See 2A13_BACKLIGHT_CONTROLLER_GIGA.ino for details.
 * @brief     Board temperature monitoring (LM75AD on the Giga backlight shield) with trigger levels and hysteresis.
 * @details   Reads the LM75AD over I2C (Wire = D20 SDA / D21 SCL) about once a second and classifies the result:
 *            OK        below WARN                       -> temp LED off
 *            WARN      at or above TEMP_WARN_C          -> temp LED blinks (1 Hz)
 *            CRITICAL  at or above TEMP_CRIT_C          -> temp LED solid on
 *            FAULT     sensor does not answer           -> temp LED off (no false alarm), retried every 5 s
 *          A level is only left again once the temperature has fallen TEMP_HYST_C below its trigger value.
 *          The fans are not connected and are not controlled here: the fan PWM line stays HIGH (see the .ino).
 *
 * @remark    NOT compiled and NOT tested on a Giga. Written against the Arduino mbed core Wire API.
 * @remark    HARDWARE: on rev 2.0.0 the shield carries SDA through the one-way buffer U2 (DIR tied to VCC, A to B only).
 *          The sensor can therefore be written to but its ACK and data never reach the Giga: every read fails and the
 *          monitor reports FAULT. It will start working once SDA has a bidirectional path (or the sensor is wired
 *          straight to the Giga's SDA/SCL). Until then nothing here can raise a false alarm.
 * @remark    The I2C transfer takes a few hundred microseconds once a second, in loop(). Interrupts stay on.
 *********************************************************************************************************************/

#ifndef TEMP_MONITOR_H
#define TEMP_MONITOR_H

#include <Arduino.h>
#include <Wire.h>

namespace temp_monitor {

// ---- Settings (edit here) ------------------------------------------------------------------------------------
constexpr uint8_t  LM75_ADDR       = 0x48;     // A0..A2 assumed tied to GND on the shield. Check against the board.
constexpr float    TEMP_WARN_C     = 55.0f;    // WARN trigger level (deg C): LED blinks
constexpr float    TEMP_CRIT_C     = 65.0f;    // CRITICAL trigger level (deg C): LED solid
constexpr float    TEMP_HYST_C     = 3.0f;     // Must fall this far below a trigger to leave that level
constexpr uint32_t READ_PERIOD_MS  = 1000;     // Read interval while the sensor answers
constexpr uint32_t RETRY_PERIOD_MS = 5000;     // Read interval while the sensor is in FAULT
constexpr uint8_t  FAIL_LIMIT      = 3;        // Consecutive failed reads before FAULT is declared
constexpr uint32_t BLINK_HALF_MS   = 500;      // WARN blink half period

enum State : uint8_t { OK = 0, WARN = 1, CRITICAL = 2, FAULT = 3 };

namespace detail {
    inline int      ledPin      = -1;
    inline State    st          = FAULT;       // FAULT until the first good read
    inline bool     haveReading = false;
    inline float    tempC       = 0.0f;
    inline float    maxC        = -1000.0f;    // Highest reading since power-up
    inline uint8_t  failCount   = 0;
    inline uint32_t lastReadMs  = 0;
    inline bool     ledOn       = false;

    // Read the LM75 temperature register. LM75: 11-bit two's complement, left-justified, 0.125 deg C per LSB.
    inline bool readLm75(float& outC) {
        Wire.beginTransmission(LM75_ADDR);
        Wire.write((uint8_t)0x00);                         // Pointer: temperature register
        if (Wire.endTransmission() != 0) return false;     // No ACK -> not present (or ACK blocked by the buffer)
        if (Wire.requestFrom((int)LM75_ADDR, 2) != 2) return false;
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        int16_t raw = (int16_t)((msb << 8) | lsb) >> 5;    // Arithmetic shift keeps the sign
        outC = raw * 0.125f;
        return true;
    }

    inline State classify(float t, State prev) {
        // Rising: use the trigger levels. Falling: only drop a level once below (trigger - hysteresis).
        if (t >= TEMP_CRIT_C) return CRITICAL;
        if (prev == CRITICAL && t > TEMP_CRIT_C - TEMP_HYST_C) return CRITICAL;
        if (t >= TEMP_WARN_C) return WARN;
        if ((prev == WARN || prev == CRITICAL) && t > TEMP_WARN_C - TEMP_HYST_C) return WARN;
        return OK;
    }
}

// Call once from setup(). ledPin is the over-temperature LED output (Giga D2, via the buffer).
inline void begin(int ledPin) {
    detail::ledPin = ledPin;
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);                             // Off. Do not leave the buffer input floating.
    Wire.begin();
    Wire.setClock(100000);
    detail::lastReadMs = millis() - READ_PERIOD_MS;        // Read straight away on the first service()
}

// Call every loop(). Non-blocking apart from the short I2C transfer.
inline void service() {
    using namespace detail;
    const uint32_t now = millis();
    const uint32_t period = (st == FAULT) ? RETRY_PERIOD_MS : READ_PERIOD_MS;

    if ((uint32_t)(now - lastReadMs) >= period) {
        lastReadMs = now;
        float t;
        if (readLm75(t)) {
            failCount   = 0;
            haveReading = true;
            tempC       = t;
            if (t > maxC) maxC = t;
            st = classify(t, (st == FAULT) ? OK : st);
        } else if (failCount < 255 && ++failCount >= FAIL_LIMIT) {
            st = FAULT;                                    // Sensor silent: report FAULT, LED off
        }
    }

    bool want = false;                                     // LED: off / blink / solid
    if (st == CRITICAL)      want = true;
    else if (st == WARN)     want = ((now / BLINK_HALF_MS) & 1) == 0;
    if (want != ledOn) {
        ledOn = want;
        digitalWrite(ledPin, want ? HIGH : LOW);
    }
}

inline State  state()          { return detail::st; }
inline bool   hasReading()     { return detail::haveReading; }
inline float  celsius()        { return detail::tempC; }
inline float  maxCelsius()     { return detail::maxC; }

} // namespace temp_monitor

#endif
