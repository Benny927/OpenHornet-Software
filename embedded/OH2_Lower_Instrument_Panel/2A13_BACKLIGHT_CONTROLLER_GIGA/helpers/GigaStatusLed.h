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
 * @file      GigaStatusLed.h
 * @author    Murtle
 * @date      28.Oct 2026
 * @version   V 1.0.0
 * @copyright Copyright 2016-2026 OpenHornet. See 2A13_BACKLIGHT_CONTROLLER_GIGA.ino for details.
 * @brief     Standard status light on the Arduino GIGA's onboard RGB LED (active low).
 * @details     solid GREEN     DCS-BIOS data is arriving (DCS connected)
 *              flashing BLUE   no DCS-BIOS data (DCS not running, bridge not connected, or paused/menu for longer than the timeout)
 *              solid MAGENTA   mode button has switched to manual or rainbow mode (not DCS controlled)
 *
 *            "Data arriving" = at least one byte waiting on the USB serial port within the last GIGA_STATUS_TIMEOUT_MS.
 *            Checked in Board.h just before DcsBios::loop() (does not consume or change any data). The LED is only written
 *            when its state changes, so it costs nothing in the main loop. Runs on the M7; the M4 does not touch the onboard LED.
 *            Switch: GIGA_STATUS_LED in the .ino (1 = on, 0 = never touch the onboard LED).
 *********************************************************************************************************************/

#ifndef GIGA_STATUS_LED_H
#define GIGA_STATUS_LED_H

#ifndef GIGA_STATUS_LED
  #define GIGA_STATUS_LED 1
#endif
#ifndef GIGA_STATUS_TIMEOUT_MS
  #define GIGA_STATUS_TIMEOUT_MS 5000   // no DCS-BIOS bytes for this long -> "not connected"
#endif
#ifndef GIGA_STATUS_BLINK_MS
  #define GIGA_STATUS_BLINK_MS 500      // blue flash: on for this long, off for this long
#endif

#if GIGA_STATUS_LED && (defined(ARDUINO_GIGA) || defined(ARDUINO_GIGA_M7))
#include "USB/PluggableUSBSerial.h"

namespace giga_status {
    static unsigned long lastRxMs = 0;
    static bool everRx = false;
    static int  shown = -1;                       // last colour written: 0 off, 1 green, 2 blue, 3 magenta, 4 red
    static uint8_t fault = 0;                     // non-zero = output fault code (see helpers/GigaDma1Port.h): N fast red flashes, then a pause

    inline void show(int c) {
        if (c == shown) return;
        shown = c;
        digitalWrite(LEDR, (c == 3 || c == 4) ? LOW : HIGH);
        digitalWrite(LEDG, (c == 1) ? LOW : HIGH);
        digitalWrite(LEDB, (c == 2 || c == 3) ? LOW : HIGH);
    }

    inline void begin() {
        pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
        show(0);
    }

    inline void pollRx() {                        // call just before DcsBios::loop()
        if (_SerialUSB.available() > 0) { lastRxMs = millis(); everRx = true; }
    }

    inline void setFault(uint8_t f) { fault = f; }

    inline void service(bool dcsMode) {           // call every loop
        if (fault) {                                  // fault code N: N fast red flashes, then a pause
            const unsigned long step = (millis() / 150) % (2UL * fault + 5);
            show((step < 2UL * fault && !(step & 1)) ? 4 : 0);
            return;
        }
        if (!dcsMode) { show(3); return; }
        unsigned long now = millis();
        if (everRx && (now - lastRxMs) < GIGA_STATUS_TIMEOUT_MS) { show(1); return; }
        show(((now / GIGA_STATUS_BLINK_MS) & 1) ? 0 : 2);
    }
}
#else
namespace giga_status {
    inline void begin() {}
    inline void setFault(uint8_t) {}
    inline void pollRx() {}
    inline void service(bool) {}
}
#endif

#endif
