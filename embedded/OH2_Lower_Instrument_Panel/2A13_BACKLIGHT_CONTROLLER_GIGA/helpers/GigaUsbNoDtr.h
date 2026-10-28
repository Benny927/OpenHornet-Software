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
 * @file      GigaUsbNoDtr.h
 * @author    Murtle
 * @date      28.Oct 2026
 * @version   V 1.0.0
 * @copyright Copyright 2016-2026 OpenHornet. See 2A13_BACKLIGHT_CONTROLLER_GIGA.ino for details.
 * @brief     Let the Arduino GIGA R1 receive USB serial data without DTR.
 * @details   The GIGA's native USB serial (Arduino mbed core, USBCDC) throws away incoming
 *            data until the PC raises DTR. DCS-BIOS opens ports with DTR off (so Megas don't
 *            reset), so the GIGA never hears it. gigaUsbNoDtrPoll() tells the USB serial port
 *            it is connected as soon as a program has opened it at 250000 baud, even with DTR off.
 *
 *            Called before every DcsBios::loop() in helpers/Board.h. Switch: GIGA_USB_NO_DTR in the .ino.
 *            Tested 21 Oct 2026 with the GigaDcsTestNoDtr sketch: DCS-BIOS Bridge with DTR off works.
 *
 *            CAVEATS (read before relying on it):
 *            - Uses internal (protected) parts of the Arduino mbed core's USBCDC class.
 *              Written against ArduinoCore-mbed main (Oct 2026). A core update could break it.
 *            - With no program reading the port, a Serial.write() can block. DCS-BIOS only
 *              writes when the board sends commands to DCS (buttons/switches). This test
 *              sketch and the backlight controller don't send any.
 *            - Only active once the port has been opened at 250000 baud (DCS-BIOS's speed),
 *              so normal uploads and the Arduino Serial Monitor are not affected.
 *********************************************************************************************************************/

#ifndef GIGA_USB_NO_DTR_H
#define GIGA_USB_NO_DTR_H

#ifndef GIGA_USB_NO_DTR
  #define GIGA_USB_NO_DTR 1     // 1 = accept DCS-BIOS data with DTR off (set in the .ino)
#endif

#if (defined(ARDUINO_GIGA) || defined(ARDUINO_GIGA_M7)) && GIGA_USB_NO_DTR
#include "USB/PluggableUSBSerial.h"

struct GigaUsbNoDtr : public arduino::USBSerial {
  // Never instantiated: only used to reach USBCDC's protected members.
  static void force(arduino::USBSerial &s) {
    s.arduino::USBCDC::lock();     // (qualified: mbed::Stream also has a lock())
    (s.*(&GigaUsbNoDtr::_change_terminal_connected))(true);   // treat the port as open
    (s.*(&GigaUsbNoDtr::_receive_isr_start))();              // start receiving
    s.arduino::USBCDC::unlock();
  }
};

inline void gigaUsbNoDtrPoll() {
  if (_SerialUSB.connected()) return;          // already open (DTR on, or forced earlier)
  if (!PluggableUSBD().configured()) return;   // USB not enumerated yet
  if (_SerialUSB.baud() != 250000) return;     // port not opened by DCS-BIOS (yet)
  GigaUsbNoDtr::force(_SerialUSB);
}
#else
inline void gigaUsbNoDtrPoll() {}
#endif

#endif
