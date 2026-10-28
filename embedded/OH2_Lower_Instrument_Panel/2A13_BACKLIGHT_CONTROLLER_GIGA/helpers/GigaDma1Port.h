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
 * @file      GigaDma1Port.h
 * @author    Murtle
 * @date      28.Oct 2026
 * @version   V 1.0.0
 * @copyright Copyright 2016-2026 OpenHornet. See 2A13_BACKLIGHT_CONTROLLER_GIGA.ino for details.
 * @brief     Giga LED output: the M7 alone runs everything; all ten channels sit on GPIO port J and are sent by DMA.
 * @details   How it works (engine: helpers/DmaOut1.h):
 *              - FastLED does everything it did before (colour correction, brightness, power limit, show()).
 *              - Each channel's controller (DmaController) copies the scaled GRB bytes into a per-channel buffer.
 *              - commit() builds the data words for the whole frame (about 0.8 ms) and starts the DMA, then returns.
 *              - One timer (TIM3) and three DMA1 streams (0, 1, 2) write GPIOJ->BSRR for all channels at once, with exact
 *                350 / 800 / 1250 ns timing. A frame of 8376 bits (349 LEDs) takes about 11.2 ms. The CPU is not involved.
 *              - service() (every loop pass) detects the end of the frame and the reset gap.
 *            Interrupts stay ON all the time; there is no deaf window for DCS-BIOS.
 *
 *            Requirements:
 *              - ALL channel pins must be on port J: D25 D26 D27 D28 D29 D31 D33 D35 D37 D38 D4 D11 D12 D22.
 *                Anything else is refused (fault 6).
 *              - TIM3 and DMA1 streams 0..2 must be free (checked at start-up; otherwise fault 1 / 4). If the core uses
 *                them, change DMAP_TIMER_SEL / DMAP_FIRST_STREAM at the top of helpers/DmaOut1.h.
 *              - The frame buffer must be in DMA-reachable RAM (not TCM): checked (fault 5).
 *
 *            Faults are shown on the onboard LED as fast RED flashing, N flashes then a pause, N = fault code
 *            (helpers/GigaStatusLed.h):
 *              1 timer TIM3 already running   2 timer clock not sane   3 timing does not fit   4 a DMA1 stream 0..2 in use
 *              5 frame buffer not DMA-reachable   6 a channel pin is not on port J   7 a channel has more than 405 LEDs
 *              8 a frame was refused or failed (DMA stall / error); the frame is resent
 *            The first fault stays displayed. The LEDs keep their last state; nothing else stops.
 *********************************************************************************************************************/

#ifndef __GIGA_DMA1PORT_H
#define __GIGA_DMA1PORT_H

#if !(defined(ARDUINO_GIGA) || defined(ARDUINO_GIGA_M7))
  #error "This sketch is for the Arduino GIGA R1 (Target core: Main Core)."
#endif

#include <Arduino.h>
#include "FastLED.h"
#include "GigaStatusLed.h"
#include "LedUpdateState.h"
#include "DmaOut1.h"

namespace giga_dma {

    static const uint8_t  NSLOT = DMAP_MAX_CH;
    static const uint16_t MAX_LEDS = DMAP_MAX_BYTES / 3;                  // 405
    static const uint32_t RESET_BEFORE_US = 400;                          // low gap before a frame
    static const uint32_t RESET_AFTER_US  = 300;                          // reset / latch after a frame

    alignas(32) static uint32_t frameWords[DMAP_MAX_BITS];               // 38.9 KB: one BSRR data word per bit (cache aligned)
    static uint8_t   lbuf[NSLOT][DMAP_MAX_BYTES];                         // per-channel GRB bytes written by FastLED's show()
    static uint16_t  lenB[NSLOT];                                         // bytes in each slot this frame
    static uint32_t  slotMask[NSLOT];                                     // PJ pin mask of each slot
    static uint8_t   nSlots = 0;
    static DmapPlan  plan;
    static const uint8_t* srcs[NSLOT];
    static uint8_t   fault = 0;
    static uint32_t  cpm = 480;                                           // M7 cycles per microsecond

    static void setFault(uint8_t f) { if (!fault) fault = f; giga_status::setFault(fault); }

    // Arduino pin -> bit on port J (Arduino GIGA variant.cpp)
    static bool portJBit(uint8_t pin, uint8_t& bit) {
        switch (pin) {
            case 25: bit = 0;  return true;   // PJ0
            case 27: bit = 1;  return true;   // PJ1
            case 29: bit = 2;  return true;   // PJ2
            case 31: bit = 3;  return true;   // PJ3
            case 33: bit = 4;  return true;   // PJ4
            case 35: bit = 5;  return true;   // PJ5
            case 37: bit = 6;  return true;   // PJ6
            case 38: bit = 7;  return true;   // PJ7
            case 4:  bit = 8;  return true;   // PJ8
            case 11: bit = 10; return true;   // PJ10
            case 12: bit = 11; return true;   // PJ11
            case 22: bit = 12; return true;   // PJ12 (D22; free now that the encoder has moved to J14 D46/D48/D50)
            case 26: bit = 14; return true;   // PJ14
            case 28: bit = 15; return true;   // PJ15
            default: return false;            // PJ9 (D57) and PJ13 (D87, onboard green LED) are deliberately not offered
        }
    }

    /** Call FIRST in setup(): enables the cycle counter and starts the engine (measures the timer clock, takes DMA1 streams 0..2). */
    static void begin() {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        cpm = SystemCoreClock / 1000000UL;
        dmapInit(cpm);
        if (!dmapReady) setFault(dmapWhy);                                // 1..4
        else if (!dmapAddrOk(frameWords, sizeof(frameWords)) || !dmapAddrOk(dmapConst, sizeof(dmapConst))) setFault(5);
    }

    /** Called from Channel::initialize(): gives the pin a slot and makes it a fast output, low. 255 = refused. */
    static uint8_t allocSlot(uint8_t pin) {
        uint8_t bit;
        if (nSlots >= NSLOT || !portJBit(pin, bit)) { setFault(6); return 255; }
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        GPIOJ->OSPEEDR |= (3UL << (bit * 2));                             // fastest edges
        GPIOJ->BSRR = 1UL << (bit + 16);                                  // idle low
        slotMask[nSlots] = 1UL << bit;
        lenB[nSlots] = 0;
        return nSlots++;
    }

    /** True when a new frame may be prepared: engine ready and no frame in flight. */
    static inline bool ready() { return dmapReady && dmapPhase == DMAP_PH_IDLE; }

    /** Call right after FastLED.show() has filled the slots: builds the frame words and starts the DMA. Returns at once. */
    static void commit() {
        uint32_t masks[NSLOT]; uint8_t n = 0; uint32_t maxB = 0;
        for (uint8_t s = 0; s < nSlots; s++) if (lenB[s] > maxB) maxB = lenB[s];
        if (!maxB) return;
        for (uint8_t s = 0; s < nSlots; s++) {
            if (!lenB[s]) continue;                                       // a channel with no data is not touched
            for (uint32_t i = lenB[s]; i < maxB; i++) lbuf[s][i] = 0;     // shorter channels are padded with black
            masks[n] = slotMask[s]; srcs[n] = lbuf[s]; n++;
        }
        dmapBuildPlan(plan, n, masks, &GPIOJ->BSRR, maxB);
        dmapTranspose(plan, srcs, frameWords);
        if (!dmapBegin(plan, frameWords, RESET_BEFORE_US * cpm, RESET_AFTER_US * cpm)) {
            setFault(dmapWhy ? dmapWhy : 8);
            LedUpdateState::getInstance()->setUpdateFlag(true);           // try again
        }
    }

    /** Call every loop(): advances the frame in flight (end of frame, reset gap). */
    static void service() {
        if (dmapPhase == DMAP_PH_IDLE) return;
        if (dmapPoll() == DMAP_FAILED) {
            setFault(8);
            LedUpdateState::getInstance()->setUpdateFlag(true);           // resend
        }
    }

    /**
     * FastLED controller for one channel: copies the scaled GRB bytes into this channel's buffer instead of bit-banging a
     * pin. Colour correction, brightness and power limiting are still done by FastLED.
     */
    class DmaController : public CPixelLEDController<GRB> {
        uint8_t mSlot;
    public:
        explicit DmaController(uint8_t slot) : CPixelLEDController<GRB>(), mSlot(slot) {}

        virtual void init() FL_NO_EXCEPT override { }          // the pin was set up in allocSlot()

    protected:
        virtual void showPixels(PixelController<GRB>& pixels) override {
            if (mSlot >= NSLOT) return;
            uint8_t* buf = lbuf[mSlot];
            int idx = 0;
            int count = 0;
            while (pixels.has(1) && count < MAX_LEDS) {
                buf[idx++] = pixels.loadAndScale0();           // G
                buf[idx++] = pixels.loadAndScale1();           // R
                buf[idx++] = pixels.loadAndScale2();           // B
                pixels.advanceData();
                pixels.stepDithering();
                count++;
            }
            if (pixels.has(1)) setFault(7);                    // more LEDs than the buffer holds: the rest is not sent
            lenB[mSlot] = (uint16_t)idx;
        }
    };

}   // namespace giga_dma

#endif  // __GIGA_DMA1PORT_H
