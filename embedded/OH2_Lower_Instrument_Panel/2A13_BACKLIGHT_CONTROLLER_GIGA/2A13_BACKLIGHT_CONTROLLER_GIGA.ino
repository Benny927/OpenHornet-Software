/**************************************************************************************
 *        ____                   _    _                       _
 *       / __ \                 | |  | |                     | |
 *      | |  | |_ __   ___ _ __ | |__| | ___  _ __ _ __   ___| |_
 *      | |  | | '_ \ / _ \ '_ \|  __  |/ _ \| '__| '_ \ / _ \ __|
 *      | |__| | |_) |  __/ | | | |  | | (_) | |  | | | |  __/ |_
 *       \____/| .__/ \___|_| |_|_|  |_|\___/|_|  |_| |_|\___|\__|
 *             | |
 *             |_|
 *   ----------------------------------------------------------------------------------
 *   Copyright 2016-2025 OpenHornet
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *   ----------------------------------------------------------------------------------
 *   Note: All other portions of OpenHornet not within the 'OpenHornet-Software' 
 *   GitHub repository is released under the Creative Commons Attribution -
 *   Non-Commercial - Share Alike License. (CC BY-NC-SA 4.0)
 *   ----------------------------------------------------------------------------------
 *   This Project uses Doxygen as a documentation generator.
 *   Please use Doxygen capable comments.
 **************************************************************************************/
/**
 * @file    2A13_BACKLIGHT_CONTROLLER_GIGA.ino
 * @author  Ulukaii, Arribe, Higgins, Murtle 
 * @date    Oct 27 2026
 * @version V 1.0.0
 * @warning This sketch is based on OH-Interconnect. Adapt it to your actual wiring and 
 *          panel versions.
 * @brief   Controls backlights & most annunciators. 
 * @details **Intended Board**
 *          For Arduino GIGA R1 + Giga backlight shield as in PR 1292 (rev 2.0.0 layout with Chan 2, 3, 4, 5, 6, 7 moved so all ten channels are on port J). Single processor:
 *          this M7 sketch only; the M4 is not used.
 *
 *          **Wiring Diagram:**
 *          All ten channels are on GPIO port J (the LEDs are sent by DMA: TIM3 + DMA1 streams 0..2). Pins as wired on the
 *          modified PR 1292 board (27 Oct 2026): J21 pad 33 (D22) = Chan 2, pad 32 (D25) = Chan 3, pad 30 (D27) = Chan 4,
 *          pad 29 (D26) = Chan 5, pad 27 (D28) = Chan 6, pad 26 (D31) = Chan 7. The pin numbers below are the physical Giga pins.
 *
 *          | Pin | Function                      | Channel on PCB   |
 *          |-----|-------------------------------|------------------|
 *          | 4   | J2 LIP_CH1  (PJ8)             | Channel 1        |
 *          | 22  | J3 LIP_CH2  (PJ12)            | Channel 2        |
 *          | 25  | J4 UIP_CH1  (PJ0)             | Channel 3        |
 *          | 27  | J5 UIP_CH2  (PJ1)             | Channel 4        |
 *          | 26  | J6 LC_CH1   (PJ14)            | Channel 5        |
 *          | 28  | J7 LC_CH2   (PJ15)            | Channel 6        |
 *          | 31  | J8 RC_CH1   (PJ3)             | Channel 7        |
 *          | 35  | J9 RC_CH2   (PJ5)             | Channel 8        |
 *          | 37  | J10 LC_FLOOD (PJ6)            | Channel 9        |
 *          | 38  | J11 RC_FLOOD (PJ7)            | Channel 10       |
 *          | 50  | J14 pin 3, encoder DT (B)     |                  |
 *          | 48  | J14 pin 4, encoder CLK (A)    |                  |
 *          | 46  | J14 pin 5, encoder SW (push)  |                  |
 *          | SDA | TEMP SNSR                     |                  |
 *          | SCL | TEMP SNSR                     |                  |
 *          | 39  | Over-temperature LED (D3)     |                  |
 *          | 32  | J13 fan PWM (output)          |                  |
 *          | 30  | J12 fan PWM (output)          |                  |
 *          | 5   | J13 fan tach (input)          |                  |
 *          | 3   | J12 fan tach (input)          |                  |
 *          | 0/1/2 | RS485 RX / TX / DE+RE (not used by this sketch yet) |   | *
 *
 *          
 * 
 *          **User guide**
 *          The BLM will start in mode 1 (DCS-BIOS) mode by default. This means you should
 *          see only dark panels. By pressing the rotary encoder, you can switch to mode 2
 *          (manual mode) and see manually dimmable backlights in green. Pressing again, 
 *          you should see a rainbow pattern. Pressing again, you get back to mode 1.
 *
 *
 * 
 */

/**********************************************************************************************************************
 * @brief  Preprocessor directives & library includes

 *********************************************************************************************************************/
#define DCSBIOS_DEFAULT_SERIAL                                        // GIGA native USB serial
#define DCSBIOS_DISABLE_SERVO                                         // Disable DCS-BIOS servo support (not used)

// ---------------------------------------------------------------------------------------------------
// BOARD: Arduino GIGA R1 + Giga backlight shield (5 V buffered data lines, so no dummy byte is sent).
// Pinout (modified PR 1292 board, all port J): CH1=4, CH2=22, CH3=25, CH4=27, CH5=26, CH6=28, CH7=31, CH8=35, CH9=37, CH10=38; encoder CLK=48 DT=50 SW=46; fan PWM 32 / 30, fan tach 5 / 3, temp LED=39; RS485 on D0/D1/D2 (reserved).
// GIGA only: 1 = receive DCS-BIOS with DTR OFF (normal DCS-BIOS / Bridge settings, same as the Megas).
// 0 = stock behaviour: the GIGA ignores incoming data unless DTR is turned on for its port.
#define GIGA_USB_NO_DTR 1
// ---------------------------------------------------------------------------------------------------
// GIGA SINGLE PROCESSOR (see helpers/GigaDma1Port.h): the M7 alone runs DCS-BIOS, panels and the LED output. All ten channels
// are sent together by DMA on port J (TIM3 + DMA1 streams 0..2), so there is no M4 sketch and no RPC, and the
// M7 never stops listening to DCS-BIOS while LEDs update. Needs the modified PR 1292 shield: every channel on a port J pin (PIN_CHn below).
// Upload with Tools > Target core: Main Core. Nothing else to upload.
// A LED output fault shows on the onboard LED as N fast red flashes, then a pause (N = code, see helpers/GigaDma1Port.h).
// GIGA onboard RGB LED as the standard status light: solid green = DCS-BIOS data arriving, flashing blue = no DCS-BIOS data,
// solid magenta = manual / rainbow mode. See helpers/GigaStatusLed.h. 0 = never touch the onboard LED.
#define GIGA_STATUS_LED 1
// ---------------------------------------------------------------------------------------------------

#include "DcsBios.h"
#include "FastLED.h"
#include "RotaryEncoder.h"
#include "helpers/Panel.h"
#include "helpers/Channel.h"
#include "helpers/Colors.h"
#include "helpers/Board.h"
#include "helpers/GigaDma1Port.h"
#include "helpers/TempMonitor.h"
#include "panels/1A2A1_MASTER_ARM.h"
#include "panels/1A4_L_EWI.h"
#include "panels/1A7A1_HUD_PANEL_REV4.h"            
#include "panels/1A7A1_HUD_PANEL_REV3.h"          
#include "panels/1A5_R_EWI.h"
#include "panels/1A6A1_SPN_RCVY.h"
#include "panels/2A2A1A6_ECM.h"
#include "panels/2A2A1A7_RWR_CONTROL.h"
#include "panels/2A2A1A8_STANDBY_INSTRUMENT.h"
#include "panels/4A2A1_LDG_GEAR_PANEL.h"
#include "panels/4A3A1_SELECT_JETT_PANEL.h"
#include "panels/4A4A1_FIRE_TEST_PANEL.h"
#include "panels/10A1A1_SIM_PWR_PANEL.h"
#include "panels/4A4A2_GEN_TIE_PANEL.h"
#include "panels/4A4A2_EXT_LTS_PANEL.h"
#include "panels/4A5A1_FUEL_PANEL.h"
#include "panels/4A1_LC2_ALL_PANELS.h"
#include "panels/4A1_LC_Flood.h"
#include "panels/5A1_RC_Flood.h"
#include "panels/5A2A7_LDG_CHECKLIST.h"
#include "panels/5A2A4_RADAR_ALT.h"
#include "panels/5A4A1_HYD_PRESS.h"
#include "panels/5A3A1_CAUTION.h"
#include "panels/5A2A3_AVCOOL_PANEL.h"
#include "panels/5A2A3_RC1_ALL_REMAINING_PANELS.h"
#include "panels/5A5_RC2_ALL_PANELS.h"
#include "panels/2A2A1A3_IFEI_Panel.h"
#include "panels/2A2A1A3_Video_Record_Panel.h"
#include "panels/2A2A1A1_Jett_Station_Panel.h"
#include "panels/2A2A1A4_Jett_Placard_Panel.h"



/********************************************************************************************************************
 * @brief   Define pinouts, LED counts and LED channels 
 * @remark  Check that the pinout corresponds to (YOUR!) wiring.
 * @details Syntax: Channel <Name as on Interconnect>(hardware pin, "Channel name as on PCB", expected max. led count);
 *          When adapting below code, observe memory constraints. Each LED uses 3 bytes of SRAM. 8KB are available.
 ********************************************************************************************************************/


// Voltage and current definitions. Adapt to your ATX PSU specs.
const int VOLTAGE = 5;
const int MAX_MILLIAMPS = 20000;

// Hardware pin definitions
// PR 1292 board. Encoder (J14), confirmed by the board designer 27 Oct: push = PH15 (D46, pin 5), CLK = PK0 (D48, pin 4), DT = PI11 (D50, pin 3).
const int encSw =    46;              // J14 pin 5 (push)  PH15
const int encA  =    48;              // J14 pin 4 (CLK)   PK0
const int encB  =    50;              // J14 pin 3 (DT)    PI11
const int FAN_PWM_J13   = 32;         // J13 fan PWM (pin 4), via buffer. Plain output, held HIGH = full speed (D32 = PK4: no analogWrite).
const int FAN_PWM_J12   = 30;         // J12 fan PWM (pin 4), via buffer. Plain output, held HIGH = full speed (D30 = PK3: no analogWrite).
const int FAN_TACH_J13  = 5;          // J13 fan tach / sense (pin 3). Input, not read yet.
const int FAN_TACH_J12  = 3;          // J12 fan tach / sense (pin 3). Input, not read yet.
const int TEMP_LED_PIN  = 39;         // Over-temperature LED (red D3 on the board), via buffer. Driven by temp_monitor (off / blink / solid). Trigger levels: helpers/TempMonitor.h.

// LED counts for each channel
const int LIP_1_LED_COUNT = 100;
const int LIP_2_LED_COUNT = 120;
const int UIP_1_LED_COUNT = 210;
const int UIP_2_LED_COUNT = 210;
const int LC_1_LED_COUNT = 250;
const int LC_2_LED_COUNT = 215;
const int RC_1_LED_COUNT = 171;
const int RC_2_LED_COUNT = 349;
const int LC_FLOOD_LED_COUNT = 40;
const int RC_FLOOD_LED_COUNT = 40;

// Static LED arrays for each channel, using the LED counts defined abv
CRGB LIP_1_leds[LIP_1_LED_COUNT];    
CRGB LIP_2_leds[LIP_2_LED_COUNT];    
CRGB UIP_1_leds[UIP_1_LED_COUNT];    
CRGB UIP_2_leds[UIP_2_LED_COUNT];    
CRGB LC_1_leds[LC_1_LED_COUNT];     
CRGB LC_2_leds[LC_2_LED_COUNT];     
CRGB RC_1_leds[RC_1_LED_COUNT];     
CRGB RC_2_leds[RC_2_LED_COUNT];     
CRGB LC_FLOOD_leds[LC_FLOOD_LED_COUNT];    
CRGB RC_FLOOD_leds[RC_FLOOD_LED_COUNT];    

// Create objects of "channel" class (with the right LED count)
// Channel pin map
// Channel pin map for the modified PR 1292 board (the order of the Mini-Fit connectors J2..J11 = "Chan 1".."Chan 10" on the PCB).
// All ten are port J pins (physical Giga pins; the net names on the board may still show the old numbers). Any pin accepted by
// helpers/GigaDma1Port.h (pinPort) works, in any order. A pin that is not on port J is refused (fault 6).
#define PIN_CH1  4
#define PIN_CH2  22
#define PIN_CH3  25
#define PIN_CH4  27
#define PIN_CH5  26
#define PIN_CH6  28
#define PIN_CH7  31
#define PIN_CH8  35
#define PIN_CH9  37
#define PIN_CH10 38

Channel LIP_1(PIN_CH1, "Channel 1", LIP_1_leds, LIP_1_LED_COUNT);
Channel LIP_2(PIN_CH2, "Channel 2", LIP_2_leds, LIP_2_LED_COUNT);
Channel UIP_1(PIN_CH3, "Channel 3", UIP_1_leds, UIP_1_LED_COUNT);
Channel UIP_2(PIN_CH4, "Channel 4", UIP_2_leds, UIP_2_LED_COUNT);
Channel LC_1(PIN_CH5, "Channel 5", LC_1_leds, LC_1_LED_COUNT);
Channel LC_2(PIN_CH6, "Channel 6", LC_2_leds, LC_2_LED_COUNT);
Channel RC_1(PIN_CH7, "Channel 7", RC_1_leds, RC_1_LED_COUNT);
Channel RC_2(PIN_CH8, "Channel 8", RC_2_leds, RC_2_LED_COUNT);
Channel LC_FLOOD(PIN_CH9, "Channel 9", LC_FLOOD_leds, LC_FLOOD_LED_COUNT);
Channel RC_FLOOD(PIN_CH10, "Channel 10", RC_FLOOD_leds, RC_FLOOD_LED_COUNT);          

// Create pointer to singleton board instance
Board* board;                                                         


/********************************************************************************************************************
 * @brief Standard Arduino setup and loop functions.
 * @remark Setup runs once, loop runs continuously. Conversion CRGB --> GRB is done by FastLED.
 *         Note that the call to DCS-Bios::loop() is done in the Board.h, where the mode logic resides.
 ********************************************************************************************************************/
void setup() {
    giga_status::begin();                                             // GIGA status light (onboard LED), no-op on other boards
    giga_dma::begin();                                                // Starts the DMA engine (measures the timer clock, takes DMA1 streams 0..2)
    pinMode(FAN_PWM_J13, OUTPUT); digitalWrite(FAN_PWM_J13, HIGH);    // Fan PWM lines: held HIGH = full speed
    pinMode(FAN_PWM_J12, OUTPUT); digitalWrite(FAN_PWM_J12, HIGH);    // (no PWM duty control yet)
    pinMode(FAN_TACH_J13, INPUT_PULLUP);                              // Fan tach lines are inputs (open-collector tach, 10k series on the board)
    pinMode(FAN_TACH_J12, INPUT_PULLUP);
    temp_monitor::begin(TEMP_LED_PIN);                                // LM75 temperature monitor + over-temperature LED (off at start)
    board = Board::getInstance();                                     // Get board instance
    board->setupRotaryEncoder(encSw, encA, encB);                    // Set up rotary encoder with switch and encoder pins
    
    // Initialize all channels
    LIP_1.initialize();                                               // Calling .initialize() on a channel object will
    LIP_2.initialize();                                               // trigger FastLED.addLeds() with pin and led count
    UIP_1.initialize();                                               
    UIP_2.initialize();                                               
    LC_1.initialize();
    LC_2.initialize();
    RC_1.initialize();
    RC_2.initialize();
    LC_FLOOD.initialize();
    RC_FLOOD.initialize();

    board->registerChannel(&LIP_1);                                   // Register channels with the board, so they are
    board->registerChannel(&LIP_2);                                   // accessible by the board object
    board->registerChannel(&UIP_1);
    board->registerChannel(&UIP_2);
    board->registerChannel(&LC_1);
    board->registerChannel(&LC_2);
    board->registerChannel(&RC_1);
    board->registerChannel(&RC_2);
    board->registerChannel(&LC_FLOOD);
    board->registerChannel(&RC_FLOOD);

    UIP_1.addPanel<MasterArmPanel>();                                 // Instantiate the panels;
    UIP_1.addPanel<EwiPanel>();                                       // Adapt order according to your physical wiring; 
    UIP_1.addPanel<HudPanel>();                                       // Do not exceed the channel's LED count defined above.
    //UIP_1.addPanel<HudPanelRev3>();
    UIP_1.addPanel<REwiPanel>();
    UIP_1.addPanel<SpnRcvyPanel>();

    LIP_1.addPanel<IfeiPanel>();
    LIP_1.addPanel<VideoRecordPanel>();
    LIP_1.addPanel<JettStationPanel>();
    LIP_1.addPanel<JettPlacardPanel>();

    LIP_2.addPanel<EcmPanel>();
    LIP_2.addPanel<RwrControlPanel>();
    LIP_2.addPanel<StandbyInstrumentPanel>();

    LC_1.addPanel<LdgGearPanel>();
    LC_1.addPanel<SelectJettPanel>();
    LC_1.addPanel<FireTestPanel>();
    LC_1.addPanel<SimPwrPanel>();
    LC_1.addPanel<GenTiePanel>();
    LC_1.addPanel<ExtLtsPanel>();
    LC_1.addPanel<FuelPanel>();
    
    LC_2.addPanel<Lc2AllPanels>();

    RC_1.addPanel<LdgChecklistPanel>();
    RC_1.addPanel<RadarAltPanel>();
    RC_1.addPanel<HydPressGauge>();
    RC_1.addPanel<CautionPanel>();
    RC_1.addPanel<AvcoolPanel>();
    RC_1.addPanel<Rc1AllRemainingPanels>();
    RC_2.addPanel<Rc2AllPanels>();

    LC_FLOOD.addPanel<LcFloodLights>();
    RC_FLOOD.addPanel<RcFloodLights>();                             

    FastLED.setMaxPowerInVoltsAndMilliamps(VOLTAGE, MAX_MILLIAMPS);   // Set the maximum power in volts and milliamps
    FastLED.setMaxRefreshRate(400);                                   // Max refresh rate 400 Hz (FastLED's standard). No effect here: frames are >= 11 ms apart.
                                                                      // (first frame goes out from loop() via Board::updateLeds())
    DcsBios::setup();                                                 // Run DCS Bios setup function
}

void loop() {
    board->handleModeChange();                                        // Handle mode changes
    board->processMode();                                             // Process current mode, incl. DCS-Bios::loop()
    giga_dma::service();                                              // Advances the DMA frame in flight (call every pass)
    temp_monitor::service();                                          // Read temperature (1 Hz), drive over-temperature LED
    board->updateLeds();                                              // Update LEDs as needed
}
