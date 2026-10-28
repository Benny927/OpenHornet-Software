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
 * @file      DmaOut1.h
 * @author    Murtle
 * @date      28.Oct 2026
 * @version   V 1.0.0
 * @copyright Copyright 2016-2026 OpenHornet. See 2A13_BACKLIGHT_CONTROLLER_GIGA.ino for details.
 * @brief     Single-port DMA engine for WS2812 outputs on the Arduino GIGA R1 (Cortex-M7 only).
 * @details   All active channels sit on ONE GPIO port (port J). ONE timer (TIM3 by default) runs a bit period of 1250 ns and raises
 *            three DMA requests per bit. THREE DMA1 streams, each on its own DMAMUX request, write the port's BSRR register:
 *              stream A  (timer update,   at count 0)    writes SET-ALL    : every active pin goes high
 *              stream B  (timer compare1, at T0H = 350)  writes the DATA word of this bit: resets the pins whose data bit is 0
 *              stream C  (timer compare2, at T1H = 800)  writes CLEAR-ALL  : every active pin goes low
 *            The data words of the whole frame (one 32-bit word per bit, 8 words per byte) are prepared by the CPU before the frame
 *            (dmapTranspose), so the CPU does nothing during the frame.
 *
 *            Use: dmapInit() once; per frame build the plan (dmapBuildPlan) and the words (dmapTranspose), call dmapBegin(), then call
 *            dmapPoll() on every pass of loop() until the phase is back to DMAP_PH_IDLE. Nothing blocks.
 *
 *            Rules: one timer request line feeds only ONE stream (ST AN5224), so three requests = three streams; only BSRR is written;
 *            TIM2 and TIM5 are not used by this engine (the core uses them).
 *
 *            DMA1 cannot reach the M7's TCM. Buffers must be in AXI SRAM (0x24000000), SRAM1-3 (0x30000000) or SRAM4 (0x38000000);
 *            dmapBegin() refuses a frame buffer anywhere else (dmapWhy = 5). The frame buffer is cache-cleaned before the DMA starts.
 *
 *            Frame end: all three streams run for exactly nBits transfers (normal mode, not circular) and stop by themselves. Stream C
 *            finishes last; then the timer is stopped. A late dmapPoll() is harmless.
 *
 *            Everything above DMAP_HOST_SIM is plain C++ so a PC test can compile the same buffer logic.
 *********************************************************************************************************************/

#ifndef DMA_OUT1_H
#define DMA_OUT1_H

#include <stdint.h>

#define DMAP_MAX_CH     10
#define DMAP_MAX_BYTES  1216                       // longest channel the frame buffer holds (= SPLIT_SLOT_STRIDE)
#define DMAP_MAX_BITS   (DMAP_MAX_BYTES * 8)

#ifndef DMAP_T0H_NS
#define DMAP_T0H_NS     350                        // end of the 0-bit pulse  (WS2812B typ. 350..400)
#endif
#ifndef DMAP_T1H_NS
#define DMAP_T1H_NS     800                        // end of the 1-bit pulse  (WS2812B typ. 700..800)
#endif
#ifndef DMAP_PERIOD_NS
#define DMAP_PERIOD_NS  1250                       // bit period
#endif

enum { DMAP_SENT = 0, DMAP_FAILED = 2, DMAP_PENDING = 3 };

// All active channels sit on one port. Masks are the pin masks (bit n = pin n of the port).
struct DmapPlan {
  uint8_t            nCh;                          // active channels this frame
  uint32_t           chMaskHi[DMAP_MAX_CH];        // channel pin mask << 16 (the BSRR "reset" half)
  uint32_t           setAll;                       // BSRR value raising every active pin
  uint32_t           clrAll;                       // BSRR value lowering every active pin
  uint32_t           nBytes;                       // bytes per channel this frame (longest channel)
  volatile uint32_t* bsrr;                         // BSRR address of the port
};

static inline void dmapBuildPlan(DmapPlan& P, uint8_t nCh, const uint32_t* mask, volatile uint32_t* bsrr, uint32_t nBytes) {
  P.nCh = nCh; P.setAll = 0; P.bsrr = bsrr; P.nBytes = nBytes;
  for (uint8_t c = 0; c < nCh; c++) { P.chMaskHi[c] = mask[c] << 16; P.setAll |= mask[c]; }
  P.clrAll = P.setAll << 16;
}

// Timer counts for a given timer clock (Hz): period, end of 0-bit pulse, end of 1-bit pulse. Rounded to the nearest count.
static inline uint32_t dmapCounts(uint32_t hz, uint32_t ns) { return (uint32_t)(((uint64_t)hz * ns + 500000000ull) / 1000000000ull); }
static inline void dmapTiming(uint32_t hz, uint32_t& arr, uint32_t& c1, uint32_t& c2) {
  arr = dmapCounts(hz, DMAP_PERIOD_NS) - 1u;
  c1  = dmapCounts(hz, DMAP_T0H_NS);
  c2  = dmapCounts(hz, DMAP_T1H_NS);
}

// Builds the data word of every bit of the frame: buf[8*i + k] = data word of bit k (MSB first) of byte i.
// Word = reset bits of all active pins, minus the pins whose data bit is 1 (those stay high until the clear-all word).
// src[c][i] = byte i of channel c; every channel has at least P.nBytes valid bytes (zero padded by the caller).
static void __attribute__((optimize("O3,unroll-loops")))
dmapTranspose(const DmapPlan& P, const uint8_t* const* src, uint32_t* buf) {
  const uint8_t nc = P.nCh;
  const uint32_t clr = P.clrAll;
  for (uint32_t i = 0; i < P.nBytes; i++) {
    uint32_t r0 = clr, r1 = clr, r2 = clr, r3 = clr, r4 = clr, r5 = clr, r6 = clr, r7 = clr;
    for (uint8_t c = 0; c < nc; c++) {
      const uint32_t b = src[c][i], m = P.chMaskHi[c];
      r0 ^= ((b >> 7) & 1u) * m;  r1 ^= ((b >> 6) & 1u) * m;
      r2 ^= ((b >> 5) & 1u) * m;  r3 ^= ((b >> 4) & 1u) * m;
      r4 ^= ((b >> 3) & 1u) * m;  r5 ^= ((b >> 2) & 1u) * m;
      r6 ^= ((b >> 1) & 1u) * m;  r7 ^= (b & 1u) * m;
    }
    uint32_t* w = buf + 8u * i;
    w[0] = r0; w[1] = r1; w[2] = r2; w[3] = r3; w[4] = r4; w[5] = r5; w[6] = r6; w[7] = r7;
  }
}


#ifndef DMAP_HOST_SIM
// ============================ hardware part (STM32H747, Cortex-M7) ============================

// Timer choice: TIM3 (default) or TIM4. Both are general-purpose, have CC1/CC2/UP requests, need no break / main-output setup.
// (TIM1 / TIM8 would also work but are not offered here. TIM2 and TIM5 are used by the core: never.)
// DMAMUX1 request ids from ST's stm32h7xx_hal_dma.h (DMA_REQUEST_TIMx_CH1 / CH2 / UP), checked 27 Oct 2026.
#ifndef DMAP_TIMER_SEL
#define DMAP_TIMER_SEL 3
#endif
#if DMAP_TIMER_SEL == 3
#define DMAP_TIM      TIM3
#define DMAP_REQ_UP   27u
#define DMAP_REQ_CC1  23u
#define DMAP_REQ_CC2  24u
#elif DMAP_TIMER_SEL == 4
#define DMAP_TIM      TIM4
#define DMAP_REQ_UP   32u
#define DMAP_REQ_CC1  29u
#define DMAP_REQ_CC2  30u
#else
#error "DMAP_TIMER_SEL must be 3 or 4"
#endif

#ifndef DMAP_FIRST_STREAM
#define DMAP_FIRST_STREAM 0                        // uses DMA1 streams FIRST, FIRST+1, FIRST+2  (A, B, C)
#endif
#ifndef DMAP_FIRST_STREAM
#define DMAP_FIRST_STREAM 0                        // uses DMA1 streams FIRST, FIRST+1, FIRST+2  (A, B, C)
#endif

#define DMAP_FLAG_DME 0x04u
#define DMAP_FLAG_TE  0x08u
#define DMAP_FLAG_HT  0x10u
#define DMAP_FLAG_TC  0x20u

#define DMAP_PH_IDLE 0
#define DMAP_PH_GAP  1                             // streams armed, waiting out the low gap before the first bit
#define DMAP_PH_RUN  2                             // timer running, bits going out
#define DMAP_PH_TAIL 3                             // last bit out, waiting out the low gap after the frame

// why dmapReady is false: 0 ready, 1 timer already running, 2 timer clock not sane, 3 timing does not fit a 16-bit timer,
// 4 a DMA1 stream we need is already enabled (used by something else), 5 frame buffer or constant words not DMA-reachable
static uint8_t  dmapWhy = 0;
static bool     dmapReady = false;
static uint32_t dmapArr = 0, dmapC1 = 0, dmapC2 = 0, dmapCpm = 480;
alignas(32) static uint32_t dmapConst[8];          // [0] set-all word, [1] clear-all word (the DMA reads these)

// state of the frame in progress
static uint8_t  dmapPhase = DMAP_PH_IDLE;
static int      dmapLast = DMAP_SENT;              // result of the last finished frame
static uint32_t dmapTs = 0, dmapTw = 0, dmapGapCyc = 0, dmapTailCyc = 0, dmapTmoCyc = 0;
static volatile uint32_t* dmapBsrr = 0;
static uint32_t dmapClr = 0;

static inline DMA_Stream_TypeDef* dmapStream(int n) { return (DMA_Stream_TypeDef*)((uint32_t)DMA1_Stream0 + 0x18u * n); }
static inline DMAMUX_Channel_TypeDef* dmapMux(int n) { return (DMAMUX_Channel_TypeDef*)((uint32_t)DMAMUX1_Channel0 + 4u * n); }
static const uint8_t dmapFlagOff[4] = { 0, 6, 16, 22 };
static inline uint32_t dmapFlagsGet(int n) { uint32_t r = (n < 4) ? DMA1->LISR : DMA1->HISR; return (r >> dmapFlagOff[n & 3]) & 0x3Du; }
static inline void dmapFlagsClear(int n, uint32_t bits) { uint32_t m = bits << dmapFlagOff[n & 3]; if (n < 4) DMA1->LIFCR = m; else DMA1->HIFCR = m; }

static inline uint32_t dmapAddrOf(const void* p) { return (uint32_t)p; }

// DMA1 reaches AXI SRAM, SRAM1-3 and SRAM4, not the M7's TCM. True if [p, p+bytes) lies inside one of those.
static inline bool dmapAddrOk(const void* p, uint32_t bytes) {
  const uint32_t a = dmapAddrOf(p), e = a + bytes - 1u;
  return (a >= 0x24000000u && e < 0x24080000u) || (a >= 0x30000000u && e < 0x30048000u) || (a >= 0x38000000u && e < 0x38010000u);
}

// The DMA reads RAM, not the data cache: write the buffer back before the DMA starts.
static inline void dmapCacheClean(const void* p, uint32_t bytes) {
  const uint32_t a = (uint32_t)p & ~31u, e = ((uint32_t)p + bytes + 31u) & ~31u;
  SCB_CleanDCache_by_Addr((uint32_t*)a, (int32_t)(e - a));
}

static inline void dmapTimerStop() { DMAP_TIM->CR1 = 0; DMAP_TIM->DIER = 0; }

// Loads the timer for a frame: free-running upcounter, auto-reload ARR, compare 1 / compare 2, no outputs, no preload (so no
// update event is needed to load anything). CNT is set to ARR, so the first update event (the first raise) comes ONE count
// after CR1.CEN is set.
static inline void dmapTimerLoad() {
  DMAP_TIM->CR1 = 0; DMAP_TIM->DIER = 0;
  DMAP_TIM->PSC = 0; DMAP_TIM->ARR = dmapArr;
  DMAP_TIM->CCMR1 = 0; DMAP_TIM->CCER = 0;                       // compare channels 1, 2 frozen, pins not driven
  DMAP_TIM->CCR1 = dmapC1; DMAP_TIM->CCR2 = dmapC2;
  DMAP_TIM->CNT = dmapArr; DMAP_TIM->SR = 0;
}
static inline void dmapTimerGo() {
  DMAP_TIM->DIER = TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE;
  DMAP_TIM->CR1 = TIM_CR1_CEN;
}

// Call once from setup() after the DWT cycle counter runs. Enables DMA1 and the timer, measures the timer clock, derives
// ARR / CCR1 / CCR2. Leaves dmapReady false (reason in dmapWhy) if the timer or a needed DMA stream is already in use or the
// timer clock looks wrong.
static void dmapInit(uint32_t cyclesPerUs) {
  dmapCpm = cyclesPerUs;
  dmapReady = false;
  dmapWhy = 1;
  if (DMAP_TIM->CR1 & TIM_CR1_CEN) return;                       // the core uses this timer: do not touch
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
#if DMAP_TIMER_SEL == 3
  RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
#else
  RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
#endif
  (void)RCC->AHB1ENR;
  dmapWhy = 4;
  for (int s = 0; s < 3; s++) if (dmapStream(DMAP_FIRST_STREAM + s)->CR & DMA_SxCR_EN) return;   // somebody else's transfer is running
  dmapWhy = 2;
  dmapTimerStop();
  DMAP_TIM->PSC = 0; DMAP_TIM->ARR = 0xFFFF; DMAP_TIM->CNT = 0; DMAP_TIM->EGR = TIM_EGR_UG; DMAP_TIM->SR = 0;
  DMAP_TIM->CR1 = TIM_CR1_CEN;
  const uint32_t c0 = DMAP_TIM->CNT, d0 = DWT->CYCCNT;
  while ((DWT->CYCCNT - d0) < 50u * dmapCpm) { }
  const uint32_t c1 = DMAP_TIM->CNT, d1 = DWT->CYCCNT;
  DMAP_TIM->CR1 = 0;
  const uint32_t dc = (c1 - c0) & 0xFFFFu, dd = d1 - d0;
  const uint32_t hz = dd ? (uint32_t)(((uint64_t)dc * dmapCpm * 1000000ull) / dd) : 0;
  if (hz < 20000000u || hz > 1000000000u) return;                // not a sane timer clock
  dmapTiming(hz, dmapArr, dmapC1, dmapC2);
  dmapWhy = 3;
  if (dmapArr > 0xFFFEu || dmapC1 >= dmapC2 || dmapC2 >= dmapArr) return;      // does not fit a 16-bit timer / nonsense
  dmapWhy = 0;
  dmapReady = true;
}

static void dmapStreamCfg(int n, uint32_t req, uint32_t par, uint32_t mar, uint32_t count, uint32_t cr) {
  DMA_Stream_TypeDef* S = dmapStream(n);
  S->CR &= ~DMA_SxCR_EN;
  const uint32_t t = DWT->CYCCNT;
  while ((S->CR & DMA_SxCR_EN) && (DWT->CYCCNT - t) < 1000u * dmapCpm) { }
  dmapFlagsClear(n, 0x3Du);
  dmapMux(n)->CCR = req;                                         // DMAMUX1 channel n serves DMA1 stream n
  S->PAR  = par;
  S->M0AR = mar;
  S->NDTR = count;
  S->FCR  = 0;                                                   // direct mode, no FIFO
  S->CR   = cr;                                                  // not enabled yet
}

// memory -> BSRR, 32 bit, normal mode, very high priority. Data stream increments through the frame buffer.
#define DMAP_CR_BASE  (DMA_SxCR_DIR_0 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PL)

// memory -> BSRR, 32 bit, normal mode, very high priority. Data stream increments through the frame buffer.
#define DMAP_CR_BASE  (DMA_SxCR_DIR_0 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PL)

// Non-blocking frame start. Returns true if the frame was armed (then call dmapPoll() until it stops returning DMAP_PENDING),
// false if nothing was started (not ready, busy, frame too long, buffer not DMA-reachable: see dmapWhy / dmapPhase).
// The caller has built the plan and the frame buffer (dmapTranspose) and decides about interrupts. The pins go low now; the
// first bit starts after cycResetBefore cycles (counted from this call) at the first dmapPoll() after that.
static bool dmapBegin(const DmapPlan& P, const uint32_t* frameBuf, uint32_t cycResetBefore, uint32_t cycResetAfter) {
  if (!dmapReady || dmapPhase != DMAP_PH_IDLE || P.nCh == 0 || P.nBytes == 0 || P.nBytes > DMAP_MAX_BYTES || DMAP_FIRST_STREAM + 3 > 8) return false;
  const uint32_t nBits = P.nBytes * 8u;
  if (!dmapAddrOk(frameBuf, nBits * 4u) || !dmapAddrOk(dmapConst, sizeof(dmapConst))) { dmapWhy = 5; return false; }
  const int nA = DMAP_FIRST_STREAM, nB = DMAP_FIRST_STREAM + 1, nC = DMAP_FIRST_STREAM + 2;
  const uint32_t par = (uint32_t)P.bsrr;

  dmapBsrr = P.bsrr; dmapClr = P.clrAll;
  dmapConst[0] = P.setAll; dmapConst[1] = P.clrAll;
  dmapCacheClean(dmapConst, sizeof(dmapConst));
  dmapCacheClean(frameBuf, nBits * 4u);                          // M7: the DMA reads RAM, not the cache
  *P.bsrr = P.clrAll;                                            // all low: the reset gap starts now
  dmapTs = DWT->CYCCNT;
  dmapGapCyc = cycResetBefore; dmapTailCyc = cycResetAfter;
  dmapTmoCyc = ((nBits * (uint32_t)DMAP_PERIOD_NS) / 1000u + 2000u) * dmapCpm;      // expected frame time + 2 ms = stall limit
  dmapTimerStop();
  dmapStreamCfg(nA, DMAP_REQ_UP,  par, dmapAddrOf(&dmapConst[0]), nBits, DMAP_CR_BASE);                    // set-all, fixed source
  dmapStreamCfg(nB, DMAP_REQ_CC1, par, dmapAddrOf(frameBuf),      nBits, DMAP_CR_BASE | DMA_SxCR_MINC);     // data words
  dmapStreamCfg(nC, DMAP_REQ_CC2, par, dmapAddrOf(&dmapConst[1]), nBits, DMAP_CR_BASE);                    // clear-all, fixed source
  dmapStream(nA)->CR |= DMA_SxCR_EN;
  dmapStream(nB)->CR |= DMA_SxCR_EN;
  dmapStream(nC)->CR |= DMA_SxCR_EN;
  dmapTimerLoad();
  dmapPhase = DMAP_PH_GAP;
  return true;
}

static inline bool dmapGapDone() { return (DWT->CYCCNT - dmapTs) >= dmapGapCyc; }
// Starts the timer now (phase GAP -> RUN). dmapPoll() does this by itself once the gap is over; a caller that must be ready
// (e.g. a pin checker) can wait for dmapGapDone() and then call this itself.
static inline void dmapKick() {
  if (dmapPhase != DMAP_PH_GAP) return;
  dmapTimerGo();                                                 // first raise one timer count later
  dmapTw = DWT->CYCCNT;
  dmapPhase = DMAP_PH_RUN;
}

// Advances the frame in progress. Returns DMAP_PENDING while it is still going, then DMAP_SENT or DMAP_FAILED once
// (afterwards the result of the last frame). Cheap: call it on every pass of loop().
static int dmapPoll() {
  const int nA = DMAP_FIRST_STREAM, nC = DMAP_FIRST_STREAM + 2;
  if (dmapPhase == DMAP_PH_GAP) { if (dmapGapDone()) dmapKick(); return DMAP_PENDING; }
  if (dmapPhase == DMAP_PH_RUN) {
    const bool done = (dmapFlagsGet(nC) & DMAP_FLAG_TC) != 0;
    if (!done && (DWT->CYCCNT - dmapTw) <= dmapTmoCyc) return DMAP_PENDING;
    bool ok = done;                                              // timed out = stall
    dmapTimerStop();
    for (int n = nA; n <= nC; n++) {
      const uint32_t f = dmapFlagsGet(n);
      if (f & (DMAP_FLAG_TE | DMAP_FLAG_DME)) ok = false;
      if (!(f & DMAP_FLAG_TC)) ok = false;                       // every stream must have done all its transfers
      dmapStream(n)->CR &= ~DMA_SxCR_EN;
      dmapFlagsClear(n, 0x3Du);
    }
    *dmapBsrr = dmapClr;                                         // all LED pins low
    if (!ok) { dmapPhase = DMAP_PH_IDLE; dmapLast = DMAP_FAILED; return DMAP_FAILED; }
    dmapTs = DWT->CYCCNT;                                        // reuse as the start of the tail gap
    dmapPhase = DMAP_PH_TAIL;
    return DMAP_PENDING;
  }
  if (dmapPhase == DMAP_PH_TAIL) {
    if ((DWT->CYCCNT - dmapTs) < dmapTailCyc) return DMAP_PENDING;   // reset / latch
    dmapPhase = DMAP_PH_IDLE; dmapLast = DMAP_SENT;
    return DMAP_SENT;
  }
  return dmapLast;
}
#endif  // DMAP_HOST_SIM

#endif
