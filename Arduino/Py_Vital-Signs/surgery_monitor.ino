/**
 * @file    surgery_monitor.ino
 * PROJECT: Vital-signs monitor — small rodents (rat / mouse)
 * @version 1.0
 * @author  Flávio Mourão — Mar, 2026
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * DESCRIPTION
 * ═══════════════════════════════════════════════════════════════════════════════
 * Real-time, non-invasive physiological monitoring during prolonged experimental
 * surgeries, stereotaxic procedures, and deep anaesthesia protocols in rodents.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * HARDWARE SCALABILITY
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * [ PORTABLE — UNO / NANO ]
 *   MCU        : ATmega328P  |  16 MHz  |  2 KB SRAM
 *   Parameters : Respiratory Rate (piezo belt) + Core Temperature (NTC)
 *   Limitation : Insufficient SRAM for MAX30102 buffers (800 B needed vs ~348 B
 *                available after static allocations). Do NOT enable MAX30102 here.
 *
 * [ FULL MULTIPARAMETRIC — MEGA 2560 ]
 *   MCU        : ATmega2560  |  16 MHz  |  8 KB SRAM
 *   Parameters : HR + SpO2 (MAX30102) + Respiratory Rate + Core Temperature
 *   SRAM usage : ~800 B buffers + ~900 B firmware = ~1700 B / 8192 B (21%)
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * SYSTEM ARCHITECTURE
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * Two execution layers, strictly decoupled:
 *
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │  INTERRUPT LAYER  (hard real-time)                                       │
 * │                                                                          │
 * │  ADC_vect  (~9.6 kHz, free-running)                                      │
 * │    4-state MUX sequencer alternates between A0 (piezo) and A1 (NTC).     │
 * │    States 1 and 3 discard the conversion in flight during MUX switching  │
 * │    (ATmega datasheet requirement: one dummy conversion after ADMUX        │
 * │    change before result is valid).                                        │
 * │    Effective rates:  piezo → ~9.6 kHz raw,  NTC → ~10 Hz                 │
 * │                                                                          │
 * │  TIMER1_COMPA_vect  (100 Hz, CTC mode, prescaler 64, OCR1A = 2499)       │
 * │    Decimates piezo from 9.6 kHz → 100 Hz via N=4 moving average.         │
 * │    Runs hysteresis peak detector for respiratory rate.                   │
 * │    Maintains RR ring buffer (N=4 intervals, power-of-2 for shift-divide).│
 * └──────────────────────────────────────────────────────────────────────────┘
 *
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │  MAIN LOOP LAYER  (cooperative, millis()-based scheduling)               │
 * │                                                                          │
 * │  Task 1 — Threshold calibration    every 2 s                             │
 * │  Task 2 — Temperature conversion   every 1 s                             │
 * │  Task 3 — MAX30102 HR + SpO2       every loop() call (non-blocking)      │
 * │  Task 4 — OLED display             ~30 FPS (33 ms)                       │
 * │  Task 5 — ESP8266 vital signs stream  100 Hz via Serial3                  │
 * │  Task 6 — Python USB stream + config  100 Hz out / on-demand in           │
 * └──────────────────────────────────────────────────────────────────────────┘
 *
 * SHARED VARIABLE PROTOCOL
 *   All variables written by an ISR and read by the main loop are declared
 *   volatile. Multi-byte reads in the main loop use noInterrupts()/interrupts()
 *   critical sections to prevent torn reads (AVR has no atomic 16/32-bit ops).
 *
 * TIMING DEPENDENCY
 *   millis() is driven by TIMER0_OVF_vect. TIMER1_COMPA_vect masks global
 *   interrupts during execution (~10-15 us). This is well below the 1 ms
 *   TIMER0 period, so millis() drift is negligible. If TIMER1_COMPA_vect
 *   execution ever exceeds ~900 us, permanent millis() drift will occur.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * DISPLAY LAYOUT  (SSD1306, 128x32 px, I2C)
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 *  col 0              col 61          col 127
 *  ┌──────────────────┬───────────────────────┐  row 0
 *  │ HR  : 350        │ RR: 85                │  row 9
 *  │ Sat%: 98         ├───────────────────────┤  row 14
 *  │ Temp: 37.2       │  respiratory waveform │  row 31
 *  └──────────────────┴───────────────────────┘
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * TARGET HARDWARE
 * ═══════════════════════════════════════════════════════════════════════════════
 *  MCU     : ATmega2560 — Arduino Mega 2560, 16 MHz, 8 KB SRAM
 *  Display : SSD1306 OLED, 128x32, I2C @ 400 kHz (address 0x3C)
 *  Piezo   : Respiratory belt → A0
 *  NTC     : 100 kOhm rectal thermistor → A1
 *            Circuit: VCC — R_FIXED(100 kOhm) — A1 — NTC — GND
 *  MAX30102: HR + SpO2 optical sensor → I2C (shared bus)  [MEGA ONLY]
 *  ESP8266 : Wi-Fi AP + WebSocket server (RobotDyn on-board co-processor)
 */

#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

// MAX30102 includes and extern declarations live in surgery_monitor_MAX30102.ino.
// The Arduino IDE merges all .ino files in the sketch folder before compiling,
// so no #include is needed here. The extern declarations below give this
// translation unit access to the shared output variables.
extern int heartRate;   // BPM — written by MAX30102 module, read by display task
extern int spO2;        // %   — written by MAX30102 module, read by display task

// Functions defined in surgery_monitor_MAX30102.ino
extern bool     max30102_init();
extern void     max30102_update();
extern uint32_t max30102_getIR();
extern bool     max30102_set_param(const String& key, float value);

// Vital Signs Streaming — ESP8266 bridge
extern void vitalsigns_init();
extern void vitalsigns_update();

// Vital Signs Streaming — Python code USB
extern void py_vital_signs_init();
extern void py_vital_signs_update();

// ═══════════════════════════════════════════════════════════════════════════════
//  DISPLAY DRIVER
// ═══════════════════════════════════════════════════════════════════════════════

// U8G2_R0 = no rotation. U8X8_PIN_NONE = no hardware reset pin used.
// HW_I2C uses the hardware I2C peripheral (pins 20/21 on Mega 2560).

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);


// ═══════════════════════════════════════════════════════════════════════════════
//  HARDWARE PINS AND ADC CHANNEL CODES
// ═══════════════════════════════════════════════════════════════════════════════

// PIN_PIEZO and PIN_NTC are not used directly — the free-running ADC is
// configured via registers (ADMUX/ADCSRA), not through Arduino analogRead().
// Kept here as documentation of the physical pin assignments.
static const uint8_t PIN_PIEZO    = A0;
static const uint8_t PIN_NTC      = A1;

// ATmega MUX codes for ADMUX[3:0] — single-ended, AVcc reference.
static const uint8_t ADC_CH_PIEZO = 0;   // A0
static const uint8_t ADC_CH_NTC   = 1;   // A1


// ═══════════════════════════════════════════════════════════════════════════════
//  FREE-RUNNING ADC — TIMING AND DECIMATION
// ═══════════════════════════════════════════════════════════════════════════════
//
//  ADC clock  = F_CPU / prescaler = 16 MHz / 128 = 125 kHz
//  Conversion = 13 ADC clocks → throughput ≈ 9615 Hz
//
//  Piezo (A0): captured on every State-0 pass at ~9.6 kHz.
//    Effective decimation to 100 Hz is performed by TIMER1_COMPA_vect,
//    not inside ADC_vect. PIEZO_DECIM is kept for documentation only.
//
//  NTC (A1): ADC_vect switches to A1 every NTC_DECIM State-0 passes.
//    NTC_DECIM = 960 → 9615 / 960 ≈ 10 Hz update rate.
//    Two dummy states (1 and 3) are consumed per NTC acquisition cycle
//    to satisfy the ATmega MUX settling requirement.
// ═══════════════════════════════════════════════════════════════════════════════

static const uint16_t PIEZO_DECIM = 96;    // Documentation only — see above.
static const uint16_t NTC_DECIM   = 960;   // State-0 counter threshold → ~10 Hz.


// ═══════════════════════════════════════════════════════════════════════════════
//  RESPIRATORY RATE DETECTION — PHYSIOLOGICAL WINDOW
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Species / state          Rate (rpm)   Period (ms)
//  ─────────────────────────────────────────────────
//  Rat,   anaesthetised     70  –  90    667 – 857
//  Rat,   awake             85  – 115    521 – 706
//  Mouse, anaesthetised     80  – 120    500 – 750
//  Mouse, awake            150  – 230    261 – 400
//
//  DELTA_MIN_MS = 200 ms → 300 rpm upper bound (safety margin above max)
//  DELTA_MAX_MS = 4500 ms → 13 rpm lower bound (covers severe bradypnoea)
//  RR_TIMEOUT_MS = 5000 ms → respRate zeroed after this apnoeic silence
//
//  PRO:  Covers full anaesthetised and awake range for both species.
//  CON:  DELTA_MIN_MS of 200 ms does not protect against motion artefact
//        producing inter-peak intervals just above 200 ms. The hysteresis
//        thresholds provide the primary artefact rejection; DELTA_MIN_MS
//        is a secondary safety gate only.
// ═══════════════════════════════════════════════════════════════════════════════

static const uint16_t DELTA_MIN_MS  = 200;
static const uint16_t DELTA_MAX_MS  = 4500;
static const uint16_t RR_TIMEOUT_MS = 5000;


// ═══════════════════════════════════════════════════════════════════════════════
//  DYNAMIC THRESHOLD CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Every CALIB_INTERVAL_MS the main loop scans the CALIB_WINDOW most recent
//  filtered piezo samples, derives the signal envelope (min / max), and sets
//  inspiration / expiration thresholds as fixed fractions of that envelope.
//
//  This adaptive approach handles:
//    - Changes in belt tightness or sensor contact over long procedures
//    - Animal posture shifts during stereotaxic surgery
//    - Drift in piezo baseline from temperature or moisture
//
//  If the envelope swing is below g_CALIB_MIN_SWING ADC counts, the signal is
//  considered invalid. Thresholds are not updated; respRate reaches zero
//  independently via the RR_TIMEOUT_MS path in TIMER1_COMPA_vect.
// ═══════════════════════════════════════════════════════════════════════════════

static const uint16_t CALIB_INTERVAL_MS = 2000;
static const uint8_t  CALIB_WINDOW      = 50;


// ═══════════════════════════════════════════════════════════════════════════════
//  g_CALIB_MIN_SWING — ADC COUNT TO MILLIVOLT REFERENCE
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Hardware: 10-bit ADC (1024 steps) with AVcc = 5 V reference.
//  Formula:  V (mV) = (ADC / 1024.0) x 5000    Resolution: 1 LSB ≈ 4.88 mV
//
//  g_CALIB_MIN_SWING sets the minimum acceptable signal swing.
//  Tune downward for smaller animals (lower respiratory effort) or if the
//  belt is loosely fitted. Tune upward in electrically noisy environments.
//
//   ADC Value | Equiv. mV | Sensitivity
//  ------------|-----------|--------------------
//      40      |  ~195 mV  | Extreme
//      60      |  ~293 mV  | High
//      80      |  ~391 mV  | High-Mid
//     100      |  ~488 mV  | Mid
//     120      |  ~586 mV  | Default  <- current
//     140      |  ~684 mV  | Mid-Low
//     160      |  ~781 mV  | Low
//     200      |  ~977 mV  | Very Low
// ═══════════════════════════════════════════════════════════════════════════════

int   g_CALIB_MIN_SWING   = 120;   // Mutable — updated by monitor_set_param()


// ═══════════════════════════════════════════════════════════════════════════════
//  HYSTERESIS THRESHOLDS — SOFTWARE SCHMITT TRIGGER
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Thresholds are expressed as fractions of the dynamic envelope
//  (envelopeMax - envelopeMin), recomputed every CALIB_INTERVAL_MS.
//
//  g_THRESH_INSP_FRAC = 0.65 (65%)
//    Signal must rise above 65% of envelope to register inspiration.
//    Rejects sub-threshold mechanical noise and baseline oscillations.
//
//  g_THRESH_EXP_FRAC = 0.45 (45%)
//    Signal must fall below 45% of envelope to reset the detector.
//    The 20% hysteresis band prevents rapid re-triggering on the signal
//    plateau near the inspiration peak.
//
//  PRO:  Adapts to signal amplitude changes without manual recalibration.
//  CON:  A 20% dead-zone may miss shallow breaths in deeply anaesthetised
//        animals where tidal volume is significantly reduced. If respRate
//        drops to zero in a clearly breathing animal, lower g_THRESH_INSP_FRAC
//        toward 0.55 and g_THRESH_EXP_FRAC toward 0.35.
// ═══════════════════════════════════════════════════════════════════════════════

float g_THRESH_INSP_FRAC  = 0.65f;
float g_THRESH_EXP_FRAC   = 0.45f;


// ═══════════════════════════════════════════════════════════════════════════════
//  TEMPERATURE — NTC 100 kOhm THERMISTOR (Beta equation)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Circuit: VCC — R_FIXED — A1 — NTC — GND
//  ADC measures voltage at the junction: V = VCC x NTC / (R_FIXED + NTC)
//  Solving for NTC resistance: R_NTC = R_FIXED x ADC / (1023 - ADC)
//
//  Temperature from Beta equation:
//    1/T = 1/T0 + (1/beta) x ln(R_NTC / R_NOMINAL)
//
//  Valid within +/-50 C of T0 (25 C). Over the clinical range 33-42 C
//  the approximation error is < 0.1 C.
//
//  PROBE: Rectal NTC, 100 kOhm at 25 C.
//  Verify NTC_BETA against your specific probe datasheet. A +/-50 K error
//  in beta produces approximately +/-0.3 C error at 37 C.
// ═══════════════════════════════════════════════════════════════════════════════

static const float NTC_R_FIXED   = 100000.0f;   // Series resistor (Ohm)
static const float NTC_R_NOMINAL = 100000.0f;   // NTC resistance at T0 (Ohm)
static const float NTC_BETA      = 3950.0f;     // Beta coefficient (K)
static const float NTC_T0_K      = 298.15f;     // Reference temperature (K = 25 C)

float g_TEMP_MIN_C    = 30.0f;   // Below: probe disconnected or shorted (see EEPROM)
float g_TEMP_MAX_C    = 45.0f;   // Above: probe disconnected or open circuit


// ═══════════════════════════════════════════════════════════════════════════════
//  MOVING-AVERAGE FILTER — PIEZO  (used inside TIMER1_COMPA_vect only)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  N=4, power-of-2 constraint: the sum-to-average step becomes a right
//  bit-shift (sum >> 2) instead of an integer division, eliminating the
//  only division inside the ISR.
//
//  At 100 Hz input, N=4 gives a simple FIR low-pass with -3 dB at ~18 Hz.
//  This is well above the respiratory frequencies of interest (0.5-4 Hz)
//  and provides adequate noise rejection without introducing phase delay
//  that would shift detected peak times.
// ═══════════════════════════════════════════════════════════════════════════════

static const uint8_t N_ADC_SAMPLES = 4;   // Must be a power of 2.
static const uint8_t N_ADC_SHIFT   = 2;   // log2(N_ADC_SAMPLES)


// ═══════════════════════════════════════════════════════════════════════════════
//  RR AVERAGING — BREATH INTERVAL RING BUFFER  (TIMER1_COMPA_vect only)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  N=4 intervals → running mean of last 4 breath cycles.
//  At 85 rpm (anaesthetised rat), N=4 covers ~2.8 s of history.
//  At 80 rpm (anaesthetised mouse), N=4 covers ~3.0 s of history.
//
//  PRO:  Responds to rate changes within 4 breaths (~3 s).
//  CON:  A single motion artefact triggering a false peak contaminates
//        the average for up to 4 real cycles. The DELTA_MIN/MAX gates
//        reject most artefacts before they reach the buffer.
// ═══════════════════════════════════════════════════════════════════════════════

static const uint8_t N_RR_SAMPLES = 4;    // Must be a power of 2.
static const uint8_t N_RR_SHIFT   = 2;    // log2(N_RR_SAMPLES)


// ═══════════════════════════════════════════════════════════════════════════════
//  WAVEFORM DISPLAY GEOMETRY
// ═══════════════════════════════════════════════════════════════════════════════
//
//  The right half of the 128x32 OLED (columns 64-127) displays a scrolling
//  respiratory PPG. The trace advances one pixel per display frame (~30 FPS).
//  WAVE_ERASER_W = 4 pixels creates a visible gap ahead of the trace cursor,
//  replicating the travelling-cursor style of ICU bedside monitors.
// ═══════════════════════════════════════════════════════════════════════════════

static const uint8_t WAVE_X_START  = 64;
static const uint8_t WAVE_X_END    = 127;
static const uint8_t WAVE_Y_TOP    = 14;   // First row below RR label.
static const uint8_t WAVE_Y_BOTTOM = 31;   // Last row of display.
static const uint8_t WAVE_ERASER_W = 4;    // Columns erased ahead of trace.


// ═══════════════════════════════════════════════════════════════════════════════
//  TASK SCHEDULING
// ═══════════════════════════════════════════════════════════════════════════════

static const uint8_t  INTERVAL_DISPLAY_MS = 33;    // ~30 FPS
static const uint16_t INTERVAL_SLOW_MS    = 1000;  // Temperature (1 Hz)
static const uint16_t BLINK_INTERVAL_MS   = 400;   // RR "--" blink when no signal


// ═══════════════════════════════════════════════════════════════════════════════
//  SHARED STATE — volatile
//  Written by ISR context, read by main loop. All multi-byte accesses in
//  the main loop must be protected with noInterrupts()/interrupts().
// ═══════════════════════════════════════════════════════════════════════════════

volatile int  piezoRaw = 0;       // Latest A0 raw sample from ADC_vect (~9.6 kHz)
volatile int  ntcRaw   = 0;       // Latest A1 raw sample from ADC_vect (~10 Hz)
volatile bool ntcReady = false;   // Handshake: set by ADC_vect, cleared by main loop

volatile int           adcDisplay   = 0;      // Filtered piezo for waveform display
volatile int           respRate     = 0;      // Respiratory rate (rpm), 0 = no signal
volatile bool          isInspiring  = false;  // Hysteresis state: true = above THRESH_INSP
volatile unsigned long lastPeakTime = 0;      // millis() at last detected inspiration peak

volatile int     calibBuf[CALIB_WINDOW];  // Circular buffer of filtered samples
volatile uint8_t calibIdx  = 0;
volatile bool    calibFull = false;

// Dynamic thresholds — initial values represent a mid-range 5 V ADC signal.
// Safe defaults before the first calibration pass at 2 s post-startup.
volatile int threshInsp = 580;
volatile int threshExp  = 530;


// ═══════════════════════════════════════════════════════════════════════════════
//  NON-SHARED STATE — TIMER1_COMPA_vect exclusive
//
//  adcBuf and rrBuf are written AND read exclusively inside TIMER1_COMPA_vect.
//  Not volatile. File-scope static placement makes sizes visible to the linker
//  map, aiding SRAM budget verification.
// ═══════════════════════════════════════════════════════════════════════════════

static int     adcBuf[N_ADC_SAMPLES];
static uint8_t adcIdx  = 0;
static bool    adcFull = false;

static uint16_t rrBuf[N_RR_SAMPLES];
static uint8_t  rrIdx   = 0;
static uint8_t  rrCount = 0;

static int   envelopeMin = 400;   // Current calibrated signal minimum
static int   envelopeMax = 700;   // Current calibrated signal maximum

float tempCelsius = 0.0f;

static uint8_t waveX          = WAVE_X_START;
static uint8_t waveYPrev      = 16;
static bool    waveFirstPixel = true;
static bool    blinkVisible   = false;

static unsigned long timerDisplay  = 0;
static unsigned long timerSlow     = 0;
static unsigned long lastCalibTime = 0;
static unsigned long lastBlinkTime = 0;


// ═══════════════════════════════════════════════════════════════════════════════
//  ISR — ADC CONVERSION COMPLETE  (~9.6 kHz, free-running)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  4-state MUX sequencer. States 1 and 3 are mandatory discard states.
//  After ADMUX is changed, the conversion already in flight was started with
//  the old MUX setting and its result is invalid (ATmega datasheet, section
//  "Changing Channel or Reference Selection").
//
//  State machine:
//    0 → 0  : normal piezo capture (most passes)
//    0 → 1  : piezo captured, MUX switched to A1, discard next result
//    1 → 2  : discarded, A1 conversion now valid
//    2 → 3  : NTC captured, MUX switched back to A0, discard next result
//    3 → 0  : discarded, back to normal piezo capture
// ═══════════════════════════════════════════════════════════════════════════════

ISR(ADC_vect)
{
    static uint16_t ntcCounter = 0;
    static uint8_t  muxState   = 0;

    const int result = ADC;   // Latch immediately — next conversion may be starting.

    switch (muxState) {

        case 0:
            piezoRaw = result;
            if (++ntcCounter >= NTC_DECIM) {
                ntcCounter = 0;
                ADMUX = (ADMUX & 0xF0) | ADC_CH_NTC;
                muxState = 1;
            }
            break;

        case 1:
            muxState = 2;
            break;

        case 2:
            ntcRaw   = result;
            ntcReady = true;
            ADMUX = (ADMUX & 0xF0) | ADC_CH_PIEZO;
            muxState = 3;
            break;

        case 3:
            muxState = 0;
            break;
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  ISR — TIMER1 OUTPUT COMPARE A  (100 Hz, CTC mode)
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Timer configuration (set in setup()):
//    Prescaler 64, OCR1A = 2499
//    f = F_CPU / (prescaler x (OCR1A + 1)) = 16e6 / (64 x 2500) = 100 Hz
//
//  Tasks in sequence:
//    1. Moving-average filter: decimates piezo from ~9.6 kHz to 100 Hz
//    2. Calibration window: feeds filtered sample into circular buffer
//    3. Hysteresis peak detection: detects inspiration onset, computes RR
//    4. Apnoea timeout: zeros respRate after RR_TIMEOUT_MS of silence
//
//  Execution budget: ~10-15 us at 16 MHz.
//  Hard limit: ~900 us (above this, TIMER0 overflows are missed → millis() drifts).
// ═══════════════════════════════════════════════════════════════════════════════

ISR(TIMER1_COMPA_vect)
{
    const unsigned long nowISR = millis();

    // 1. Moving-average filter
    adcBuf[adcIdx] = piezoRaw;
    adcIdx = (adcIdx + 1) % N_ADC_SAMPLES;
    if (!adcFull && adcIdx == 0) adcFull = true;

    const uint8_t count = adcFull ? N_ADC_SAMPLES : max((int)adcIdx, 1);
    int32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) sum += adcBuf[i];
    adcDisplay = adcFull ? (int)(sum >> N_ADC_SHIFT) : (int)(sum / count);

    // 2. Calibration window
    calibBuf[calibIdx] = adcDisplay;
    calibIdx = (calibIdx + 1) % CALIB_WINDOW;
    if (!calibFull && calibIdx == 0) calibFull = true;

    // 3. Hysteresis peak detection (software Schmitt trigger)
    if (adcDisplay > threshInsp && !isInspiring) {
        isInspiring = true;
        const unsigned long deltaT = nowISR - lastPeakTime;
        lastPeakTime = nowISR;

        if (deltaT > DELTA_MIN_MS && deltaT < DELTA_MAX_MS) {
            rrBuf[rrIdx] = (uint16_t)deltaT;
            rrIdx = (rrIdx + 1) % N_RR_SAMPLES;
            if (rrCount < N_RR_SAMPLES) rrCount++;

            uint32_t rrSum = 0;
            for (uint8_t i = 0; i < rrCount; i++) rrSum += rrBuf[i];
            const uint32_t avgDelta = (rrCount == N_RR_SAMPLES)
                                      ? (rrSum >> N_RR_SHIFT)
                                      : (rrSum / rrCount);
            respRate = (int)(60000UL / avgDelta);
        }
    }
    else if (adcDisplay < threshExp) {
        isInspiring = false;
    }

    // 4. Apnoea timeout
    if (nowISR - lastPeakTime > RR_TIMEOUT_MS) {
        respRate = 0;
        rrCount  = 0;   // Clear stale intervals so average is unbiased on recovery.
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  recomputeThresholds()
//
//  Scans calibBuf, derives signal envelope, updates hysteresis thresholds.
//  Called every CALIB_INTERVAL_MS from the main loop.
//
//  Each calibBuf element is read under its own noInterrupts() section to
//  minimise total interrupt latency per call (rather than one large section).
//  Tradeoff: calibBuf may be partially updated by the ISR during the scan.
//  At 100 Hz and CALIB_WINDOW=50, this introduces at most one stale sample
//  in 50 — negligible for envelope estimation.
// ═══════════════════════════════════════════════════════════════════════════════

static void recomputeThresholds()
{
    noInterrupts();
    const bool    cFull = calibFull;
    const uint8_t cIdx  = calibIdx;
    interrupts();

    if (!cFull && cIdx == 0) return;

    const uint8_t n = cFull ? CALIB_WINDOW : cIdx;
    int vMin = 1023, vMax = 0;

    for (uint8_t i = 0; i < n; i++) {
        noInterrupts();
        const int val = calibBuf[i];
        interrupts();
        if (val < vMin) vMin = val;
        if (val > vMax) vMax = val;
    }

    if ((vMax - vMin) < g_CALIB_MIN_SWING) {
        // Envelope too narrow: sensor disconnected or animal not breathing.
        // Set display envelope to narrow centred range → stable flat waveform.
        envelopeMin = 400;
        envelopeMax = 600;
        return;
    }

    envelopeMin = vMin;
    envelopeMax = vMax;

    const int newInsp = (int)(vMin + g_THRESH_INSP_FRAC * (vMax - vMin));
    const int newExp  = (int)(vMin + g_THRESH_EXP_FRAC  * (vMax - vMin));

    noInterrupts();
    threshInsp = newInsp;
    threshExp  = newExp;
    interrupts();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  showSplashScreen()
// ═══════════════════════════════════════════════════════════════════════════════

static void showSplashScreen()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB10_tf);
    u8g2.setCursor(8, 14);
    u8g2.print(F("Surgery Monitor"));
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(8, 26);
    u8g2.print(F("Initializing..."));
    u8g2.sendBuffer();
    delay(3000);
}


// ════════════════════════════════════════════════════════════════════════════
//  monitor_set_param()
//  Called from Py_Vital-Signs.ino for piezo and temperature parameters.
//  Returns true if key recognised; false otherwise (caller tries next module).
// ════════════════════════════════════════════════════════════════════════════

extern void eeprom_save_all();

bool monitor_set_param(const String& key, float value)
{
    if      (key == "CALIB_SWING") { g_CALIB_MIN_SWING  = (int)constrain(value, 20.0f, 400.0f); }
    else if (key == "THRESH_INSP") {
        g_THRESH_INSP_FRAC = constrain(value, 0.40f, 0.85f);
        if (g_THRESH_INSP_FRAC <= g_THRESH_EXP_FRAC + 0.05f)
            g_THRESH_INSP_FRAC = g_THRESH_EXP_FRAC + 0.05f;
    }
    else if (key == "THRESH_EXP")  {
        g_THRESH_EXP_FRAC  = constrain(value, 0.20f, 0.70f);
        if (g_THRESH_EXP_FRAC >= g_THRESH_INSP_FRAC - 0.05f)
            g_THRESH_EXP_FRAC = g_THRESH_INSP_FRAC - 0.05f;
    }
    else if (key == "TEMP_MIN")    { g_TEMP_MIN_C = constrain(value, 20.0f, 35.0f); }
    else if (key == "TEMP_MAX")    { g_TEMP_MAX_C = constrain(value, 35.0f, 50.0f);
                                     if (g_TEMP_MAX_C <= g_TEMP_MIN_C) g_TEMP_MAX_C = g_TEMP_MIN_C + 1.0f; }
    else { return false; }
    eeprom_save_all();
    return true;
}



// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(115200);
    Wire.setClock(400000UL);   // 400 kHz I2C — required for SSD1306 and MAX30102
    u8g2.begin();
    showSplashScreen();

    lastPeakTime  = millis();
    lastCalibTime = millis();

    // Disable digital input buffers on A0 and A1.
    // These buffers toggle with the analog signal and inject switching current
    // into the analog supply, adding ~1-2 LSB noise. Disabling improves SNR.
    DIDR0 |= (1 << ADC0D) | (1 << ADC1D);

    // ADMUX: AVcc reference (REFS0=1), right-adjusted (ADLAR=0), start on A0.
    ADMUX = (1 << REFS0) | ADC_CH_PIEZO;

    // ADCSRA: ADEN=enable, ADSC=start, ADATE=free-running, ADIE=interrupt,
    // prescaler=128 → ADC clock = 125 kHz (ATmega spec: 50-200 kHz for full res).
    ADCSRA = (1 << ADEN)  |
             (1 << ADSC)  |
             (1 << ADATE) |
             (1 << ADIE)  |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    ADCSRB = 0;   // Auto-trigger source: free-running mode.

    // Timer1 CTC: f = 16e6 / (64 x 2500) = 100 Hz
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A  = 2499;
    TCCR1B |= (1 << WGM12);               // CTC mode (OCR1A as TOP).
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler 64.
    TIMSK1 |= (1 << OCIE1A);              // Enable compare-match A interrupt.
    interrupts();

    u8g2.clearBuffer();

    // MAX30102 init [MEGA ONLY] — defined in surgery_monitor_MAX30102.ino.
    // Halt on failure: sensor required for full multiparametric operation.
    if (!max30102_init()) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setCursor(0, 12);
        u8g2.print(F("MAX30102 not found"));
        u8g2.setCursor(0, 26);
        u8g2.print(F("Check wiring / I2C"));
        u8g2.sendBuffer();
        while (true);
    }

    vitalsigns_init();      // ESP8266 WebSocket stream (Serial3)
    py_vital_signs_init();  // Python USB stream + bidirectional command parser (Serial0)
}


// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop()
{
    const unsigned long now = millis();

    // TASK 1 — Threshold calibration (every 2 s)
    if (now - lastCalibTime >= CALIB_INTERVAL_MS) {
        lastCalibTime += CALIB_INTERVAL_MS;
        recomputeThresholds();
    }

    // TASK 2 — Temperature conversion (1 Hz)
    if (now - timerSlow >= INTERVAL_SLOW_MS) {
        timerSlow += INTERVAL_SLOW_MS;

        // Atomic read of NTC sample. ntcReady is cleared here; ADC_vect will
        // set it again on the next NTC acquisition cycle (~100 ms later).
        bool ntcAvail;
        int  rawNTC;
        noInterrupts();
        ntcAvail = ntcReady;
        rawNTC   = ntcRaw;
        ntcReady = false;
        interrupts();

        if (ntcAvail) {
            // Guard against ADC rail values (open or shorted probe).
            // rawNTC ≈ 0    → NTC shorted (R_NTC ≈ 0, A1 pulled to GND)
            // rawNTC ≈ 1023 → NTC open    (R_NTC → inf, A1 pulled to VCC)
            if (rawNTC > 5 && rawNTC < 1018) {
                const float rNTC  = NTC_R_FIXED
                                    * ((float)rawNTC / (1023.0f - (float)rawNTC));
                const float tempK = 1.0f / (  (1.0f / NTC_T0_K)
                                             + (1.0f / NTC_BETA)
                                             * log(rNTC / NTC_R_NOMINAL));
                tempCelsius = tempK - 273.15f;
            } else {
                tempCelsius = 0.0f;   // Sentinel: probe open or shorted.
            }
        }
        // If !ntcAvail: no new sample this tick; retain previous value.
    }

    // TASK 3 — MAX30102 HR + SpO2 [MEGA ONLY]
    // Non-blocking. Drains FIFO, runs beat detection, runs SpO2 block processing.
    // Defined in surgery_monitor_MAX30102.ino.
    max30102_update();

    // TASK 4 — OLED display 128x32 (~30 FPS)
    if (now - timerDisplay >= INTERVAL_DISPLAY_MS) {
        timerDisplay += INTERVAL_DISPLAY_MS;

        if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
            lastBlinkTime = now;
            blinkVisible  = !blinkVisible;
        }

        // Atomic snapshot of ISR-written variables.
        // Both read inside one critical section to guarantee a consistent frame.
        noInterrupts();
        const int safeResp = respRate;
        const int safeADC  = adcDisplay;
        interrupts();

        // heartRate and spO2 are written only from max30102_update() in the
        // main loop — no ISR writes them, so no critical section needed.
        const int safeHR   = heartRate;
        const int safeSpo2 = spO2;

        // Selective erase: clear text regions only. Waveform area is managed
        // column-by-column by the rolling eraser below to avoid flicker.
        u8g2.setDrawColor(0);
        u8g2.drawBox(0,  0, 61, 32);
        u8g2.drawBox(61, 0, 67, WAVE_Y_TOP);
        u8g2.setDrawColor(1);

        u8g2.setFont(u8g2_font_6x10_tf);

        // Left column: HR, SpO2, Temperature
        u8g2.setCursor(0, 9);
        u8g2.print(F("HR  : "));
        if (safeHR > 0) u8g2.print(safeHR); else u8g2.print(F("---"));

        u8g2.setCursor(0, 20);
        u8g2.print(F("Sat%: "));
        if (safeSpo2 > 0) u8g2.print(safeSpo2); else u8g2.print(F("---"));

        u8g2.setCursor(0, 31);
        u8g2.print(F("Temp: "));
        if (tempCelsius >= g_TEMP_MIN_C && tempCelsius <= g_TEMP_MAX_C) {
            u8g2.print(tempCelsius, 1);
        } else if (tempCelsius == 0.0f) {
            u8g2.print(F("---"));   // Probe disconnected.
        } else {
            u8g2.print(F("OUT"));   // Outside physiological range.
        }

        // Right column: RR
        u8g2.setCursor(63, 9);
        u8g2.print(F("RR: "));
        if (safeResp > 0) {
            u8g2.print(safeResp);
        } else if (blinkVisible) {
            u8g2.print(F("--"));   // Blinks during apnoea or no signal.
        }

        // Respiratory waveform trace
        // Map filtered ADC value to display row using current calibrated envelope.
        // constrain() prevents Y from leaving the waveform area if the signal
        // transiently exceeds the calibrated bounds between calibration passes.
        const int yNow = constrain(
            map(safeADC, envelopeMin, envelopeMax, WAVE_Y_BOTTOM, WAVE_Y_TOP),
            WAVE_Y_TOP, WAVE_Y_BOTTOM
        );

        // Suppress connecting line at sweep wrap to avoid a vertical artefact
        // joining the last Y of the previous sweep to the first Y of the new one.
        if (!waveFirstPixel) {
            u8g2.drawLine(waveX - 1, waveYPrev, waveX, yNow);
        }
        waveFirstPixel = false;

        // Rolling eraser: WAVE_ERASER_W columns cleared ahead of the trace.
        u8g2.setDrawColor(0);
        u8g2.drawBox(waveX + 1, WAVE_Y_TOP,
                     WAVE_ERASER_W, WAVE_Y_BOTTOM - WAVE_Y_TOP + 1);
        u8g2.setDrawColor(1);

        waveYPrev = yNow;
        waveX++;

        if (waveX > WAVE_X_END) {
            waveX          = WAVE_X_START;
            waveFirstPixel = true;
            u8g2.setDrawColor(0);
            u8g2.drawBox(WAVE_X_START, WAVE_Y_TOP,
                         WAVE_ERASER_W + 1, WAVE_Y_BOTTOM - WAVE_Y_TOP + 1);
            u8g2.setDrawColor(1);
        }

        u8g2.sendBuffer();
    }

    // TASK 5 — Vital Signs Streaming (non-blocking, 100 Hz out to Serial3)
    vitalsigns_update();

    // TASK 6 — USB Data Stream (Py_Vital-signs DAQ via Serial0)
    py_vital_signs_update();
}
