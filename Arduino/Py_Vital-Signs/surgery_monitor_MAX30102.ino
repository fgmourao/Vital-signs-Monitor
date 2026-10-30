/**
 * @file    surgery_monitor_MAX30102.ino
 * PROJECT: Vital-signs monitor
 * @version 1.0
 * @author  Flávio Mourão — Mar, 2026
 * @updated Oct, 2026
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * MODULE RESPONSIBILITY
 * ═══════════════════════════════════════════════════════════════════════════════
 * This file owns all MAX30102 interaction: sensor initialisation, FIFO draining,
 * heart rate detection, and SpO2 block processing. It exposes two integer
 * globals (heartRate, spO2) declared extern in surgery_monitor.ino.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * SIGNAL CHAIN ARCHITECTURE
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 *  MAX30102 internal:
 *    Sample rate  : 400 Hz
 *    sampleAverage: 4  →  FIFO delivers 100 Hz (hardware averages 4→1)
 *    LED mode     : 2  →  Red + IR channels simultaneously
 *
 *                         ┌─────────────────────────────────────┐
 *  FIFO (100 Hz) ─────────┤                                     │
 *                         │  PATH 1 — HEART RATE                │
 *                         │  checkForBeat(irRaw) @ 100 Hz       │
 *                         │  → _process_beat()                  │
 *                         │    physiological gate               │
 *                         │    outlier rejection (hrFill >= 4)  │
 *                         │    ring buffer N=6, rolling mean    │
 *                         │    → heartRate (BPM)                │
 *                         │                                     │
 *                         │  PATH 2 — SpO2                      │
 *                         │  Software decimation x1             │
 *                         │  (g_DECIM_RATIO = 1, no extra avg)  │
 *                         │  → irBuffer[100] + redBuffer[100]   │
 *                         │  → maxim_heart_rate_and_oxygen_sat  │
 *                         │    (Sparkfun / Maxim algorithm)     │
 *                         │    sliding window: retain 75,       │
 *                         │    refill 25 → update every 25      │
 *                         │    samples (0.25 s at 100 Hz)       │
 *                         │    → spO2 (%), last valid value     │
 *                         │      held up to SPO2_TIMEOUT_MS     │
 *                         └─────────────────────────────────────┘
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * BEAT TIMING — SAMPLE INDEX, NOT millis()
 * ═══════════════════════════════════════════════════════════════════════════════
 * The MAX30102 produces samples on its own oscillator while loop() may read
 * several of them per call. Time-stamping a burst with one millis() value would
 * quantise beat intervals to the loop period. Beat interval =
 * (samples between beats) x g_samplePeriodMs, where the period starts at the
 * nominal 2.5 ms x sampleAverage and is re-measured every >= 4 s as
 * (elapsed ms) / (samples consumed); windows outside 0.85-1.15 x nominal are
 * ignored. On contact loss (IR < 5000) all HR/SpO2 state is reset. Without
 * contact loss, heartRate is cleared after HR_TIMEOUT_MS (5 s) without an
 * accepted beat and spO2 after SPO2_TIMEOUT_MS (10 s) without a valid block, so
 * a stale value is never shown (e.g. when the AC amplitude leaves the 20-1000
 * gate of checkForBeat and no beat is accepted).
 * Every filter below is per sample and assumes the nominal FIFO rate: a wrong
 * rate (e.g. 16 Hz from a slow loop, or 50 Hz from sampleAverage = 8 left in
 * EEPROM) makes heart-rate and SpO2 results meaningless. The boot message
 * prints avg / adc / LED / pw; check it first when results look wrong.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * KNOWN DESIGN TRADEOFFS — HUMAN VS RODENT
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * [T1] g_DECIM_RATIO = 1 (no software decimation)
 *   The Maxim spo2_algorithm.cpp FIR filter coefficients were derived for
 *   25 Hz input (i.e., 100 Hz FIFO with g_DECIM_RATIO=4). With g_DECIM_RATIO=1
 *   the algorithm receives 100 Hz input — the internal filter cutoff scales
 *   proportionally, passing 4x more low-frequency content into the DC estimate.
 *
 *   HUMAN bench test: SpO2 values physiologically correct (97-99%). The
 *   filter deviation does not produce detectable error at human HR (60-80 BPM).
 *
 *   RODENT (untested): At 300-600 BPM, cardiac frequency (5-10 Hz) approaches
 *   the filter passband at 100 Hz input. AC attenuation by the internal FIR
 *   may reduce measured SpO2 accuracy. If systematic SpO2 underestimation is
 *   observed in rodent trials, reduce sampleAverage to 1 in max30102_init(),
 *   set g_DECIM_RATIO=4, and verify that heartRate detection still functions
 *   (checkForBeat() was calibrated for high-amplitude raw signals).
 *
 * [T2] checkForBeat() — what it really does, and its limits for rodents
 *   Chain (heartRate.cpp): IIR DC estimate (alpha 1/16) subtracted from the
 *   input, 23-tap symmetric FIR low-pass (>>15), rising zero-crossing detector.
 *   A beat is accepted only if the AC peak-to-peak of the previous cycle is
 *   between 21 and 999 counts (gate 20 < pk-pk < 1000).
 *
 *   Frequency response at 100 Hz (computed from the coefficients):
 *     DC estimator high-pass corner ~1.0 Hz
 *     FIR gain relative to DC: 1 Hz 0.96 | 2 Hz 0.86 | 4 Hz 0.53 | 5 Hz 0.36 |
 *                              6 Hz 0.22 | 7 Hz 0.11 | 8 Hz 0.04 | >= 10 Hz 0
 *     -3 dB at ~3 Hz, group delay 110 ms.
 *   Human HR (1-3 Hz) passes well. A rat (300-500 BPM = 5-8 Hz) is attenuated
 *   to 4-36 % and a mouse (250-800 BPM = 4-13 Hz) even more, so the AC may fall
 *   below the gate or the zero-crossings become unreliable. Both ends of the
 *   filter scale with the FIFO rate (sampleAverage 2 -> corners x2, 1 -> x4;
 *   the DC high-pass corner at 400 Hz is ~4 Hz and starts to cut a 5 Hz
 *   cardiac wave), so there is no sampleAverage setting that fits a rat
 *   well. If HR detection fails on rodents, a detector with a band-pass of
 *   ~3.5-12 Hz and an adaptive threshold (alt_hr_detector.m) is the intended
 *   replacement; verify offline with emulate_firmware_hr.m on a rodent record.
 *
 *   Human bench validation (finger, LED 80, ADC 16384, avg 4, 100 Hz): IR DC
 *   ~145000, AC 280-560 counts pk-pk; 95 of 97 reference beats detected, median
 *   HR within 1-3 BPM of a zero-phase offline reference.
 *
 * [T3] adcRange = 16384 required for human finger (current EEPROM/default)
 *   With LED=80 on a human fingertip, adcRange=4096 saturates the ADC
 *   (IR DC reaches 262143 = 18-bit hardware ceiling). adcRange=16384 prevents
 *   saturation. For rodents (thinner tissue, less absorption) adcRange=4096
 *   or even 2048 may be appropriate — verify with IR_raw diagnostic output.
 *
 * [T4] Physiological filter range
 *   g_HR_MIN_BPM (40) and g_HR_MAX_BPM (800) are currently set for human bench
 *   testing; change them in the Python Settings panel (saved to EEPROM) before
 *   animal use:
 *     Rat:   HR_MIN=200, HR_MAX=500
 *     Mouse: HR_MIN=250, HR_MAX=800
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * DEBUG AIDS  (all off by default)
 * ═══════════════════════════════════════════════════════════════════════════════
 *   DEBUG_SERIAL = 1      1 Hz plain-text status line (IR, contact, HR, SpO2)
 *                         and the HR auto-reset message. Text on Serial0 is
 *                         ignored by the Python DAQ but is not valid JSON:
 *                         use only with the Arduino Serial Monitor.
 *   PLOTTER_ENABLE = true raw IR counts for the Arduino Serial Plotter
 *                         (mutually exclusive with the text and the JSON stream).
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * SRAM BUDGET  [MEGA 2560 only — DO NOT enable on Uno/Nano]
 * ═══════════════════════════════════════════════════════════════════════════════
 *   irBuffer[100]  = 400 B
 *   redBuffer[100] = 400 B
 *   Other firmware static data ≈ 900 B
 *   Total ≈ 1700 B / 8192 B available  →  safe
 *   On ATmega328P (2048 B total): the 800 B buffers alone exhaust SRAM.
 */

#include "MAX30105.h"
#include <EEPROM.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  EEPROM PERSISTENCE
// ═══════════════════════════════════════════════════════════════════════════════
//
//  All g_* mutable parameters are saved to EEPROM whenever a value is changed
//  via max30102_set_param() or monitor_set_param(). On power-up / reset, the
//  firmware loads saved values from EEPROM instead of using compile-time defaults.
//
//  Layout (ATmega2560 has 4 KB EEPROM):
//    Addr 0    : magic byte (0xA6) — confirms EEPROM has been written at least once
//    Addr 1–4  : g_HR_MIN_BPM       (float, 4 bytes)
//    Addr 5–8  : g_HR_MAX_BPM       (float, 4 bytes)
//    Addr 9–12 : g_HR_OUTLIER_FRAC  (float, 4 bytes)
//    Addr 13   : g_SPO2_MIN_VALID   (uint8_t, 1 byte)
//    Addr 14   : g_DECIM_RATIO      (uint8_t, 1 byte)
//    Addr 15   : g_LED_BRIGHTNESS   (uint8_t, 1 byte)
//    Addr 16   : g_SAMPLE_AVERAGE   (uint8_t, 1 byte)
//    Addr 17–20: g_ADC_RANGE        (uint32_t, 4 bytes)
//    Addr 21–22: g_PULSE_WIDTH      (uint16_t, 2 bytes)
//    Addr 23–24: g_CALIB_MIN_SWING  (int, 2 bytes on AVR)  [owned by monitor_set_param]
//    Addr 25–28: g_THRESH_INSP_FRAC (float, 4 bytes)
//    Addr 29–32: g_THRESH_EXP_FRAC  (float, 4 bytes)
//    Addr 33–36: g_TEMP_MIN_C       (float, 4 bytes)
//    Addr 37–40: g_TEMP_MAX_C       (float, 4 bytes)
//  Addresses are computed with sizeof() in eeprom_save_all()/eeprom_load_all();
//  any change to the order or types needs a new EEPROM_MAGIC.
//  A wrong value saved here (e.g. sampleAverage = 8) survives power cycles and
//  changes the FIFO rate: always check the boot line "LED | avg | adc | decim | pw".
//
//  EEPROM endurance: 100,000 write cycles per address. With EEPROM.put() using
//  update-only writes (skips unchanged bytes), this is effectively unlimited
//  for laboratory use.
// ═══════════════════════════════════════════════════════════════════════════════

static const uint8_t  EEPROM_MAGIC      = 0xA6;   // bumped when g_PULSE_WIDTH was added to the layout
static const uint16_t EEPROM_ADDR_MAGIC = 0;
static const uint16_t EEPROM_ADDR_DATA  = 1;

// Forward declarations (order-independent of the .ino concatenation): the
// MAX30102 parameters defined further down, and the piezo/temp variables
// owned by surgery_monitor.ino.
extern float    g_HR_MIN_BPM;
extern float    g_HR_MAX_BPM;
extern float    g_HR_OUTLIER_FRAC;
extern uint8_t  g_SPO2_MIN_VALID;
extern uint8_t  g_DECIM_RATIO;
extern uint8_t  g_LED_BRIGHTNESS;
extern uint8_t  g_SAMPLE_AVERAGE;
extern uint32_t g_ADC_RANGE;
extern uint16_t g_PULSE_WIDTH;
extern int   g_CALIB_MIN_SWING;
extern float g_THRESH_INSP_FRAC;
extern float g_THRESH_EXP_FRAC;
extern float g_TEMP_MIN_C;
extern float g_TEMP_MAX_C;

void eeprom_save_all()
{
    // EEPROM.put() uses update-only writes — only writes bytes that changed,
    // preserving EEPROM endurance. Safe to call on every parameter change.
    EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    uint16_t addr = EEPROM_ADDR_DATA;
    EEPROM.put(addr, g_HR_MIN_BPM);      addr += sizeof(g_HR_MIN_BPM);
    EEPROM.put(addr, g_HR_MAX_BPM);      addr += sizeof(g_HR_MAX_BPM);
    EEPROM.put(addr, g_HR_OUTLIER_FRAC); addr += sizeof(g_HR_OUTLIER_FRAC);
    EEPROM.put(addr, g_SPO2_MIN_VALID);  addr += sizeof(g_SPO2_MIN_VALID);
    EEPROM.put(addr, g_DECIM_RATIO);     addr += sizeof(g_DECIM_RATIO);
    EEPROM.put(addr, g_LED_BRIGHTNESS);  addr += sizeof(g_LED_BRIGHTNESS);
    EEPROM.put(addr, g_SAMPLE_AVERAGE);  addr += sizeof(g_SAMPLE_AVERAGE);
    EEPROM.put(addr, g_ADC_RANGE);       addr += sizeof(g_ADC_RANGE);
    EEPROM.put(addr, g_PULSE_WIDTH);     addr += sizeof(g_PULSE_WIDTH);
    EEPROM.put(addr, g_CALIB_MIN_SWING); addr += sizeof(g_CALIB_MIN_SWING);
    EEPROM.put(addr, g_THRESH_INSP_FRAC);addr += sizeof(g_THRESH_INSP_FRAC);
    EEPROM.put(addr, g_THRESH_EXP_FRAC); addr += sizeof(g_THRESH_EXP_FRAC);
    EEPROM.put(addr, g_TEMP_MIN_C);      addr += sizeof(g_TEMP_MIN_C);
    EEPROM.put(addr, g_TEMP_MAX_C);
}

bool eeprom_load_all()
{
    uint8_t magic = 0;
    EEPROM.get(EEPROM_ADDR_MAGIC, magic);
    if (magic != EEPROM_MAGIC) return false;   // Never written — use defaults.

    uint16_t addr = EEPROM_ADDR_DATA;
    EEPROM.get(addr, g_HR_MIN_BPM);      addr += sizeof(g_HR_MIN_BPM);
    EEPROM.get(addr, g_HR_MAX_BPM);      addr += sizeof(g_HR_MAX_BPM);
    EEPROM.get(addr, g_HR_OUTLIER_FRAC); addr += sizeof(g_HR_OUTLIER_FRAC);
    EEPROM.get(addr, g_SPO2_MIN_VALID);  addr += sizeof(g_SPO2_MIN_VALID);
    EEPROM.get(addr, g_DECIM_RATIO);     addr += sizeof(g_DECIM_RATIO);
    EEPROM.get(addr, g_LED_BRIGHTNESS);  addr += sizeof(g_LED_BRIGHTNESS);
    EEPROM.get(addr, g_SAMPLE_AVERAGE);  addr += sizeof(g_SAMPLE_AVERAGE);
    EEPROM.get(addr, g_ADC_RANGE);       addr += sizeof(g_ADC_RANGE);
    EEPROM.get(addr, g_PULSE_WIDTH);     addr += sizeof(g_PULSE_WIDTH);
    EEPROM.get(addr, g_CALIB_MIN_SWING); addr += sizeof(g_CALIB_MIN_SWING);
    EEPROM.get(addr, g_THRESH_INSP_FRAC);addr += sizeof(g_THRESH_INSP_FRAC);
    EEPROM.get(addr, g_THRESH_EXP_FRAC); addr += sizeof(g_THRESH_EXP_FRAC);
    EEPROM.get(addr, g_TEMP_MIN_C);      addr += sizeof(g_TEMP_MIN_C);
    EEPROM.get(addr, g_TEMP_MAX_C);
    return true;
}



#include "spo2_algorithm.h"
#include "heartRate.h"

MAX30105 particleSensor;

// Debug text on Serial0 (see DEBUG AIDS in the header). Keep 0 with the Python DAQ.
#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL 0
#endif


// ═══════════════════════════════════════════════════════════════════════════════
//  PHYSIOLOGICAL CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════
//
//  CURRENT SETTING: human bench validation range.
//  CHANGE BEFORE ANIMAL USE:
//    Rat anaesthetised:   HR_MIN=200, HR_MAX=500
//    Mouse anaesthetised: HR_MIN=250, HR_MAX=800
//
//  g_HR_OUTLIER_FRAC: fraction by which a new beat may deviate from the rolling
//  mean before being rejected. 0.35 = 35%. Tighten to 0.20 for cleaner data
//  in stable conditions; relax to 0.50 during induction/recovery when HR
//  changes rapidly.
// ═══════════════════════════════════════════════════════════════════════════════

float g_HR_MIN_BPM      = 40.0f;    // Mutable — updated by monitor_set_param()
float g_HR_MAX_BPM      = 800.0f;
float g_HR_OUTLIER_FRAC = 0.35f;

// SpO2 validity gate. Values outside [80, 100] are rejected regardless of
// validSpO2 flag. Physiologically implausible in a healthy anaesthetised animal.
// Lowering g_SPO2_MIN_VALID to 70 can be useful during diagnostic sessions to
// see algorithm output even from a degraded signal.
uint8_t g_SPO2_MIN_VALID = 80;
static const uint8_t SPO2_MAX_VALID  = 100;


// ═══════════════════════════════════════════════════════════════════════════════
//  SIGNAL PROCESSING CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════
//
//  g_DECIM_RATIO = 1: every FIFO sample (100 Hz) is written directly to the
//  SpO2 buffer. No additional software averaging is applied.
//
//  Consequence: the Maxim FIR filters inside spo2_algorithm.cpp operate at
//  100 Hz instead of the design frequency of 25 Hz. See tradeoff [T1] above.
//
//  If g_DECIM_RATIO is changed to 4: samples are averaged in groups of 4 before
//  writing to the buffer (25 Hz effective). This matches algorithm design
//  frequency but may attenuate AC amplitude, potentially causing validSpO2=0.
//  See tradeoff discussion above.
// ═══════════════════════════════════════════════════════════════════════════════

uint8_t g_DECIM_RATIO    = 1;    // Mutable — updated by monitor_set_param()

// HR ring buffer: rolling mean of last N valid beats.
// N=6 at 70 BPM (human) covers ~5 s. N=6 at 400 BPM (rat) covers ~0.9 s.
// Larger N → smoother but slower to respond to genuine rate changes.
static const uint8_t HR_RING_SIZE = 6;

// Values that are not refreshed are cleared instead of being held forever.
static const uint32_t HR_TIMEOUT_MS   = 5000;    // no accepted beat -> heartRate = 0
static const uint32_t SPO2_TIMEOUT_MS = 10000;   // no valid SpO2 block -> spO2 = 0

// SpO2 block: fixed at 100 by the Maxim algorithm. Do not change.
static const uint8_t SPO2_BLOCK   = 100;

// Sliding window parameters: retain 75 old samples, refill 25 per update.
// With g_DECIM_RATIO=1 at 100 Hz: buffer fills in 1 s, updates every 0.25 s.
// With g_DECIM_RATIO=4 at 25 Hz: buffer fills in 4 s, updates every 1 s.
static const uint8_t SPO2_KEEP    = 75;
static const uint8_t SPO2_REFILL  = SPO2_BLOCK - SPO2_KEEP;   // = 25


// ═══════════════════════════════════════════════════════════════════════════════
//  MODULE STATE
// ═══════════════════════════════════════════════════════════════════════════════

// SpO2 sample buffers — 400 B each. Mega 2560 only (see SRAM budget above).
static uint32_t irBuffer[SPO2_BLOCK];
static uint32_t redBuffer[SPO2_BLOCK];
static uint8_t  spo2FillIdx = 0;

// Decimation accumulators (used only when g_DECIM_RATIO > 1)
static uint8_t  decimCount = 0;
static uint32_t sumRed     = 0;
static uint32_t sumIR      = 0;

// HR ring buffer state
static float         hrRates[HR_RING_SIZE];
static uint8_t       hrSpot        = 0;
static uint8_t       hrFill        = 0;        // Valid entries (0 to HR_RING_SIZE)
static uint8_t       hrRejectCount = 0;        // Consecutive outlier rejections
static uint32_t      lastAcceptedIdx  = 0;      // sample index of the last accepted beat
static uint32_t      lastSpo2OkIdx    = 0;      // sample index of the last valid SpO2 block

// Beat timing is based on the FIFO SAMPLE INDEX, not on millis(). The MAX30102
// produces samples on its own hardware clock, while loop() may read them in
// bursts (several samples per call). Time-stamping a burst with one millis()
// value would quantise beat intervals to the loop period; counting samples
// does not. Interval (ms) = samples between beats x measured sample period.
static uint32_t      lastBeatIdx      = 0;      // sample index of previous beat / last no-contact sample
static uint32_t      sampleCount      = 0;      // FIFO samples consumed since boot
static float         g_samplePeriodMs = 10.0f;  // FIFO sample period (ms); measured at run time
static bool          ratePrimed       = false;  // rate-measurement window started
static unsigned long rateT0           = 0;      // millis() at window start
static uint32_t      rateN0           = 0;      // sampleCount at window start

// Last IR raw value — used by _debug_serial() for contact diagnostics
static uint32_t irRawLast = 0;

// Shared outputs — declared extern in surgery_monitor.ino
int heartRate = 0;   // BPM, 0 = no valid data
int spO2      = 0;   // %,   0 = no valid data


// ── Sensor hardware parameters — mutable, updated by max30102_set_param() ────
uint8_t  g_LED_BRIGHTNESS = 80;
uint8_t  g_SAMPLE_AVERAGE = 4;
uint32_t g_ADC_RANGE      = 16384;
uint16_t g_PULSE_WIDTH    = 215;    // Valid values: 69, 118, 215, 411 µs


// ════════════════════════════════════════════════════════════════════════════
//  max30102_apply_sensor_config()
//  Re-runs particleSensor.setup() with current g_* values.
//  Called by max30102_set_param() when LED, ADC_RANGE, SAMPLE_AVERAGE or PULSE_WIDTH change.
// ════════════════════════════════════════════════════════════════════════════

static void max30102_apply_sensor_config()
{
    particleSensor.setup(
        g_LED_BRIGHTNESS,
        g_SAMPLE_AVERAGE,
        2,      // Red + IR (fixed)
        400,    // internal sample rate (fixed)
        g_PULSE_WIDTH,  // pulse width µs — set in max30102_set_param()
        g_ADC_RANGE
    );
    Wire.setClock(400000UL);

    // New sensor configuration → new FIFO rate (400 Hz / sampleAverage).
    // Restart the run-time measurement of the sample period from nominal.
    g_samplePeriodMs = 2.5f * (float)g_SAMPLE_AVERAGE;
    ratePrimed       = false;
}


// ════════════════════════════════════════════════════════════════════════════
//  max30102_set_param()
//  Called from Py_Vital-Signs.ino when a {"cmd":"set"} command arrives.
//  Returns true if key recognised and value applied; false otherwise.
// ════════════════════════════════════════════════════════════════════════════

bool max30102_set_param(const String& key, float value)
{
    bool sensorChanged = false;

    if      (key == "HR_MIN")    { g_HR_MIN_BPM      = constrain(value, 20.0f, 780.0f); }
    else if (key == "HR_MAX")    { g_HR_MAX_BPM      = constrain(value, 50.0f, 800.0f);
                                   if (g_HR_MAX_BPM <= g_HR_MIN_BPM) g_HR_MAX_BPM = g_HR_MIN_BPM + 50.0f; }
    else if (key == "HR_OUTLIER"){ g_HR_OUTLIER_FRAC = constrain(value, 0.10f, 0.70f); }
    else if (key == "SPO2_MIN")  { g_SPO2_MIN_VALID  = (uint8_t)constrain(value, 50.0f, 95.0f); }
    else if (key == "DECIM_RATIO"){
        uint8_t d = (uint8_t)constrain(value, 1.0f, 4.0f);
        g_DECIM_RATIO = (d <= 1) ? 1 : (d <= 2) ? 2 : 4;
        decimCount = 0; sumRed = 0; sumIR = 0;
    }
    else if (key == "LED")       { g_LED_BRIGHTNESS  = (uint8_t)constrain(value, 10.0f, 255.0f); sensorChanged = true; }
    else if (key == "ADC_RANGE") {
        uint32_t r = (uint32_t)value;
        g_ADC_RANGE = (r <= 2048) ? 2048 : (r <= 4096) ? 4096 : (r <= 8192) ? 8192 : 16384;
        sensorChanged = true;
    }
    else if (key == "SAMPLE_AVG"){
        uint8_t s = (uint8_t)constrain(value, 1.0f, 32.0f);
        g_SAMPLE_AVERAGE = (s<=1)?1:(s<=2)?2:(s<=4)?4:(s<=8)?8:(s<=16)?16:32;
        sensorChanged = true;
    }
    else if (key == "PULSE_WIDTH") {
        // Valid values: 69, 118, 215, 411 µs. Clamp to nearest valid.
        uint16_t pw = (uint16_t)value;
        g_PULSE_WIDTH = (pw <= 69) ? 69 : (pw <= 118) ? 118 : (pw <= 215) ? 215 : 411;
        sensorChanged = true;
    }
    else { return false; }

    if (sensorChanged) max30102_apply_sensor_config();
    eeprom_save_all();
    return true;
}



// ════════════════════════════════════════════════════════════════════════════
//  max30102_init()
//  Called once from setup() in surgery_monitor.ino.
//  Returns true on success, false if sensor not detected on I2C bus.
// ════════════════════════════════════════════════════════════════════════════

bool max30102_init()
{
    Serial.println(F(""));
    Serial.println(F("========================================"));
    Serial.println(F("  SURGERY MONITOR — MAX30102 INIT"));
    Serial.println(F("========================================"));

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println(F("[ERROR] MAX30102 not found on I2C bus."));
        Serial.println(F("  Check: SDA/SCL wiring, 3.3V supply, address 0x57"));
        return false;
    }

    // Load saved parameters from EEPROM (if previously written).
    // If EEPROM is blank (first boot), g_* variables retain their
    // compile-time defaults defined above.
    if (eeprom_load_all()) {
        Serial.println(F("[OK] Parameters loaded from EEPROM."));
    } else {
        Serial.println(F("[OK] EEPROM blank — using compile-time defaults."));
    }

    // Initialise sensor with current g_* values (defaults or EEPROM).
    max30102_apply_sensor_config();

    // Initialise HR buffer to zero (not a valid BPM value — outlier gate
    // is inactive until hrFill >= 4, so these zeros are never tested).
    for (uint8_t i = 0; i < HR_RING_SIZE; i++) hrRates[i] = 0.0f;

    Serial.println(F("[OK] MAX30102 initialised."));
    Serial.print(F("  LED="));   Serial.print(g_LED_BRIGHTNESS);
    Serial.print(F(" | avg="));  Serial.print(g_SAMPLE_AVERAGE);
    Serial.print(F(" | adc="));  Serial.print(g_ADC_RANGE);
    Serial.print(F(" | decim=")); Serial.print(g_DECIM_RATIO);
    Serial.print(F(" | pw="));   Serial.print(g_PULSE_WIDTH); Serial.println(F("us"));
    Serial.println(F("  Place sensor on fingertip and hold steady..."));
    Serial.println(F("  IR > 50000 = good contact | IR < 5000 = no contact"));
    Serial.println(F("========================================"));
    return true;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  _process_beat()
//  Called per detected beat event from max30102_update().
//  sampleIdx: running FIFO sample index of the sample that triggered the beat.
//  The beat interval is (sample count between beats) x (measured sample
//  period), independent of when loop() happened to read the FIFO.
// ═══════════════════════════════════════════════════════════════════════════════

static void _process_beat(uint32_t sampleIdx)
{
    const uint32_t deltaSamples = sampleIdx - lastBeatIdx;
    lastBeatIdx = sampleIdx;

    // First beat after reset: no interval available yet.
    if (deltaSamples == 0) return;

    const float bpm = 60000.0f / ((float)deltaSamples * g_samplePeriodMs);

    // Physiological gate: reject beats outside the expected range.
    // This is the primary defence against double-detection and motion artefacts.
    // IMPORTANT: change g_HR_MIN_BPM / g_HR_MAX_BPM before animal use (see above).
    if (bpm < g_HR_MIN_BPM || bpm > g_HR_MAX_BPM) return;

    // Outlier rejection: active only after hrFill >= 4 (buffer has enough
    // history to form a meaningful mean). Before that, all valid beats are
    // accepted unconditionally to allow fast initial convergence.
    if (hrFill >= 4) {
        float mean = 0.0f;
        for (uint8_t i = 0; i < hrFill; i++) mean += hrRates[i];
        mean /= (float)hrFill;

        if (mean > 0.0f && fabsf(bpm - mean) / mean > g_HR_OUTLIER_FRAC) {
            hrRejectCount++;
            // Auto-recovery: 5 consecutive rejections suggest the true rate
            // has shifted (e.g., induction, recovery from anaesthesia).
            // Reset buffer to allow re-convergence at the new rate.
            if (hrRejectCount >= 5) {
                hrFill        = 0;
                hrSpot        = 0;
                hrRejectCount = 0;
#if DEBUG_SERIAL
                Serial.println(F("[HR] Auto-reset: sustained rate shift detected."));
#endif
            }
            return;
        }
    }

    hrRejectCount = 0;

    lastAcceptedIdx = sampleIdx;

    // Write to ring buffer (circular, overwrites oldest entry when full).
    hrRates[hrSpot] = bpm;
    hrSpot = (hrSpot + 1) % HR_RING_SIZE;
    if (hrFill < HR_RING_SIZE) hrFill++;

    // Recompute rolling mean from all valid entries.
    float sum = 0.0f;
    for (uint8_t i = 0; i < hrFill; i++) sum += hrRates[i];
    heartRate = (int)(sum / (float)hrFill + 0.5f);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  _process_spo2_block()
//  Called when spo2FillIdx reaches SPO2_BLOCK (100 samples).
//  Runs the Maxim SpO2 algorithm and slides the window.
// ═══════════════════════════════════════════════════════════════════════════════

static void _process_spo2_block()
{
    int8_t  validSpO2 = 0, validHR = 0;
    int32_t tempSpO2  = 0, tempHR  = 0;

    // The Maxim algorithm computes SpO2 from the ratio:
    //   R = (AC_red / DC_red) / (AC_ir / DC_ir)
    // and maps R to SpO2% via an internal empirical curve.
    // validSpO2 = 1 only when AC amplitude and signal quality pass
    // internal acceptance criteria. validHR from this algorithm is
    // not used — heartRate is computed independently via checkForBeat().
    maxim_heart_rate_and_oxygen_saturation(
        irBuffer,  SPO2_BLOCK, redBuffer,
        &tempSpO2, &validSpO2,
        &tempHR,   &validHR
    );

    // Accept only when the algorithm flags the block as valid AND the value
    // is within the physiological plausibility window.
    // If invalid: spO2 retains its previous value (last-value-hold behaviour,
    // consistent with clinical monitor practice during transient signal loss).
    if (validSpO2 == 1 &&
        tempSpO2 >= g_SPO2_MIN_VALID &&
        tempSpO2 <= SPO2_MAX_VALID)
    {
        spO2 = (int)tempSpO2;
        lastSpo2OkIdx = sampleCount;
    }

    // Slide the window: shift the last SPO2_KEEP (75) samples to the start,
    // making room for SPO2_REFILL (25) new samples.
    // This produces a new SpO2 estimate every SPO2_REFILL samples instead of
    // waiting for a full fresh block of 100.
    for (uint8_t i = SPO2_REFILL; i < SPO2_BLOCK; i++) {
        irBuffer[i  - SPO2_REFILL] = irBuffer[i];
        redBuffer[i - SPO2_REFILL] = redBuffer[i];
    }
    spo2FillIdx = SPO2_KEEP;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL PLOTTER SUBSYSTEM — RAW IR WAVEFORM
// ═══════════════════════════════════════════════════════════════════════════════
//
//  ACTIVATION
//  Set PLOTTER_ENABLE = true  → Serial Plotter (raw IR counts, one per sample)
//  Set PLOTTER_ENABLE = false → normal operation (JSON stream; text only with DEBUG_SERIAL = 1)
//  The two modes are mutually exclusive — they share the same UART.
//
//  HOW TO USE
//  1. Set PLOTTER_ENABLE = true.
//  2. Upload the sketch.
//  3. Open Tools → Serial Plotter (not Serial Monitor).
//  4. Place sensor on fingertip — raw IR waveform appears immediately.
//
//  WHAT YOU SEE
//  One channel: IR_RAW — the raw IR count value from the FIFO as delivered
//  by the hardware (after sampleAverage=4 smoothing). No DC removal, no
//  filtering. Every FIFO sample is sent — the waveform is complete.
//
//  The DC level sits at ~75000-85000 counts (human fingertip, LED=80,
//  adcRange=16384). The heartbeat pulse modulates this baseline by roughly
//  200-1000 counts per beat — visible as small oscillations on top of the
//  DC plateau. The Arduino Serial Plotter auto-scales the Y axis to fit the
//  full DC range, which compresses the AC detail visually. This is expected
//  behaviour — the signal is correct even if the oscillations look small.
//
//  SAMPLE RATE
//  One Serial.println() per FIFO sample, inside the while() loop.
//  At sampleAverage=4 and 400 Hz: FIFO delivers 100 Hz.
//  100 samples/s x ~9 bytes/sample = ~900 bytes/s at 115200 baud — safe.
//
//  For rodent use: IR_RAW DC will be lower (thinner tissue, less absorption).
//  Expected range: 20000-80000 counts depending on site and LED setting.
// ═══════════════════════════════════════════════════════════════════════════════

static const bool PLOTTER_ENABLE = false;  // true = Plotter | false = Monitor

static void _plot_serial(uint32_t irRaw)
{
    if (!PLOTTER_ENABLE) return;
    Serial.println(irRaw);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  _debug_serial()
//  Prints one diagnostic line per second to Serial Monitor.
//  Compiled only with DEBUG_SERIAL = 1; suppressed when PLOTTER_ENABLE = true.
//
//  OUTPUT FORMAT (plain text — use only without Python DAQ connected):
//    IR=XXXXX [status]  HR=XXXbpm  SpO2=XX%
//
//  NOTE: this function emits plain text. Do not use simultaneously with the
//  Python DAQ (Py_VitalSigns_DAQ.py), which expects pure JSON on Serial0.
//  The _debug_serial() output is intended for Arduino IDE Serial Monitor only.
//
//  IR CONTACT THRESHOLDS (adcRange=16384, LED=80, human fingertip):
//    IR < 5000    No contact — sensor in air
//    IR 5k-50k    Weak contact — press harder or reposition
//    IR > 50000   Adequate contact
//
//  For rodent sites (tail, paw): expected DC level will differ.
//  Recalibrate thresholds after first animal measurements.
// ═══════════════════════════════════════════════════════════════════════════════

#if DEBUG_SERIAL
static void _debug_serial()
{
    // Suppressed when Serial Plotter is active — text labels corrupt CSV parsing.
    if (PLOTTER_ENABLE) return;

    static unsigned long lastPrintMs = 0;
    const unsigned long nowMs = millis();
    if (nowMs - lastPrintMs < 1000) return;
    lastPrintMs = nowMs;

    Serial.print(F("IR="));
    Serial.print(irRawLast);

    if (irRawLast < 5000) {
        Serial.print(F(" [NO CONTACT]"));
    } else if (irRawLast < 50000) {
        Serial.print(F(" [WEAK CONTACT]"));
    } else {
        Serial.print(F(" [OK]"));
    }

    Serial.print(F("  HR="));
    if (hrFill == 0) {
        Serial.print(F("waiting"));
    } else if (hrFill < 4) {
        Serial.print(heartRate);
        Serial.print(F("bpm [settling "));
        Serial.print(hrFill);
        Serial.print(F("/4]"));
    } else {
        Serial.print(heartRate);
        Serial.print(F("bpm"));
    }

    Serial.print(F("  SpO2="));
    if (spO2 == 0) {
        Serial.print(F("buf "));
        Serial.print(spo2FillIdx);
        Serial.print(F("/100"));
    } else {
        Serial.print(spO2);
        Serial.print(F("%"));
    }

    Serial.println();
}
#endif


// ═══════════════════════════════════════════════════════════════════════════════
//  max30102_update()
//  Called every loop() iteration in surgery_monitor.ino. Non-blocking.
//
//  Execution flow per call:
//    1. particleSensor.check() — transfers hardware FIFO to library buffer
//    2. while(available) — process each buffered sample:
//         a. Disconnection guard: reset state if IR < 5000
//         b. PATH 1: checkForBeat(irRaw) → _process_beat() if beat detected
//         c. PATH 2: accumulate sample → write to SpO2 buffer when decimated
//                    → _process_spo2_block() when buffer full
//    3. FIFO sample-period measurement (every >= 4 s)
//    4. _debug_serial() — 1 Hz diagnostic line (only with DEBUG_SERIAL = 1)
//
//  PATH 1 and PATH 2 are fully independent: a beat detection event does not
//  affect SpO2 buffer accumulation and vice versa.
// ═══════════════════════════════════════════════════════════════════════════════

void max30102_update()
{
    particleSensor.check();

    while (particleSensor.available())
    {
        const uint32_t redRaw = particleSensor.getFIFORed();
        const uint32_t irRaw  = particleSensor.getFIFOIR();
        particleSensor.nextSample();
        sampleCount++;       // Every FIFO sample is counted, contact or not.

        irRawLast = irRaw;   // Store for debug output.

        // ── Disconnection guard ───────────────────────────────────────────────
        // IR < 5000 reliably indicates the sensor is not in contact with tissue.
        // Reset all running state to prevent stale values persisting on display
        // and to ensure a clean restart when contact is re-established.
        if (irRaw < 5000) {
            heartRate   = 0;
            spO2        = 0;
            hrFill      = 0;
            hrSpot      = 0;
            hrRejectCount = 0;
            spo2FillIdx = 0;
            decimCount  = 0;
            sumRed      = 0;
            sumIR       = 0;
            lastBeatIdx = sampleCount;
            continue;
        }

        // ── PATH 1 — Heart Rate (100 Hz) ──────────────────────────────────────
        // checkForBeat() is a derivative-based peak detector from the SparkFun
        // library. It operates on raw irRaw values (no DC removal applied here).
        // The hardware sampleAverage=4 smooths the signal sufficiently for
        // detection on human fingertip. For rodent sites with weaker AC signal,
        // a DC-removal pre-filter may be required (see tradeoff [T2]).
        // checkForBeat() result used only for _process_beat — no state stored.
        if (checkForBeat(irRaw)) {
            _process_beat(sampleCount);
        }

        // Serial Plotter — sends raw IR count. No-op when PLOTTER_ENABLE = false.
        _plot_serial(irRaw);

        // ── PATH 2 — SpO2 (g_DECIM_RATIO = 1, effective 100 Hz to algorithm) ───
        // With g_DECIM_RATIO=1, sumRed/sumIR accumulate exactly one sample each,
        // and the division sumRed/1 is a no-op. The decimation structure is
        // retained so that g_DECIM_RATIO can be changed to 2 or 4 without
        // restructuring the code.
        sumRed += redRaw;
        sumIR  += irRaw;
        decimCount++;

        if (decimCount >= g_DECIM_RATIO) {
            redBuffer[spo2FillIdx] = sumRed / g_DECIM_RATIO;
            irBuffer[spo2FillIdx]  = sumIR  / g_DECIM_RATIO;
            sumRed     = 0;
            sumIR      = 0;
            decimCount = 0;
            spo2FillIdx++;

            if (spo2FillIdx >= SPO2_BLOCK) {
                _process_spo2_block();
            }
        }
    }

    // Timeouts: a value that stopped being refreshed must not stay on screen.
    if (heartRate > 0 &&
        (float)(sampleCount - lastAcceptedIdx) * g_samplePeriodMs > (float)HR_TIMEOUT_MS) {
        heartRate     = 0;
        hrFill        = 0;
        hrSpot        = 0;
        hrRejectCount = 0;
    }
    if (spO2 > 0 &&
        (float)(sampleCount - lastSpo2OkIdx) * g_samplePeriodMs > (float)SPO2_TIMEOUT_MS) {
        spO2 = 0;
    }

    // ── FIFO sample-period measurement ────────────────────────────────────────
    // The MAX30102 samples from its internal oscillator, the Arduino from its
    // crystal; their ratio is not exactly nominal. Every >= 4 s, the real
    // period is estimated as (elapsed ms) / (samples consumed). Windows with
    // implausible results (sensor absent, FIFO overflow) are ignored.
    {
        const unsigned long tEnd = millis();
        if (!ratePrimed) {
            ratePrimed = true;
            rateT0     = tEnd;
            rateN0     = sampleCount;
        } else if (tEnd - rateT0 >= 4000UL) {
            const uint32_t n = sampleCount - rateN0;
            if (n >= 20) {
                const float p       = (float)(tEnd - rateT0) / (float)n;
                const float nominal = 2.5f * (float)g_SAMPLE_AVERAGE;
                if (p > 0.85f * nominal && p < 1.15f * nominal) g_samplePeriodMs = p;
            }
            rateT0 = tEnd;
            rateN0 = sampleCount;
        }
    }

    // 1 Hz diagnostic — Serial Monitor only (DEBUG_SERIAL = 1). Not valid JSON.
#if DEBUG_SERIAL
    _debug_serial();
#endif
}


// ═══════════════════════════════════════════════════════════════════════════════
//  ACCESSORS
// ═══════════════════════════════════════════════════════════════════════════════

int  max30102_getHR()     { return heartRate; }
int  max30102_getSpO2()   { return spO2; }

// hrValid: true once 4 beats have been accepted (mean is reliable).
bool max30102_hrValid()   { return (hrFill >= 4 && heartRate > 0); }

// spo2Valid: true when last accepted SpO2 value is within valid range.
bool max30102_spo2Valid() { return (spO2 >= g_SPO2_MIN_VALID); }
uint32_t max30102_getIR() { return irRawLast; }
