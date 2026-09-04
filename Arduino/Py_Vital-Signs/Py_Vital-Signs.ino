/**
 * @file    Py_Vital-Signs.ino
 * PROJECT: Vital-signs monitor — small rodents (rat / mouse)
 * @version 1.0
 * @author  Flávio Mourão — Mar, 2026
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * MODULE RESPONSIBILITY
 * ═══════════════════════════════════════════════════════════════════════════════
 * Handles two independent communication paths on Serial0 (USB, 115200 baud):
 *
 *   OUTBOUND (Arduino → Python, 100 Hz):
 *     JSON stream of all physiological variables + millis() timestamp.
 *     Format: {"hr":N,"sp":N,"rr":N,"pz":N,"t":N,"ir":N,"ts":N}
 *
 *   INBOUND (Python → Arduino, on demand):
 *     Configuration commands from the Python Settings panel.
 *     Set:  SOH(0x01) + {"cmd":"set","key":"PARAM","val":VALUE}\n
 *     Get:  SOH(0x01) + {"cmd":"get"}\n
 *     ACK:  {"ack":"PARAM","val":APPLIED_VALUE}
 *     NAK:  {"nak":"reason"}
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * SOH HANDSHAKE PROTOCOL
 * ═══════════════════════════════════════════════════════════════════════════════
 * The Python side prefixes every command with SOH (0x01, Start of Header).
 * When the Arduino receives SOH, it immediately sets cmdInProgress = true,
 * halting the outbound 100 Hz stream. This prevents data bytes from
 * interleaving with the incoming command and corrupting key extraction.
 *
 * After the command '\n' is received, the ACK/NAK is sent and the outbound
 * stream resumes. Total command window: ~5–10 ms per parameter.
 *
 * The Python side uses a queue.Queue (same pattern as conditioning_setup_dark.py):
 * all serial writes happen inside SerialThread.run() — never from the main Qt
 * thread — so there is no concurrent port access.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * CONFIGURABLE PARAMETERS (via Python Settings panel)
 * ═══════════════════════════════════════════════════════════════════════════════
 * Key            Module                    Variable
 * ─────────────────────────────────────────────────────────────────────────────
 * HR_MIN         surgery_monitor_MAX30102  g_HR_MIN_BPM
 * HR_MAX         surgery_monitor_MAX30102  g_HR_MAX_BPM
 * HR_OUTLIER     surgery_monitor_MAX30102  g_HR_OUTLIER_FRAC
 * SPO2_MIN       surgery_monitor_MAX30102  g_SPO2_MIN_VALID
 * DECIM_RATIO    surgery_monitor_MAX30102  g_DECIM_RATIO
 * LED            surgery_monitor_MAX30102  g_LED_BRIGHTNESS  (reruns sensor setup)
 * ADC_RANGE      surgery_monitor_MAX30102  g_ADC_RANGE       (reruns sensor setup)
 * SAMPLE_AVG     surgery_monitor_MAX30102  g_SAMPLE_AVERAGE  (reruns sensor setup)
 * CALIB_SWING    surgery_monitor           g_CALIB_MIN_SWING
 * THRESH_INSP    surgery_monitor           g_THRESH_INSP_FRAC
 * THRESH_EXP     surgery_monitor           g_THRESH_EXP_FRAC
 * TEMP_MIN       surgery_monitor           g_TEMP_MIN_C
 * TEMP_MAX       surgery_monitor           g_TEMP_MAX_C
 *
 * All parameters are saved to EEPROM on change and restored on power-up.
 * See surgery_monitor_MAX30102.ino for the EEPROM memory map.
 */

#include <Arduino.h>

// ── External variables ────────────────────────────────────────────────────────
// Defined in other .ino files; merged by the Arduino IDE before compiling.

extern volatile int   respRate;      // Respiratory rate (rpm)   — surgery_monitor.ino
extern volatile int   adcDisplay;    // Filtered piezo ADC value — surgery_monitor.ino
extern float          tempCelsius;   // Core temperature (°C)    — surgery_monitor.ino
extern int            heartRate;     // Heart rate (BPM)         — surgery_monitor_MAX30102.ino
extern int            spO2;          // Oxygen saturation (%)    — surgery_monitor_MAX30102.ino
extern uint32_t       max30102_getIR();

// Parameter dispatch functions — defined in their respective modules.
extern bool max30102_set_param(const String& key, float value);
extern bool monitor_set_param(const String& key, float value);

// All mutable g_* parameters — declared extern here for _send_all_params().
extern float    g_HR_MIN_BPM;
extern float    g_HR_MAX_BPM;
extern float    g_HR_OUTLIER_FRAC;
extern uint8_t  g_SPO2_MIN_VALID;
extern uint8_t  g_DECIM_RATIO;
extern uint8_t  g_LED_BRIGHTNESS;
extern uint8_t  g_SAMPLE_AVERAGE;
extern uint32_t g_ADC_RANGE;
extern int      g_CALIB_MIN_SWING;
extern float    g_THRESH_INSP_FRAC;
extern float    g_THRESH_EXP_FRAC;
extern float    g_TEMP_MIN_C;
extern float    g_TEMP_MAX_C;


// ── Outbound stream timing ────────────────────────────────────────────────────

static const uint8_t  INTERVAL_USB_MS = 10;   // 100 Hz outbound rate
static unsigned long  timerUSB        = 0;


// ── Inbound command parser state ──────────────────────────────────────────────
// cmdInProgress: true from SOH receipt until '\n' is processed.
// While true, the outbound stream is suppressed to prevent interleaving.

static const uint8_t CMD_BUF_SIZE = 96;
static char          cmdBuffer[CMD_BUF_SIZE];
static uint8_t       cmdLen        = 0;
static bool          cmdInProgress = false;


// ════════════════════════════════════════════════════════════════════════════
//  _send_ack_val()
//  Emits one {"ack":"KEY","val":VALUE} line for _send_all_params().
// ════════════════════════════════════════════════════════════════════════════

static void _send_ack_val(const char* key, float val)
{
    Serial.print(F("{\"ack\":\""));
    Serial.print(key);
    Serial.print(F("\",\"val\":"));
    Serial.print(val, 4);
    Serial.println(F("}"));
}


// ════════════════════════════════════════════════════════════════════════════
//  _send_all_params()
//  Called when Python sends {"cmd":"get"} immediately after connecting.
//  Responds with all current g_* values as individual ACK-format packets
//  so the Python Settings dialog can populate its param_cache with real
//  firmware values rather than hardcoded defaults.
// ════════════════════════════════════════════════════════════════════════════

static void _send_all_params()
{
    _send_ack_val("HR_MIN",      g_HR_MIN_BPM);
    _send_ack_val("HR_MAX",      g_HR_MAX_BPM);
    _send_ack_val("HR_OUTLIER",  g_HR_OUTLIER_FRAC);
    _send_ack_val("SPO2_MIN",    (float)g_SPO2_MIN_VALID);
    _send_ack_val("DECIM_RATIO", (float)g_DECIM_RATIO);
    _send_ack_val("LED",         (float)g_LED_BRIGHTNESS);
    _send_ack_val("ADC_RANGE",   (float)g_ADC_RANGE);
    _send_ack_val("SAMPLE_AVG",  (float)g_SAMPLE_AVERAGE);
    _send_ack_val("CALIB_SWING", (float)g_CALIB_MIN_SWING);
    _send_ack_val("THRESH_INSP", g_THRESH_INSP_FRAC);
    _send_ack_val("THRESH_EXP",  g_THRESH_EXP_FRAC);
    _send_ack_val("TEMP_MIN",    g_TEMP_MIN_C);
    _send_ack_val("TEMP_MAX",    g_TEMP_MAX_C);
}


// ════════════════════════════════════════════════════════════════════════════
//  _parse_and_apply()
//  Parses the accumulated cmdBuffer and dispatches to set_param().
//  Sends ACK or NAK before the next outbound data packet.
//
//  Handles two command types:
//    {"cmd":"get"}                        — report all current g_* values
//    {"cmd":"set","key":"K","val":V}      — update one parameter
//
//  Parser uses char[] only — no String heap allocation — to avoid heap
//  fragmentation that corrupts adjacent static buffers at 100 Hz stream rate.
//  A single String(keyBuf) allocation is made only for the set_param dispatch
//  interface; it is freed before the next command arrives.
// ════════════════════════════════════════════════════════════════════════════

static void _parse_and_apply(const char* buf)
{
    // ── {"cmd":"get"} ─────────────────────────────────────────────────────────
    if (strstr(buf, "\"cmd\":\"get\"")) {
        _send_all_params();
        return;
    }

    // ── Extract key ───────────────────────────────────────────────────────────
    const char* ks = strstr(buf, "\"key\":\"");
    if (!ks) { Serial.println(F("{\"nak\":\"no_key\"}")); return; }
    ks += 7;   // skip past: "key":"
    const char* ke = strchr(ks, '"');
    if (!ke)  { Serial.println(F("{\"nak\":\"bad_key\"}")); return; }

    char    keyBuf[32];
    uint8_t keyLen = (uint8_t)(ke - ks);
    if (keyLen >= sizeof(keyBuf)) { Serial.println(F("{\"nak\":\"key_too_long\"}")); return; }
    strncpy(keyBuf, ks, keyLen);
    keyBuf[keyLen] = '\0';

    // ── Extract val ───────────────────────────────────────────────────────────
    const char* vs = strstr(buf, "\"val\":");
    if (!vs) { Serial.println(F("{\"nak\":\"no_val\"}")); return; }
    float val = atof(vs + 6);

    // ── Dispatch ──────────────────────────────────────────────────────────────
    String key = String(keyBuf);
    bool ok = max30102_set_param(key, val) || monitor_set_param(key, val);

    if (ok) {
        Serial.print(F("{\"ack\":\""));
        Serial.print(keyBuf);
        Serial.print(F("\",\"val\":"));
        Serial.print(val, 4);
        Serial.println(F("}"));
    } else {
        Serial.print(F("{\"nak\":\"unknown_"));
        Serial.print(keyBuf);
        Serial.println(F("\"}"));
    }
}


// ════════════════════════════════════════════════════════════════════════════
//  py_vital_signs_init()
//  Called once from setup() in surgery_monitor.ino.
// ════════════════════════════════════════════════════════════════════════════

void py_vital_signs_init()
{
    cmdLen        = 0;
    cmdInProgress = false;
    memset(cmdBuffer, 0, sizeof(cmdBuffer));
    Serial.println(F("[OK] USB stream ready — 100 Hz outbound | inbound cmd parser active."));
}


// ════════════════════════════════════════════════════════════════════════════
//  py_vital_signs_update()
//  Called every loop() iteration from surgery_monitor.ino. Non-blocking.
//
//  Step 1 — Inbound: drain Serial receive buffer one byte at a time.
//           SOH sets cmdInProgress and suppresses outbound until '\n'.
//  Step 2 — Outbound: emit 100 Hz JSON packet if not in command mode.
// ════════════════════════════════════════════════════════════════════════════

void py_vital_signs_update()
{
    // ── Step 1: Inbound parser ────────────────────────────────────────────────
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\x01') {
            // SOH: start of command. Halt outbound stream and reset buffer.
            cmdInProgress = true;
            cmdLen        = 0;
            memset(cmdBuffer, 0, sizeof(cmdBuffer));

        } else if (c == '\n') {
            // End of command line — process and resume outbound.
            cmdBuffer[cmdLen] = '\0';
            if (cmdLen > 2) _parse_and_apply(cmdBuffer);
            cmdLen        = 0;
            cmdInProgress = false;

        } else if (c != '\r' && cmdInProgress) {
            // Accumulate command bytes (only after SOH).
            if (cmdLen < CMD_BUF_SIZE - 1) {
                cmdBuffer[cmdLen++] = c;
            } else {
                // Buffer overflow — discard and reset.
                cmdLen        = 0;
                cmdInProgress = false;
            }
        }
        // Bytes received before SOH are silently discarded.
    }

    // ── Step 2: Outbound 100 Hz JSON stream ──────────────────────────────────
    // Suppressed while a command is in progress to prevent outbound bytes
    // from entering the Python receive buffer and corrupting ACK parsing.
    if (cmdInProgress) return;

    const unsigned long nowMs = millis();
    if (nowMs - timerUSB < INTERVAL_USB_MS) return;
    timerUSB += INTERVAL_USB_MS;

    // Atomic snapshot of ISR-written volatile variables.
    noInterrupts();
    const int safeRR    = respRate;
    const int safePiezo = adcDisplay;
    interrupts();

    // heartRate, spO2, tempCelsius are written only from main loop tasks —
    // no ISR writes them, so no critical section is needed here.
    const float safeTemp = tempCelsius;

    // Manual JSON serialisation — avoids ArduinoJson heap allocation.
    Serial.print(F("{\"hr\":"));  Serial.print(heartRate);
    Serial.print(F(",\"sp\":"));  Serial.print(spO2);
    Serial.print(F(",\"rr\":"));  Serial.print(safeRR);
    Serial.print(F(",\"pz\":"));  Serial.print(safePiezo);
    Serial.print(F(",\"t\":"));
    if (safeTemp >= g_TEMP_MIN_C && safeTemp <= g_TEMP_MAX_C) {
        Serial.print(safeTemp, 1);
    } else {
        Serial.print(0);   // Sentinel: probe disconnected or out of range.
    }
    Serial.print(F(",\"ir\":"));  Serial.print(max30102_getIR());
    Serial.print(F(",\"ts\":"));  Serial.print(nowMs);
    Serial.println(F("}"));
}
