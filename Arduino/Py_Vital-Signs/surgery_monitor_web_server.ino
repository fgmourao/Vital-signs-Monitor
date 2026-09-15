/**
 * @file    surgery_monitor_web_server.ino
 * PROJECT: Vital-signs monitor — small rodents (rat / mouse)
 * @version 2.0
 * @author  Flávio Mourão — Mar, 2026
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * MODULE RESPONSIBILITY
 * ═══════════════════════════════════════════════════════════════════════════════
 * Transmits a 100 Hz JSON stream of all physiological parameters to the
 * on-board ESP8266 via hardware Serial3. The ESP8266 forwards each packet to
 * connected browser clients over WebSocket, enabling real-time waveform display
 * on any device connected to the monitor Wi-Fi access point.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * HARDWARE CONTEXT — RobotDyn Mega+WiFi
 * ═══════════════════════════════════════════════════════════════════════════════
 * The RobotDyn Mega+WiFi board integrates an ATmega2560 and an ESP8266 on a
 * single PCB. The two processors communicate via a hardware UART bridge:
 *
 *   ATmega2560 Serial3 (TX3/RX3, pins 14/15)
 *         ↕  internal PCB traces (no external wiring required)
 *   ESP8266 Serial (GPIO1/GPIO3)
 *
 * Baud rate: 115200. Must match ESP8266_VitalSigns_Server.ino.
 * The DIP switch bank on the board controls which UART is routed to the USB
 * chip — set it for ESP8266 programming when flashing, and for Mega operation
 * during normal use. Consult the RobotDyn board documentation.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * JSON PACKET FORMAT
 * ═══════════════════════════════════════════════════════════════════════════════
 * One packet per 10 ms (100 Hz). Each packet is a single-line JSON object
 * terminated by '\n'. The ESP8266 reads line by line (non-blocking byte
 * accumulation) and broadcasts the complete object over WebSocket.
 *
 * Field map:
 *   "hr"  — Heart rate (BPM, int). 0 = no valid signal.
 *   "sp"  — SpO2 (%, int). 0 = no valid signal.
 *   "rr"  — Respiratory rate (rpm, int). 0 = apnoea or no signal.
 *   "pz"  — Respiratory sensor ADC value (counts, 0–1023, int).
 *           Sensor-agnostic: compatible with piezo film and resistive
 *           pressure sensors. Used for respiratory waveform in the browser.
 *   "t"   — Core temperature (°C, one decimal). 0 = probe disconnected
 *           or reading outside [g_TEMP_MIN_C, g_TEMP_MAX_C].
 *   "ir"  — Raw IR photodetector count from MAX30102 (uint32, 0–262143).
 *           Used for PPG waveform display in the browser.
 *   "ts"  — ATmega2560 millis() at packet serialisation time (ms, uint32).
 *           Wraps at 2^32 ms (~49.7 days). Used by the browser dashboard
 *           as the common time axis for both waveform channels.
 *
 * Example packet:
 *   {"hr":350,"sp":98,"rr":85,"pz":512,"t":37.2,"ir":75000,"ts":12480}
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * BANDWIDTH BUDGET
 * ═══════════════════════════════════════════════════════════════════════════════
 * Worst-case packet: ~65 bytes (all fields at maximum value width).
 * At 100 Hz: 65 × 100 = 6500 bytes/s.
 * At 115200 baud (~11520 bytes/s usable): ~56% UART utilisation. Safe.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * TIMING INTERACTION WITH MAIN LOOP
 * ═══════════════════════════════════════════════════════════════════════════════
 * vitalsigns_update() is called at the end of every loop() iteration in
 * surgery_monitor.ino. It uses a millis()-based gate (INTERVAL_VS_MS = 10 ms)
 * so it fires at ~100 Hz regardless of how fast loop() runs.
 *
 * Serial3.print() is blocking on ATmega: at 115200 baud, each byte takes ~87 µs
 * to shift out. A 65-byte packet takes ~5.7 ms. Since this task runs at 100 Hz
 * (10 ms budget), ~57% of each budget period is consumed by the UART write.
 * The remaining tasks (MAX30102 FIFO drain, OLED update) must complete within
 * the ~4.3 ms remainder.
 *
 * If loop() exhibits timing jitter, increase INTERVAL_VS_MS (e.g., 20 ms = 50 Hz)
 * or remove the "ir" field if PPG waveform display in the browser is not needed.
 */

#include <Arduino.h>

// ── External variable declarations ───────────────────────────────────────────
// Defined in other .ino files in the same sketch folder.
// The Arduino IDE merges all .ino files before compiling.

extern volatile int  respRate;    // Respiratory rate (rpm)      — surgery_monitor.ino
extern volatile int  adcDisplay;  // Filtered respiratory sensor ADC value — surgery_monitor.ino
extern float         tempCelsius; // Core temperature (°C)       — surgery_monitor.ino
extern int           heartRate;   // Heart rate (BPM)            — surgery_monitor_MAX30102.ino
extern int           spO2;        // Oxygen saturation (%)       — surgery_monitor_MAX30102.ino
extern uint32_t      max30102_getIR();
extern float         g_TEMP_MIN_C;
extern float         g_TEMP_MAX_C;


// ── Streaming timing ─────────────────────────────────────────────────────────

static const uint8_t INTERVAL_VS_MS = 10;   // 10 ms → 100 Hz
static unsigned long timerVS        = 0;


// ════════════════════════════════════════════════════════════════════════════
//  vitalsigns_init()
//  Called once from setup() in surgery_monitor.ino.
//  Initialises Serial3 at 115200 baud to match the ESP8266 UART.
// ════════════════════════════════════════════════════════════════════════════

void vitalsigns_init()
{
    // Serial3 on ATmega2560: TX3 = pin 14, RX3 = pin 15.
    // On the RobotDyn Mega+WiFi these pins are bridged internally to the
    // ESP8266 UART — no external wiring required.
    Serial3.begin(115200);
    Serial.println(F("[OK] ESP8266 stream initialised on Serial3 @ 115200 baud."));
}


// ════════════════════════════════════════════════════════════════════════════
//  vitalsigns_update()
//  Called every loop() iteration. Non-blocking (millis()-gated).
//  Serialises all physiological parameters into a single JSON line and
//  transmits it over Serial3 to the ESP8266 at 100 Hz.
// ════════════════════════════════════════════════════════════════════════════

void vitalsigns_update()
{
    const unsigned long nowMs = millis();

    if (nowMs - timerVS < INTERVAL_VS_MS) return;
    timerVS += INTERVAL_VS_MS;

    // Atomic snapshot of ISR-written volatile variables.
    // adcDisplay and respRate are written by TIMER1_COMPA_vect — a torn read
    // on a multi-byte value would produce a corrupted sample.
    noInterrupts();
    const int safeRR    = respRate;
    const int safeResp  = adcDisplay;   // respiratory sensor ADC value
    interrupts();

    // heartRate, spO2, and tempCelsius are written only from the main loop —
    // no ISR touches them, so no critical section is needed here.
    const float safeTemp = tempCelsius;

    // ── JSON serialisation ────────────────────────────────────────────────────
    // Manual construction avoids the ArduinoJson library and its heap
    // allocation overhead. Fields are written in a single burst; the Serial3
    // hardware FIFO buffers the bytes during transmission.
    //
    // Temperature sentinel: transmit 0 when the probe is disconnected or the
    // reading is outside [g_TEMP_MIN_C, g_TEMP_MAX_C] (configurable via
    // the Python Settings panel). The browser dashboard displays "--.-".

    Serial3.print(F("{\"hr\":"));  Serial3.print(heartRate);
    Serial3.print(F(",\"sp\":"));  Serial3.print(spO2);
    Serial3.print(F(",\"rr\":"));  Serial3.print(safeRR);
    Serial3.print(F(",\"pz\":"));  Serial3.print(safeResp);

    Serial3.print(F(",\"t\":"));
    if (safeTemp >= g_TEMP_MIN_C && safeTemp <= g_TEMP_MAX_C) {
        Serial3.print(safeTemp, 1);   // One decimal place (0.1 °C resolution)
    } else {
        Serial3.print(0);             // Sentinel: probe disconnected or out of range
    }

    Serial3.print(F(",\"ir\":"));  Serial3.print(max30102_getIR());

    // "ts": millis() at serialisation time. The browser dashboard uses this
    // as the common X-axis for both waveform channels (IR and respiratory),
    // ensuring temporal alignment regardless of WebSocket delivery jitter.
    Serial3.print(F(",\"ts\":"));  Serial3.print(nowMs);

    Serial3.println(F("}"));   // '\n' terminates the packet for ESP8266 readline()
}
