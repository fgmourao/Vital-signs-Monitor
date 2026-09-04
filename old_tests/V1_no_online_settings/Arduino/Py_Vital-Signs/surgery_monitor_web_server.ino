/**
 * @file    surgery_monitor_web_server.ino
 * PROJECT: Vital-signs monitor
 * @version 1.0
 * @author  Flávio Mourão — Mar, 2026
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * MODULE RESPONSIBILITY
 * ═══════════════════════════════════════════════════════════════════════════════
 * Transmits a 100 Hz JSON stream containing all physiological parameters to the
 * integrated ESP8266 via hardware Serial3. The ESP8266 forwards each packet to
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
 *         ↕  internal PCB traces
 *   ESP8266 Serial (GPIO1/GPIO3)
 *
 * Baud rate must match on both sides: 115200 baud is used here.
 * The DIP switch bank on the board controls which UART is routed to the USB
 * chip — set it for ESP8266 programming when flashing the ESP, and for Mega
 * operation during normal use. Consult the RobotDyn board documentation for
 * the exact switch configuration.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * JSON PACKET FORMAT
 * ═══════════════════════════════════════════════════════════════════════════════
 * One packet per 10 ms (100 Hz). Each packet is a single-line JSON object
 * terminated by '\n' (Serial3.println). The ESP8266 reads until '\n' and
 * broadcasts the complete object over WebSocket.
 *
 * Field map:
 *   "hr"  — Heart rate (BPM, int). 0 = no valid signal.
 *   "sp"  — SpO2 percentage (%, int). 0 = no valid signal.
 *   "rr"  — Respiratory rate (rpm, int). 0 = apnoea or no signal.
 *   "pz"  — Piezo ADC filtered value (counts, int). Used for waveform display.
 *           Range: 0–1023 (10-bit ADC). Calibrated envelope applies in browser.
 *   "t"   — Core temperature (°C, one decimal). 0 = probe disconnected.
 *   "ir"  — Raw IR photodetector count from MAX30102 (uint32). Used for PPG
 *           waveform display. Range: 0–262143 (18-bit hardware ceiling).
 *
 * Example packet:
 *   {"hr":350,"sp":98,"rr":85,"pz":512,"t":37.2,"ir":75000}
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * BANDWIDTH BUDGET
 * ═══════════════════════════════════════════════════════════════════════════════
 * Worst-case packet length: ~55 bytes (all fields at maximum value width).
 * At 100 Hz: 55 × 100 = 5500 bytes/s.
 * At 115200 baud (~11520 bytes/s usable): ~48% UART utilisation. Safe.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * TIMING INTERACTION WITH MAIN LOOP
 * ═══════════════════════════════════════════════════════════════════════════════
 * vitalsigns_update() is called at the end of every loop() iteration in
 * surgery_monitor.ino. It uses a millis()-based gate (INTERVAL_VS_MS = 10 ms)
 * so it fires at ~100 Hz regardless of how fast loop() runs.
 *
 * Serial3.print() is blocking on ATmega: at 115200 baud, each byte takes ~87 µs
 * to shift out. A 55-byte packet takes ~4.8 ms maximum. Since this task runs at
 * 100 Hz (10 ms budget), up to ~48% of each budget period is consumed by the
 * UART write. The remaining tasks (MAX30102 FIFO drain, OLED update) must
 * complete within the ~5.2 ms remainder per 10 ms window.
 *
 * If loop() exhibits timing jitter, reduce the packet rate by increasing
 * INTERVAL_VS_MS (e.g., 20 ms = 50 Hz) or shorten the JSON by removing the
 * "ir" field if PPG waveform display is not needed.
 */

#include <Arduino.h>

// ── External variable declarations ──────────────────────────────────────────
// These variables are defined in other .ino files in the same sketch folder.
// The Arduino IDE merges all .ino files before compiling; no #include needed.

extern volatile int  respRate;    // Respiratory rate (rpm) — surgery_monitor.ino
extern volatile int  adcDisplay;  // Filtered piezo ADC value — surgery_monitor.ino
extern float         tempCelsius; // Core temperature (°C)  — surgery_monitor.ino
extern int           heartRate;   // Heart rate (BPM)       — surgery_monitor_MAX30102.ino
extern int           spO2;        // Oxygen saturation (%)  — surgery_monitor_MAX30102.ino

// max30102_getIR() returns the last raw IR sample from the MAX30102 FIFO.
// Declared as extern function — defined in surgery_monitor_MAX30102.ino.
extern uint32_t max30102_getIR();


// ── Vital Signs Streaming Constants ─────────────────────────────────────────

// Streaming interval: 10 ms → 100 Hz.
// Increasing this value reduces UART load at the cost of lower temporal
// resolution in the browser waveform display.
static const uint8_t INTERVAL_VS_MS = 10;

// Timer state — tracks last transmission timestamp.
static unsigned long timerVS = 0;


// ════════════════════════════════════════════════════════════════════════════
//  vitalsigns_init()
//  Called once from setup() in surgery_monitor.ino.
//  Initialises Serial3 at 115200 baud to match the ESP8266 UART configuration.
// ════════════════════════════════════════════════════════════════════════════

void vitalsigns_init()
{
    // Serial3 on ATmega2560: TX3 = pin 14, RX3 = pin 15.
    // On the RobotDyn Mega+WiFi, these pins are bridged internally to the
    // ESP8266 UART. No external wiring is required.
    Serial3.begin(115200);
    Serial.println(F("[OK] Vital Signs Streaming initialised on Serial3 @ 115200 baud."));
}


// ════════════════════════════════════════════════════════════════════════════
//  vitalsigns_update()
//  Called every loop() iteration. Non-blocking (millis()-gated).
//  Serialises all physiological parameters into a single JSON line and
//  transmits it over Serial3 to the ESP8266 at INTERVAL_VS_MS rate.
// ════════════════════════════════════════════════════════════════════════════

void vitalsigns_update()
{
    const unsigned long nowMs = millis();

    if (nowMs - timerVS < INTERVAL_VS_MS) return;
    timerVS += INTERVAL_VS_MS;

    // Atomic snapshot of volatile ISR-written variables.
    // adcDisplay and respRate are written by TIMER1_COMPA_vect — a torn read
    // (ISR writing a multi-byte value while main loop reads it) would produce
    // a corrupted sample. noInterrupts() prevents this.
    noInterrupts();
    const int safeRR    = respRate;
    const int safePiezo = adcDisplay;
    interrupts();

    // heartRate, spO2, and tempCelsius are written only from the main loop
    // (max30102_update() and the temperature task). No ISR writes them,
    // so no critical section is needed for these reads.
    const float safeTemp = tempCelsius;

    // ── JSON serialisation ────────────────────────────────────────────────────
    // Manual construction avoids the ArduinoJson library dependency and the
    // associated heap allocation overhead on the ATmega2560.
    // All fields are written in a single burst; Serial3 FIFO buffers the bytes.
    //
    // Format: {"hr":NNN,"sp":NNN,"rr":NNN,"pz":NNN,"t":NN.N,"ir":NNNNNN,"ts":NNNNNN}
    //
    // Field "ts": ATmega2560 millis() timestamp at the moment this packet is
    // serialised. Unit: milliseconds, wraps at 2^32 ms (~49.7 days).
    //
    // PURPOSE — temporal alignment in the Python DAQ:
    //   The piezo signal (pz) is captured by TIMER1_COMPA_vect at exactly 100 Hz,
    //   while the IR signal (ir) is read from the MAX30102 FIFO in loop(), which
    //   has variable execution timing. Both values in a given packet were read
    //   within the same loop() iteration, so "ts" represents the common capture
    //   instant for that packet. The Python side uses "ts" as the X-axis for
    //   both signals, eliminating USB transport jitter (~1-15 ms) from the
    //   alignment. time.time() on the Python side reflects arrival time, not
    //   capture time — using it for waveform alignment produces the jitter you
    //   observed.
    //
    // BANDWIDTH NOTE: "ts" adds up to 8 bytes per packet (field name + 6-digit
    // value). Worst case: 63 bytes × 100 Hz = 6.3 kB/s — still well within
    // the 115200 baud (~11.5 kB/s) UART capacity.
    //
    // Temperature sentinel: if tempCelsius == 0.0f (probe open/shorted) or
    // outside [30, 45] °C, transmit 0 so the browser can display "--.-".

    Serial3.print(F("{\"hr\":"));
    Serial3.print(heartRate);

    Serial3.print(F(",\"sp\":"));
    Serial3.print(spO2);

    Serial3.print(F(",\"rr\":"));
    Serial3.print(safeRR);

    Serial3.print(F(",\"pz\":"));
    Serial3.print(safePiezo);

    Serial3.print(F(",\"t\":"));
    if (safeTemp >= 30.0f && safeTemp <= 45.0f) {
        Serial3.print(safeTemp, 1);   // One decimal place (0.1 °C resolution)
    } else {
        Serial3.print(0);             // Sentinel: probe disconnected or out of range
    }

    Serial3.print(F(",\"ir\":"));
    Serial3.print(max30102_getIR());  // Raw IR count for PPG waveform display

    // Capture timestamp — millis() at the moment of serialisation.
    // Used by the Python DAQ to align ir and pz on a common time axis,
    // removing USB transport jitter from the recorded waveforms.
    Serial3.print(F(",\"ts\":"));
    Serial3.print(nowMs);

    Serial3.println(F("}"));          // '\n' terminates the packet for ESP8266 readStringUntil('\n')
}
