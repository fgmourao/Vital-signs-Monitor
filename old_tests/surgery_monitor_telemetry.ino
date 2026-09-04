/**
 * @file    surgery_monitor_Telemetry.ino
 * PROJECT: Vital-signs monitor — small rodents (rat / mouse)
 * @author  Flávio Mourão — Mar, 2026
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * THIS FILE — TELEMETRY ENGINE (Serial3 to ESP8266)
 * ═══════════════════════════════════════════════════════════════════════════════
 * Transmits a high-speed (100 Hz) JSON stream containing all physiological 
 * variables to the integrated ESP8266 via hardware Serial3.
 */

#include <Arduino.h>

// ── IMPORTANDO VARIÁVEIS DOS OUTROS ARQUIVOS ────────────────────────────────
extern volatile int   respRate;     // From surgery_monitor.ino
extern volatile int   adcDisplay;   // From surgery_monitor.ino
extern float          tempCelsius;  // CORRIGIDO: Sem 'volatile' e sem 'static' no arquivo original
extern int            heartRate;    // From surgery_monitor_MAX30102.ino
extern int            spO2;         // From surgery_monitor_MAX30102.ino
extern uint32_t       max30102_getIR(); // Acessor para a onda bruta

// ── CONSTANTES DA TELEMETRIA ────────────────────────────────────────────────
static const uint8_t INTERVAL_TELEMETRY_MS = 10; // 100 Hz
static unsigned long timerTelemetry = 0;

// ════════════════════════════════════════════════════════════════════════════
//  TELEMETRY INITIALIZATION
// ════════════════════════════════════════════════════════════════════════════

void telemetry_init()
{
    // A RobotDyn Mega+WiFi usa a Serial3 internamente para falar com o ESP8266.
    Serial3.begin(115200);
    Serial.println(F("[OK] Telemetry Engine initialized on Serial3 @ 115200 baud."));
}

// ════════════════════════════════════════════════════════════════════════════
//  MAIN TELEMETRY UPDATE — Call from main loop()
// ════════════════════════════════════════════════════════════════════════════

void telemetry_update()
{
    const unsigned long nowMs = millis();

    if (nowMs - timerTelemetry >= INTERVAL_TELEMETRY_MS) {
        timerTelemetry += INTERVAL_TELEMETRY_MS;

        // Atomic snapshot of volatile ISR variables
        noInterrupts();
        const int safeRR    = respRate;
        const int safePiezo = adcDisplay;
        interrupts();

        // Variáveis que rodam no loop não precisam de noInterrupts()
        const float safeTemp = tempCelsius; 

        // ── FORMATO DO JSON ──────────────────────────────
        // Exemplo: {"hr":350,"sp":98,"rr":85,"pz":512,"t":37.2,"ir":75000}
        
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
            Serial3.print(safeTemp, 1);
        } else {
            Serial3.print(0); // Fail-safe se o sensor cair
        }
        
        Serial3.print(F(",\"ir\":"));
        Serial3.print(max30102_getIR());
        
        Serial3.println(F("}"));
    }
}