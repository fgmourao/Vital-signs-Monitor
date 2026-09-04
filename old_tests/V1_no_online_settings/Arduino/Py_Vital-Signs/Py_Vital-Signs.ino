/**
 * @file    Py_Vital-Signs.ino
 * PROJECT: Vital-signs monitor
 * @version 1.0
 * @author  Flávio Mourão — Mar, 2026
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * THIS FILE — USB DATA ENGINE (Serial0 to PC/Py_Vital-signs)
 * ═══════════════════════════════════════════════════════════════════════════════
 * Transmits a high-speed (100 Hz) JSON stream containing all physiological 
 * variables directly to the computer via the main USB cable for standalone 
 * recording and event marking.
 */

#include <Arduino.h>

// ── IMPORTANDO VARIÁVEIS DOS OUTROS ARQUIVOS ────────────────────────────────
extern volatile int   respRate;
extern volatile int   adcDisplay;
extern float          tempCelsius;
extern int            heartRate;
extern int            spO2;
extern uint32_t       max30102_getIR();

// ── CONSTANTES DA CONEXÃO USB ───────────────────────────────────────────────
static const uint8_t INTERVAL_USB_MS = 10; // 100 Hz
static unsigned long timerUSB = 0;

// ════════════════════════════════════════════════════════════════════════════
//  USB INITIALIZATION
// ════════════════════════════════════════════════════════════════════════════
void py_vital_signs_init()
{
    // A Serial0 (Cabo USB) já foi inicializada no main loop com 115200 baud,
    // mas podemos adicionar configurações específicas aqui no futuro se necessário.
    Serial.println(F("[OK] Py_Vital-signs initialized."));
}

// ════════════════════════════════════════════════════════════════════════════
//  USB UPDATE — Call from main loop()
// ════════════════════════════════════════════════════════════════════════════
void py_vital_signs_update()
{
    const unsigned long nowMs = millis();

    if (nowMs - timerUSB >= INTERVAL_USB_MS) {
        timerUSB += INTERVAL_USB_MS;

        noInterrupts();
        const int safeRR    = respRate;
        const int safePiezo = adcDisplay;
        interrupts();

        const float safeTemp = tempCelsius; 

        // ── FORMATO DO JSON (Compatível com Py_Vital-signs) ───────────────
        Serial.print(F("{\"hr\":"));
        Serial.print(heartRate);
        
        Serial.print(F(",\"sp\":"));
        Serial.print(spO2);
        
        Serial.print(F(",\"rr\":"));
        Serial.print(safeRR);
        
        Serial.print(F(",\"pz\":"));
        Serial.print(safePiezo);
        
        Serial.print(F(",\"t\":"));
        if (safeTemp >= 30.0f && safeTemp <= 45.0f) {
            Serial.print(safeTemp, 1);
        } else {
            Serial.print(0);
        }
        
        Serial.print(F(",\"ir\":"));
        Serial.print(max30102_getIR());
        
        Serial.println(F("}"));
    }
}