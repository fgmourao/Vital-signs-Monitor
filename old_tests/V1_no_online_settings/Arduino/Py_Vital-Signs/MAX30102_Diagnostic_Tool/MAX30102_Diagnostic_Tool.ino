/**
 * @file    MAX30102_Diagnostic_Tool.ino
 * PROJECT: Vital-signs monitor — DIAGNOSTIC & PLOTTING TOOL
 * * OBJETIVO: Ferramenta exclusiva para visualização gráfica no Serial Plotter.
 * Use este arquivo para calibrar o LED e entender a matemática do SpO2.
 */

#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

MAX30105 particleSensor;

// ════════════════════════════════════════════════════════════════════════════
//  CHAVE SELETORA DE GRÁFICO (Mude aqui o que você quer ver no Serial Plotter)
// ════════════════════════════════════════════════════════════════════════════
//
//   1 = MODO ONDA: Desenha apenas o sinal IR bruto. 
//       (Use para ver seu coração batendo e ajustar a potência do LED).
//
//   2 = MODO SpO2: Desenha a "Mentira Estável" vs "Verdade Matemática".
//       (Use para testar o impacto do DECIM_RATIO).
//
#define PLOT_MODE 2

// ════════════════════════════════════════════════════════════════════════════
//  PARÂMETROS DE TESTE (Altere para simular diferentes cenários)
// ════════════════════════════════════════════════════════════════════════════

// Mude para 1 (Rápido, instável matematicamente) ou 4 (Correto para SpO2)
static const uint8_t DECIM_RATIO  = 1; 

// Brilho do LED (Mude para 50, 80 ou 120 conforme seu dedo)
static const uint8_t TEST_LED_PWR = 80; 

// ════════════════════════════════════════════════════════════════════════════

static const uint8_t SPO2_BLOCK   = 100;
static const uint8_t SPO2_KEEP    = 75;
static const uint8_t SPO2_REFILL  = 25;

static uint32_t irBuffer[SPO2_BLOCK];
static uint32_t redBuffer[SPO2_BLOCK];
static uint8_t  spo2FillIdx = 0;

static uint8_t  decimCount = 0;
static uint32_t sumRed     = 0;
static uint32_t sumIR      = 0;

int spO2_Display = 0;       // O valor que fica "preso" na tela
int statusRealAlgoritmo = 0; // 1 = Leu com sucesso, 0 = Falhou/Lixo

void setup()
{
    Serial.begin(115200);
    
    // Pequeno atraso para dar tempo de abrir o Serial Plotter
    delay(2000); 

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("ERRO: MAX30102 nao encontrado!");
        while (1);
    }

    // Setup focado em 100Hz limpos
    particleSensor.setup(TEST_LED_PWR, 4, 2, 400, 411, 16384);
    Wire.setClock(400000UL);
}

void loop()
{
    particleSensor.check();

    while (particleSensor.available())
    {
        uint32_t redRaw = particleSensor.getFIFORed();
        uint32_t irRaw  = particleSensor.getFIFOIR();
        particleSensor.nextSample();

        // Se tirar o dedo, zera tudo
        if (irRaw < 5000) {
            spO2_Display = 0;
            statusRealAlgoritmo = 0;
            spo2FillIdx = 0;
            decimCount = 0;
            sumRed = 0;
            sumIR = 0;
            
            // Se estiver no MODO 1, plota o zero para o gráfico cair
            if (PLOT_MODE == 1) Serial.println(0);
            
            continue; 
        }

        // ─── PLOTAGEM MODO 1: ONDA CARDÍACA BRUTA ────────────────────────
        if (PLOT_MODE == 1) {
            Serial.println(irRaw);
        }

        // ─── PROCESSAMENTO DO SpO2 ───────────────────────────────────────
        sumRed += redRaw;
        sumIR  += irRaw;
        decimCount++;

        if (decimCount >= DECIM_RATIO) {
            redBuffer[spo2FillIdx] = sumRed / DECIM_RATIO;
            irBuffer[spo2FillIdx]  = sumIR  / DECIM_RATIO;
            sumRed     = 0;
            sumIR      = 0;
            decimCount = 0;
            spo2FillIdx++;

            if (spo2FillIdx >= SPO2_BLOCK) {
                int8_t  validSpO2, validHR;
                int32_t tempSpO2, tempHR;

                maxim_heart_rate_and_oxygen_saturation(
                    irBuffer, SPO2_BLOCK, redBuffer,
                    &tempSpO2, &validSpO2,
                    &tempHR,   &validHR
                );

                // Captura a "Verdade" (Falhou ou Passou?)
                statusRealAlgoritmo = validSpO2;

                // Captura a "Mentira Estável" (Só atualiza a tela se for válido)
                if (validSpO2 == 1 && tempSpO2 >= 80 && tempSpO2 <= 100) {
                    spO2_Display = tempSpO2;
                }

                // ─── PLOTAGEM MODO 2: ANÁLISE DO ALGORITMO ───────────────
                if (PLOT_MODE == 2) {
                    Serial.print("Display_SpO2:");
                    Serial.print(spO2_Display);
                    Serial.print(","); // Vírgula separa as linhas no Plotter novo
                    Serial.print("Status_Real(0_ou_100):");
                    // Multiplica por 100 para o gráfico pular entre 0 e 100
                    Serial.println(statusRealAlgoritmo == 1 ? 100 : 0); 
                }

                // Desliza a janela
                for (uint8_t i = SPO2_REFILL; i < SPO2_BLOCK; i++) {
                    irBuffer[i  - SPO2_REFILL] = irBuffer[i];
                    redBuffer[i - SPO2_REFILL] = redBuffer[i];
                }
                spo2FillIdx = SPO2_KEEP;
            }
        }
    }
}