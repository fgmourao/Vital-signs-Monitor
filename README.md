# Vital-signs Monitor v1.0
> **Under development**

Real-time, non-invasive physiological monitoring during experimental surgeries, stereotaxic procedures, and deep anaesthesia protocols in small rodents. ATmega2560 acquisition board controlled by a Python/PyQt5 application over USB, with simultaneous Wi-Fi browser dashboard via an on-board ESP8266.

---

## Overview

Four physiological parameters are acquired simultaneously:

- **Heart Rate** — MAX30102 optical sensor (IR channel), SparkFun `checkForBeat()` peak detector, rolling mean over N=6 beats
- **SpO2** — MAX30102 Red + IR channels, Maxim `spo2_algorithm` (sliding window: retain 75, refill 25 per update)
- **Respiratory Rate** — respiratory sensor at A0 (piezo film or FSR406 pressure sensor), software Schmitt trigger with adaptive envelope calibration every 2 s
- **Core Temperature** — NTC 100 kΩ rectal thermistor (A1), Beta equation conversion

All parameters are streamed at 100 Hz to the Python DAQ via USB-Serial and to a browser dashboard via WebSocket over Wi-Fi.

---

## Architecture

```
ATmega2560 (RobotDyn Mega+WiFi)
      │
      ├─ Serial0 (USB, 115200) ────────────────────────────────────────────────
      │        ↕ bidirectional JSON                                            │
      │   Py_VitalSigns_DAQ.py (PyQt5)                                        │
      │        outbound: {"hr":N,"sp":N,"rr":N,"pz":N,"t":N,"ir":N,"ts":N}   │
      │        inbound:  SOH + {"cmd":"set","key":"K","val":V}                 │
      │                  SOH + {"cmd":"get"}                                   │
      │                                                                        │
      ├─ Serial3 (pins 14/15, 115200) → ESP8266 (on-board) ──────────────────
      │        JSON 100 Hz stream → WebSocket → browser dashboard             │
      │                                                                        │
      ├─ I²C (pins 20/21)                                                     │
      │        ├─ MAX30102 optical sensor (address 0x57)                      │
      │        └─ SSD1306 OLED 128×32 (address 0x3C)                          │
      │                                                                        │
      ├─ A0 — Respiratory sensor (piezo film or FSR406 pressure sensor)       │
      ├─ A1 — NTC rectal thermistor                                            │
      │                                                                        │
      └─ Free-running ADC (~9.6 kHz) + TIMER1 CTC (100 Hz)                   │
               ISR decimation and Schmitt trigger for respiratory rate
```

---

## Signal Chain

### Heart Rate and SpO2

```
MAX30102 internal (400 Hz raw)
    └─ hardware averaging 4:1 → FIFO at 100 Hz
              │
              ├─ PATH 1 — Heart Rate
              │    checkForBeat(irRaw) at 100 Hz
              │    → _process_beat(): physiological gate + outlier rejection
              │    → rolling mean N=6 → heartRate (BPM)
              │
              └─ PATH 2 — SpO2
                   software decimation × g_DECIM_RATIO (default 1)
                   → irBuffer[100] + redBuffer[100]
                   → maxim_heart_rate_and_oxygen_saturation()
                   → sliding window (retain 75, refill 25) → spO2 (%)
```

### Respiratory Rate

```
Respiratory sensor (A0)
    └─ ADC_vect free-running ~9.6 kHz (ISR)
              └─ TIMER1_COMPA_vect 100 Hz CTC (ISR)
                       └─ N=4 moving average
                                └─ software Schmitt trigger
                                         └─ adaptive envelope calibration every 2 s
                                                  └─ respRate (rpm)
```

### Respiratory Sensor Signal Conditioning (Python)

The raw ADC counts from the respiratory sensor are processed in Python before display and CSV recording:

1. **Unit conversion** — counts → mV (10-bit ADC, 5 V reference: 1 count ≈ 4.89 mV)
2. **DC baseline removal** — IIR low-pass filter (α = 0.005 @ 100 Hz, fc ≈ 0.08 Hz) tracks slow drift from animal weight and posture. Time constant ~200 s.
3. **Polarity correction** — controlled by `_INVERT_RESP` constant:
   - `True` (default): FSR406 and piezo where inspiration causes a voltage drop
   - `False`: piezo where inspiration already produces a positive peak
4. **Resp. Tare** — forces immediate convergence of the DC estimate when the animal is placed on the sensor

The same processing is applied in the browser dashboard JavaScript.

### Temporal Alignment

Both `ir` and `pz` are packed in the same JSON packet. The `"ts"` field carries `millis()` at packet serialisation time and is used by the Python DAQ as the common X-axis for both waveforms, eliminating USB transport jitter (~1–15 ms) from the recorded data.

---

## EEPROM Persistence

All mutable parameters (`g_*` variables) are saved to EEPROM on every change and restored on power-up. The first boot after a fresh firmware upload uses compile-time defaults; subsequent boots load the last saved values.

EEPROM layout (ATmega2560, 4 KB available):

| Addr | Variable | Type | Bytes |
|---|---|---|---|
| 0 | Magic byte (0xA5) | uint8_t | 1 |
| 1 | g_HR_MIN_BPM | float | 4 |
| 5 | g_HR_MAX_BPM | float | 4 |
| 9 | g_HR_OUTLIER_FRAC | float | 4 |
| 13 | g_SPO2_MIN_VALID | uint8_t | 1 |
| 14 | g_DECIM_RATIO | uint8_t | 1 |
| 15 | g_LED_BRIGHTNESS | uint8_t | 1 |
| 16 | g_SAMPLE_AVERAGE | uint8_t | 1 |
| 17 | g_ADC_RANGE | uint32_t | 4 |
| 21 | g_CALIB_MIN_SWING | int | 4 |
| 25 | g_THRESH_INSP_FRAC | float | 4 |
| 29 | g_THRESH_EXP_FRAC | float | 4 |
| 33 | g_TEMP_MIN_C | float | 4 |
| 37 | g_TEMP_MAX_C | float | 4 |

---

## USB Protocol

Bidirectional JSON over Serial0 (115200 baud). Python prefixes every command with SOH (0x01) to halt the outbound 100 Hz stream before sending.

```
Outbound (100 Hz):
    {"hr":N,"sp":N,"rr":N,"pz":N,"t":N.N,"ir":N,"ts":N}

Inbound — set parameter:
    SOH + {"cmd":"set","key":"HR_MAX","val":500} + \n
    → {"ack":"HR_MAX","val":500.0000}
    → {"nak":"reason"}  (on failure)

Inbound — read all parameters:
    SOH + {"cmd":"get"} + \n
    → {"ack":"HR_MIN","val":40.0000}
       {"ack":"HR_MAX","val":800.0000}
       ... (one line per parameter)
```

### Configurable Parameters

| Key | Variable | Module | Description |
|---|---|---|---|
| `HR_MIN` | g_HR_MIN_BPM | MAX30102 | HR lower physiological gate (BPM) |
| `HR_MAX` | g_HR_MAX_BPM | MAX30102 | HR upper physiological gate (BPM) |
| `HR_OUTLIER` | g_HR_OUTLIER_FRAC | MAX30102 | Outlier rejection fraction (0.10–0.70) |
| `SPO2_MIN` | g_SPO2_MIN_VALID | MAX30102 | Minimum accepted SpO2 (%) |
| `DECIM_RATIO` | g_DECIM_RATIO | MAX30102 | SpO2 software decimation (1, 2, or 4) |
| `LED` | g_LED_BRIGHTNESS | MAX30102 | LED drive current (1–255) — reruns sensor setup |
| `ADC_RANGE` | g_ADC_RANGE | MAX30102 | ADC full-scale (2048/4096/8192/16384) — reruns setup |
| `SAMPLE_AVG` | g_SAMPLE_AVERAGE | MAX30102 | Hardware averaging per FIFO entry — reruns setup |
| `CALIB_SWING` | g_CALIB_MIN_SWING | monitor | Minimum respiratory sensor swing (mV) for valid calibration |
| `THRESH_INSP` | g_THRESH_INSP_FRAC | monitor | Inspiration threshold fraction (0.40–0.85) |
| `THRESH_EXP` | g_THRESH_EXP_FRAC | monitor | Expiration threshold fraction (0.20–0.70) |
| `TEMP_MIN` | g_TEMP_MIN_C | monitor | Temperature lower physiological gate (°C) |
| `TEMP_MAX` | g_TEMP_MAX_C | monitor | Temperature upper physiological gate (°C) |

---

## Wi-Fi Dashboard

The ESP8266 creates a Wi-Fi access point (`SSID: Vital_Signs_Web_Monitor`) and serves an HTML dashboard at `http://192.168.4.1`. A WebSocket server on port 81 forwards the 100 Hz JSON stream to connected browsers. The browser applies the same respiratory signal conditioning as the Python DAQ (mV conversion, DC removal, polarity correction).

Multi-client dispatch:
- **Primary client** (first to connect): receives every packet → 100 Hz
- **Secondary clients** (subsequent): receive 1 in 4 packets → 25 Hz

---

## Python Application

### Requirements

```
pip install pyserial PyQt5 pyqtgraph scipy numpy
```

Python 3.8 or later. Compatible with Spyder IDE.

### Features

- Real-time scrolling waveform display (IR-PPG and respiratory sensor)
- Numeric readouts: HR, SpO2, respiratory rate, core temperature
- Arduino timestamp (`"ts"`) used as the common X-axis — eliminates USB transport jitter from recorded data
- **Settings panel** — runtime adjustment of all sensor and algorithm parameters without recompilation; changes saved to EEPROM automatically; values auto-synced from Arduino on connect
- **Help panel** — parameter reference with species-specific suggested values and tooltips on every field
- **FFT viewer** — live power spectral density for both IR-PPG and respiratory channels (Welch method, configurable window and segment length)
- **Resp. Tare** — instant respiratory sensor baseline zeroing; cancels animal weight offset without waiting for IIR filter convergence
- **Event marker** — intra-session surgical event marking with visual timeline markers on both waveforms
- **CSV recording** split into two files:
  - `*_vitals.csv` — low-rate physiological values, wall-clock time axis
  - `*_raw.csv` — high-rate conditioned waveform samples, Arduino timestamp axis (`TS_Arduino_ms`)
- Pause / Resume without disconnecting

### CSV Output Format

**`*_vitals.csv`**

| Column | Description |
|---|---|
| Time_Seconds | Wall-clock elapsed time since REC start (s) |
| HeartRate_BPM | HR from MAX30102 ring buffer (BPM) |
| SpO2_% | SpO2 from Maxim algorithm (%) |
| RespRate_RPM | Respiratory rate from sensor detector (rpm) |
| Temperature_C | NTC rectal temperature (°C) |
| Event_Marker | "Event Start" on marked rows |

**`*_raw.csv`**

| Column | Description |
|---|---|
| Time_Seconds | Wall-clock elapsed time — for cross-reference only |
| TS_Arduino_ms | Arduino millis(), normalised to 0 at REC start — **use this as the time axis for signal analysis** |
| IR_Raw | Raw IR count from MAX30102 FIFO (counts, 18-bit) |
| Resp_mV | Respiratory sensor signal in mV — DC-removed and polarity-corrected; inspiration = positive peak. NOT the raw ADC value. |
| Event_Marker | 1 on event rows, 0 otherwise |

---

## Respiratory Sensor

The system is compatible with two sensor types connected to A0 via a voltage divider (R_fixed in series to VCC):

| Sensor type | R_fixed | Notes |
|---|---|---|
| FSR406 pressure sensor | 10 kΩ | Set `_INVERT_RESP = True`. Place under the animal. Target DC ~2500 mV with animal weight. |
| Piezo film sensor | Depends on interface circuit | Set `_INVERT_RESP` based on observed polarity. Inherently AC — Resp. Tare less critical. |

**Optimal operating point:** DC level ≈ 2500 mV (mid-range) with the animal positioned. If DC is below 1000 mV, reduce R_fixed. If above 4000 mV, increase R_fixed. Adjust Calib Min Swing (default 586 mV) downward for animals with shallow breathing.

---

## Species Reference Values

| Parameter | Human (bench) | Rat (anaest.) | Mouse (anaest.) |
|---|---|---|---|
| HR Min | 40 BPM | 200 BPM | 250 BPM |
| HR Max | 200 BPM | 500 BPM | 800 BPM |
| LED Brightness | 80 | 40–60 | 20–40 |
| ADC Range | 16384 | 4096 | 2048–4096 |
| Sample Average | 4 | 1–4 | 1–2 |

Target IR raw DC level: 50,000–130,000 counts (20–50% of ADC full-scale). Verify in the waveform plot after positioning the sensor.

---

## Design Tradeoffs

**[T1] DECIM_RATIO = 1**
The Maxim `spo2_algorithm.cpp` FIR filters were designed for 25 Hz input (DECIM_RATIO = 4). With DECIM_RATIO = 1 the algorithm receives 100 Hz. Human bench tests show physiologically correct SpO2 (97–99%). Performance at rodent cardiac frequencies (300–600 BPM) is untested — if systematic SpO2 overestimation (e.g. constant 100%) is observed, increase DECIM_RATIO to 4 via Settings.

**[T2] checkForBeat() with hardware-averaged signal**
The SparkFun `checkForBeat()` detector was calibrated for unaveraged signals. With `sampleAverage = 4` the AC component is smoothed before detection. If HR detection fails on rodents, reduce `sampleAverage` to 1 via Settings to restore full AC amplitude at 400 Hz.

**[T3] ADC Range**
`adcRange = 16384` is required for human fingertip at LED = 80 to avoid saturation (IR DC reaches 262,143 = 18-bit ceiling). For rodent thin tissue, try `adcRange = 4096` and verify IR raw in the waveform plot.

**[T4] IIR DC baseline convergence**
The respiratory signal DC filter has a time constant of ~200 s at 100 Hz. When the animal is first placed on the sensor, the display may appear shifted until the filter converges. Use **Resp. Tare** to force immediate convergence.

---

## Hardware Requirements

| Component | Specification |
|---|---|
| Microcontroller | RobotDyn Mega+WiFi (ATmega2560 + ESP8266) |
| Optical sensor | MAX30102 breakout — I²C, address 0x57, 3.3 V |
| Display | SSD1306 OLED 128×32 — I²C, address 0x3C |
| Respiratory sensor | Piezo film or FSR406 → A0 (voltage divider with R_fixed to VCC) |
| Temperature probe | NTC 100 kΩ rectal thermistor → A1 (series 100 kΩ to VCC) |
| USB | ATmega2560 USB-Serial bridge (Arduino IDE upload port) |

---

## Files

| File | Description |
|---|---|
| `surgery_monitor.ino` | Main sketch — ISRs, task scheduler, OLED display, monitor_set_param() |
| `surgery_monitor_MAX30102.ino` | MAX30102 HR + SpO2, EEPROM persistence, max30102_set_param() |
| `Py_Vital-Signs.ino` | USB bidirectional stream — 100 Hz outbound + SOH command parser |
| `surgery_monitor_web_server.ino` | Serial3 → ESP8266 JSON bridge at 100 Hz |
| `ESP8266_VitalSigns_Server.ino` | ESP8266 firmware — Wi-Fi AP, HTTP dashboard, WebSocket multi-client |
| `MAX30102_Diagnostic_Tool.ino` | Standalone diagnostic sketch — IR waveform and SpO2 algorithm analysis |
| `Py_VitalSigns_DAQ.py` | Python DAQ application (PyQt5 + pyqtgraph) |
| `fft_vitalsigns.py` | Live PSD viewer — Welch method for HR and respiratory spectral analysis |

Arduino library dependencies (install via Arduino IDE Library Manager):
- **SparkFun MAX3010x Pulse and Proximity Sensor Library**
- **U8g2** (Oliver Kraus)
- **WebSockets** (Markus Sattler) — ESP8266 only

---

## Known Limitations

- HR and SpO2 values are not validated against a calibrated reference instrument.
- SpO2 may read 100% consistently when DECIM_RATIO = 1 — increase to 4 via Settings if this occurs.
- The `_debug_serial()` diagnostic output is plain text and incompatible with the Python DAQ. Use only with the Arduino IDE Serial Monitor when the Python application is not connected.
- If the USB cable is disconnected during a recording session, the CSV files may be incomplete. Always stop recording before disconnecting.
- Primary Wi-Fi client assignment is first-connect order. If a secondary workstation connects before the primary, it will receive 25 Hz instead of 100 Hz.
- The Resp. Tare is session-only — it resets if the application is restarted or the connection is toggled.

---

## Author

**v1.0 (2026)**
Flavio Afonso Goncalves Mourao — [mourao.fg@gmail.com](mailto:mourao.fg@gmail.com)
*CNPq/MCTI/FNDCT Nº 21/2024 — Processo 446467/2024-3*
*Federal University of Minas Gerais, Brazil*

---

*Last update: September 2026*
