"""
Py_VitalSigns_DAQ.py
====================
PROJECT : Vital-signs monitor
VERSION : 1.0
AUTHOR  : Flávio Mourão — Mar, 2026
Last update : Oct, 2026

════════════════════════════════════════════════════════════════════════════════
MODULE RESPONSIBILITY
════════════════════════════════════════════════════════════════════════════════
Standalone USB data-acquisition and real-time display application for the
rodent vital-signs monitor. Receives a 100 Hz JSON stream from the ATmega2560
(Py_Vital-Signs.ino) over USB-Serial and provides:

  - Scrolling waveforms (~3 s): IR-PPG and respiratory sensor
  - Numeric readouts: HR, SpO2, respiratory rate, core temperature
  - Event markers on both waveforms (while recording)
  - CSV recording in two files:
      *_vitals.csv — low-rate physiological values, wall-clock time axis
      *_raw.csv    — high-rate waveform samples, Arduino timestamp axis
  - Settings panel: runtime parameter adjustment over the bidirectional serial
    protocol, no recompilation required
  - Resp. Tare: instant zeroing of the respiratory baseline
  - Live PSD viewer (fft_vitalsigns.py) for heart-rate and respiratory-rate
    spectral analysis, fed by a ~60 s history
  - IR display filter (visual analysis only): Remove DC, configurable causal
    band-pass (HP / LP corners) and a "checkForBeat view" showing the signal as
    the firmware beat detector sees it. The PSD follows the displayed signal.
    CSV files always store the raw, unfiltered IR counts.

════════════════════════════════════════════════════════════════════════════════
TIME BASE
════════════════════════════════════════════════════════════════════════════════
Every packet carries "ts" (ATmega2560 millis() at serialisation), the common
time axis of both signals; the PC clock is not used for signal analysis.

  Resp. sensor (pz): 100 Hz Timer1 ISR (moving average of 4), latest value
                     read at serialisation.
  IR (ir):           latest MAX30102 FIFO sample (100 Hz for sample average 4),
                     read in loop(). When loop() is late a sample can repeat
                     (~10 % of packets) or be skipped; use the CSV for
                     morphology and rate, not for sample-exact timing.

Packets leave the Arduino every 10–13 ms. The PSD viewer therefore receives a
history linearly interpolated onto a uniform grid at the mean packet period
(Welch assumes uniform sampling). A time gap > 100 ms, a backwards "ts" (Arduino
reset on reconnect) or a Settings pause restarts that history.

All firmware filters (checkForBeat, SpO2) run per sample and assume the nominal
FIFO rate (400 Hz / sample average). The measured rate is shown next to the IR
filter controls ("fs ≈ N Hz"); a value far below 100 Hz makes the display
filter and any offline analysis meaningless.

════════════════════════════════════════════════════════════════════════════════
RESPIRATORY CONDITIONING  (Resp_mV)
════════════════════════════════════════════════════════════════════════════════
  1. counts -> mV (10-bit ADC, 5 V reference, 1 count ≈ 4.89 mV)
  2. slow IIR DC estimate (alpha 0.005 @ 100 Hz, fc ≈ 0.08 Hz, ~200 s)
  3. AC = DC - raw (_INVERT_RESP = True, FSR406: inspiration lowers the ADC
     value) or raw - DC (False), so inspiration is a positive deflection
  4. DC added back, so the trace stays positive at rest
  Resp. Tare sets the DC estimate to the mean of the last 50 samples
  (session-only).

════════════════════════════════════════════════════════════════════════════════
SETTINGS WORKFLOW
════════════════════════════════════════════════════════════════════════════════
  1. Settings -> adjust values -> Send to Arduino (display pauses meanwhile).
  2. Parameters are sent one at a time; each is applied and confirmed by ACK
     before the next one. CALIB_SWING is edited in mV and sent in ADC counts.
  3. Values are stored in the Arduino EEPROM and survive power cycles.
  4. On connect (after 1.5 s) Python sends {"cmd":"get"} to read the current
     firmware values into the Settings panel.
  Changing LED, ADC range, sample average or pulse width reconfigures the
  sensor (new FIFO rate and signal level).

════════════════════════════════════════════════════════════════════════════════
JSON PACKET FORMAT  (defined in Py_Vital-Signs.ino)
════════════════════════════════════════════════════════════════════════════════
  {"hr":NNN,"sp":NNN,"rr":NNN,"pz":NNN,"t":NN.N,"ir":NNNNNN,"ts":NNNNNN}

  hr  — heart rate (BPM, int). 0 = no valid signal.
  sp  — SpO2 (%, int). 0 = no valid signal; holds the last valid value.
  rr  — respiratory rate (rpm, int). 0 = apnoea or no signal.
  pz  — respiratory sensor ADC value (10-bit counts, 0–1023).
  t   — core temperature (deg C, one decimal). 0 = probe disconnected or
        outside the configured range.
  ir  — raw IR photodetector count from MAX30102 (uint32, 0–262143).
  ts  — ATmega2560 millis() at packet serialisation (ms, uint32).

BIDIRECTIONAL PROTOCOL  (Python -> Arduino, on demand)
  Set:     SOH(0x01) + {"cmd":"set","key":"KEY","val":VALUE} + newline
  Get all: SOH(0x01) + {"cmd":"get"} + newline
  ACK:     {"ack":"KEY","val":APPLIED_VALUE}
  NAK:     {"nak":"reason"}
  SOH makes the firmware suspend its outbound stream until the newline, so
  replies never interleave with data packets. Lines that are not JSON objects
  (boot text) are ignored.

════════════════════════════════════════════════════════════════════════════════
CSV OUTPUT FORMAT
════════════════════════════════════════════════════════════════════════════════
  *_vitals.csv
    Time_Seconds, HeartRate_BPM, SpO2_%, RespRate_RPM, Temperature_C,
    Event_Marker
    Time_Seconds is wall-clock time since REC start (PC clock).

  *_raw.csv
    Time_Seconds, TS_Arduino_ms, IR_Raw, Resp_mV, Event_Marker
    Use TS_Arduino_ms (Arduino clock, zeroed at REC start), not Time_Seconds,
    as the time axis for signal analysis.
    IR_Raw: raw MAX30102 counts, never filtered.
    Resp_mV: conditioned respiratory signal in mV (see above): polarity
             corrected, DC level retained, tare applied if clicked.
             NOT the raw ADC value.
    Pausing stops logging; the next sample after a pause follows a time gap.

════════════════════════════════════════════════════════════════════════════════
DEPENDENCIES
════════════════════════════════════════════════════════════════════════════════
  pyserial   >= 3.5   — pip install pyserial
  PyQt5      >= 5.15  — pip install PyQt5
  pyqtgraph  >= 0.13  — pip install pyqtgraph
  numpy, scipy        — filters and PSD
  fft_vitalsigns.py   — PSD viewer, same folder

  Compatible with Spyder IDE: uses QApplication.instance() to avoid creating
  a second QApplication when IPython already manages one.

════════════════════════════════════════════════════════════════════════════════
HARDWARE CONTEXT
════════════════════════════════════════════════════════════════════════════════
  Board  : RobotDyn Mega+WiFi (ATmega2560 + ESP8266 co-processor)
  USB    : ATmega2560 USB-Serial bridge, 115200 baud (must match the sketch)
  Stream : JSON lines at 100 Hz, terminated by newline
"""

import sys
import json
import time
import csv
import queue
from collections import deque
import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFileDialog, QFrame, QComboBox,
    QDialog, QDoubleSpinBox, QSpinBox, QGroupBox, QGridLayout,
    QCheckBox, QTextBrowser
)
from PyQt5.QtGui import QTextOption
from PyQt5.QtCore import QThread, QTimer, pyqtSignal, Qt
import pyqtgraph as pg
from fft_vitalsigns import FFTVitalSignsWindow

# 10-bit ADC, 5 V reference: 1 count = 5000/1023 mV
_ADC_MV = 5000.0 / 1023.0

# Respiratory polarity. True: inspiration lowers the raw ADC value (FSR406, or a
# piezo mounted so that inspiration is a valley). False: raw peak = inspiration.
_INVERT_RESP = True

# IR display filter (never written to CSV)
_IR_DC_FC_HZ     = 0.1    # "Remove DC" slow estimator corner
_IR_FILT_ORDER   = 3      # Butterworth order of each band-pass section
# SparkFun checkForBeat() 23-tap symmetric FIR (>>15)
_FW_FIR_HALF     = [172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012]
_FW_FIR_TAPS     = np.array(_FW_FIR_HALF + [4096] + _FW_FIR_HALF[::-1], dtype=np.int64)


# ════════════════════════════════════════════════════════════════════════════
# SERIAL READING THREAD
# ════════════════════════════════════════════════════════════════════════════

class SerialThread(QThread):
    """
    Owns the serial port: all reads and writes happen in run(). send() is
    thread-safe (queue). data_received is emitted for every valid JSON line
    (data packets and ACK/NAK).
    """

    data_received    = pyqtSignal(dict)
    connection_error = pyqtSignal(str)

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port      = port
        self.baudrate  = baudrate
        self.ser       = None
        self.running   = True
        self._tx_queue = queue.Queue()

    def send(self, cmd_dict):
        """Queue a command; prefixed with SOH so the firmware pauses its stream."""
        payload = '\x01' + json.dumps(cmd_dict, separators=(',', ':')) + '\n'
        self._tx_queue.put(payload)

    def _drain_tx(self):
        """
        Write queued commands. Per command: send SOH, wait 60 ms for the firmware
        to finish its current packet, drop the packets received meanwhile, then
        send the JSON body.
        """
        while not self._tx_queue.empty():
            try:
                payload = self._tx_queue.get_nowait()
                if payload and payload[0] == '\x01':
                    self.ser.write(b'\x01')
                    self.ser.flush()
                    time.sleep(0.060)
                    self.ser.reset_input_buffer()
                    self.ser.write(payload[1:].encode('utf-8'))
                else:
                    self.ser.write(payload.encode('utf-8'))
                self.ser.flush()
            except queue.Empty:
                break

    def _read_lines(self):
        """Emit data_received for every valid JSON line."""
        while self.ser.in_waiting > 0:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode('utf-8', errors='ignore').strip()
            if text.startswith('{') and text.endswith('}'):
                try:
                    self.data_received.emit(json.loads(text))
                except json.JSONDecodeError:
                    pass

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            time.sleep(1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            while self.running:
                self._drain_tx()
                self._read_lines()
                time.sleep(0.01)

        except Exception as e:
            self.connection_error.emit(str(e))
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def stop(self):
        self.running = False


# ════════════════════════════════════════════════════════════════════════════
# HELP DIALOG
# ════════════════════════════════════════════════════════════════════════════

class HelpDialog(QDialog):
    """Parameter reference, opened from the Settings dialog."""

    HELP_TEXT = """
<style>
    body  { background:#1a1a1a; color:#cccccc; font-family:'Segoe UI',sans-serif;
            font-size:13px; margin:16px; }
    h2    { color:#44aaff; font-size:14px; margin-top:18px; margin-bottom:4px; }
    ul    { margin:0 0 8px 16px; padding:0; }
    li    { margin-bottom:3px; line-height:1.4; }
    span.key  { color:#ffffff; font-weight:bold; }
    span.unit { color:#888888; font-size:11px; }
</style>

<h2>Heart Rate</h2>
<ul>
  <li><span class="key">HR Min</span> — Lower physiological gate.
      Beats slower than this are rejected as artefacts.
      <span class="unit">Human: 40 BPM | Rat: 200 BPM | Mouse: 250 BPM</span></li>
  <li><span class="key">HR Max</span> — Upper physiological gate.
      Beats faster than this are rejected as artefacts.
      <span class="unit">Human: 200 BPM | Rat: 500 BPM | Mouse: 800 BPM</span></li>
  <li><span class="key">Outlier Frac</span> — Maximum allowed deviation from the rolling mean
      before a beat is discarded (e.g. 0.35 = 35%).
      Lower values give cleaner data in stable conditions;
      higher values tolerate rapid rate changes during induction/recovery.</li>
</ul>

<h2>SpO2</h2>
<ul>
  <li><span class="key">Min Valid SpO2</span> — SpO2 values below this threshold are rejected
      as physiologically implausible and not displayed.
      <span class="unit">Healthy anaesthetised animal: ≥ 95%</span></li>
  <li><span class="key">Decim Ratio</span> — Software decimation applied before the Maxim SpO2
      algorithm (1, 2, or 4). Value 1 = no decimation (recommended).
      Higher values slow SpO2 updates and may attenuate the AC signal.</li>
</ul>

<h2>Sensor Setup</h2>
<ul>
  <li><span class="key">LED Brightness</span> — LED drive current sent to the MAX30102
      (raw value 1–255, not a percentage). Higher = more light = higher IR DC level.
      Target IR raw ≈ 50 000–130 000 counts. Verify in the waveform plot.
      <span class="unit">Human fingertip: 80 | Rodent paw/tail: start at 40</span></li>
  <li><span class="key">ADC Range</span> — Full-scale range of the MAX30102 photodetector ADC
      (2048 / 4096 / 8192 / 16384 nA). Larger range = higher saturation threshold.
      <span class="unit">Human: 16384 | Rodent thin tissue: try 4096</span></li>
  <li><span class="key">Sample Average</span> — Hardware averaging per FIFO entry
      (1 / 2 / 4 / 8 / 16 / 32). Higher = smoother but lower temporal resolution.
      Effective FIFO rate = 400 Hz ÷ Sample Average.
      <span class="unit">Default: 4 → 100 Hz effective</span></li>
</ul>

<h2>Respiratory Rate</h2>
<ul>
  <li><span class="key">Calib Min Swing</span> — Minimum respiratory sensor signal swing (mV)
      required for calibration to be valid. Increase in noisy environments;
      decrease for animals with shallow breathing.
      <span class="unit">Range: 50–1950 mV | Default: 586 mV ≈ 120 ADC counts</span></li>
  <li><span class="key">Insp Threshold</span> — Fraction of the signal envelope at which
      an inspiration is detected (Schmitt trigger upper threshold).
      Must be &gt; Exp Threshold + 0.05.
      <span class="unit">Default: 0.65</span></li>
  <li><span class="key">Exp Threshold</span> — Fraction of the signal envelope at which
      the detector resets (Schmitt trigger lower threshold).
      The gap between Insp and Exp determines noise rejection.
      <span class="unit">Default: 0.45</span></li>
</ul>

<h2>Temperature</h2>
<ul>
  <li><span class="key">Temp Min</span> — Values below this are treated as probe disconnected
      or shorted. Display shows '--'.
      <span class="unit">Normal rodent range: 36.5–38.5 °C</span></li>
  <li><span class="key">Temp Max</span> — Values above this are treated as probe disconnected
      or open circuit. Display shows '--'.</li>
</ul>
"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Parameter Reference")
        self.setMinimumWidth(700)
        self.setMinimumHeight(620)
        self.resize(720, 660)
        self.setStyleSheet("QDialog { background-color: #1a1a1a; }")

        layout = QVBoxLayout(self)

        browser = QTextBrowser()
        browser.setOpenExternalLinks(False)
        browser.setStyleSheet(
            "QTextBrowser { background:#1a1a1a; border:none; padding:8px; }")
        browser.setWordWrapMode(QTextOption.WrapAtWordBoundaryOrAnywhere)
        browser.setHtml(self.HELP_TEXT)
        layout.addWidget(browser)

        btn_close = QPushButton("Close")
        btn_close.setStyleSheet(
            "background-color:#333; color:#fff; padding:8px 20px; "
            "font-weight:bold; border:none; border-radius:4px;")
        btn_close.clicked.connect(self.accept)
        layout.addWidget(btn_close, alignment=Qt.AlignRight)


# ════════════════════════════════════════════════════════════════════════════
# SETTINGS DIALOG
# ════════════════════════════════════════════════════════════════════════════

class SettingsDialog(QDialog):
    """
    Runtime parameter panel. Parameters are sent one at a time (the next after
    the ACK of the previous). The data stream keeps being read while the dialog
    is open; display updates are paused.
    """

    # (label, key, type, min, max, step, decimals, default, tooltip)

    PARAMS = {
        "Heart Rate": [
            ("HR Min (BPM)",        "HR_MIN",     "float", 20,   780,  10,   0, 40,
             "Lower physiological gate. Beats slower than this are rejected.\n"
             "Human: 40 | Rat: 200 | Mouse: 250"),
            ("HR Max (BPM)",        "HR_MAX",     "float", 50,   800,  10,   0, 200,
             "Upper physiological gate. Beats faster than this are rejected.\n"
             "Human: 200 | Rat: 500 | Mouse: 800"),
            ("Outlier Frac",        "HR_OUTLIER", "float", 0.10, 0.70, 0.05, 2, 0.35,
             "Maximum deviation from rolling mean before a beat is discarded.\n"
             "0.35 = 35%. Lower = cleaner data; higher = tolerates rapid rate changes."),
        ],
        "SpO2": [
            ("Min Valid SpO2 (%)",  "SPO2_MIN",   "float", 50,   95,   5,    0, 80,
             "SpO2 values below this threshold are rejected as implausible.\n"
             "Healthy anaesthetised animal: ≥ 95%"),
            ("Decim Ratio",         "DECIM_RATIO","int",   1,    4,    1,    0, 1,
             "Software decimation before the Maxim SpO2 algorithm (1, 2, or 4).\n"
             "1 = no decimation (recommended). Higher values slow SpO2 updates."),
        ],
        "Sensor Setup": [
            ("LED Brightness",      "LED",        "int",   10,   255,  10,   0, 80,
             "MAX30102 LED drive current (raw value 0–255, not a percentage).\n"
             "Target IR raw ≈ 50 000–130 000 counts. Verify in the waveform plot.\n"
             "Human fingertip: 80 | Rodent paw/tail: start at 40"),
            ("ADC Range",           "ADC_RANGE",  "int",   2048, 16384,2048, 0, 16384,
             "MAX30102 photodetector ADC full-scale (2048 / 4096 / 8192 / 16384 nA).\n"
             "Larger range = higher saturation threshold.\n"
             "Human: 16384 | Rodent thin tissue: try 4096"),
            ("Sample Average",      "SAMPLE_AVG", "int",   1,    32,   1,    0, 4,
             "Hardware averaging per FIFO entry (1 / 2 / 4 / 8 / 16 / 32).\n"
             "Effective FIFO rate = 400 Hz ÷ Sample Average.\n"
             "Default 4 → 100 Hz. Reduce to 1 if HR detection fails on rodents."),
            ("Pulse Width (µs)",    "PULSE_WIDTH","int",   69,   411,  1,    0, 215,
             "LED pulse width per ADC conversion (69 / 118 / 215 / 411 µs).\n"
             "Longer = higher SNR and ADC resolution but more power consumption.\n"
             "For rodent thin tissue with weak AC signal, try 411 µs.\n"
             "Valid values snap to nearest: 69, 118, 215, 411."),
        ],
        "Respiratory Rate": [
            ("Calib Min Swing (mV)", "CALIB_SWING","float", 50,   1950, 50,   0, 586,
             "Minimum respiratory sensor signal swing (mV) for calibration to be valid.\n"
             "Decrease for animals with shallow breathing. Increase in noisy environments.\n"
             "586 mV ≈ 120 ADC counts (10-bit, 5 V reference)  |  Sensor-independent."),
            ("Insp Threshold",      "THRESH_INSP","float", 0.40, 0.85, 0.05, 2, 0.65,
             "Schmitt trigger upper threshold — fraction of the signal envelope\n"
             "at which an inspiration event is detected (0.0–1.0).\n"
             "Must be > Exp Threshold + 0.05. Default: 0.65"),
            ("Exp Threshold",       "THRESH_EXP", "float", 0.20, 0.70, 0.05, 2, 0.45,
             "Schmitt trigger lower threshold — fraction of the signal envelope\n"
             "at which the detector resets after an inspiration (0.0–1.0).\n"
             "The gap (Insp − Exp) determines noise rejection. Default: 0.45"),
        ],
        "Temperature": [
            ("Temp Min (°C)",       "TEMP_MIN",   "float", 20,   35,   1,    1, 30,
             "Readings below this are treated as probe disconnected or shorted.\n"
             "Normal rodent rectal temperature: 36.5–38.5 °C"),
            ("Temp Max (°C)",       "TEMP_MAX",   "float", 35,   50,   1,    1, 45,
             "Readings above this are treated as probe open circuit.\n"
             "Normal rodent rectal temperature: 36.5–38.5 °C"),
        ],
    }

    def __init__(self, serial_thread, param_cache, daq_window, parent=None):
        super().__init__(parent)
        self.thread  = serial_thread
        self.cache   = param_cache
        self.daq     = daq_window
        self.widgets = {}
        self._pending = []   # Keys waiting for ACK

        # Pause display updates while dialog is open.
        self._was_paused = daq_window.is_paused if daq_window else False
        if daq_window and not daq_window.is_paused:
            daq_window.is_paused = True

        self.setWindowTitle("Sensor & Algorithm Settings")
        self.setMinimumWidth(480)
        self.setStyleSheet("""
            QDialog    { background-color: #1a1a1a; color: #ffffff; }
            QGroupBox  { color: #aaaaaa; border: 1px solid #333;
                         border-radius: 4px; margin-top: 8px; padding-top: 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; color: #888; }
            QLabel     { color: #cccccc; }
            QDoubleSpinBox, QSpinBox {
                background-color: #2a2a2a; color: #ffffff;
                border: 1px solid #444; border-radius: 3px;
                padding: 3px; min-width: 80px; }
            QPushButton { padding: 6px 16px; border-radius: 4px;
                          font-weight: bold; border: none; }
        """)

        main_layout = QVBoxLayout(self)

        for group_name, params in self.PARAMS.items():
            group = QGroupBox(group_name)
            grid  = QGridLayout(group)
            grid.setColumnStretch(1, 1)
            for row, (label, key, wtype, mn, mx, step, dec, default, tooltip) in enumerate(params):
                cached = self.cache.get(key, default)
                if wtype == "int":
                    spin = QSpinBox()
                    spin.setRange(int(mn), int(mx))
                    spin.setSingleStep(int(step))
                    spin.setValue(int(cached))
                else:
                    spin = QDoubleSpinBox()
                    spin.setRange(float(mn), float(mx))
                    spin.setSingleStep(float(step))
                    spin.setDecimals(dec)
                    spin.setValue(float(cached))
                # No suffix — units are shown in the label (in parentheses).
                if tooltip:
                    spin.setToolTip(tooltip)
                self.widgets[key] = spin
                lbl = QLabel(label)
                if tooltip:
                    lbl.setToolTip(tooltip)
                grid.addWidget(lbl,  row, 0)
                grid.addWidget(spin, row, 1)
            main_layout.addWidget(group)

        self.lbl_status = QLabel("")
        self.lbl_status.setStyleSheet("color: #888; font-size: 11px; padding: 4px;")
        self.lbl_status.setWordWrap(True)
        main_layout.addWidget(self.lbl_status)

        btn_layout = QHBoxLayout()
        self.btn_send = QPushButton("▶  Send to Arduino")
        self.btn_send.setStyleSheet("background-color: #0055a4; color: #fff; padding: 10px;")
        self.btn_send.clicked.connect(self._send_all)
        btn_help = QPushButton("?  Help")
        btn_help.setStyleSheet("background-color: #1a3a1a; color: #6c6; padding: 10px; "
                               "font-weight: bold; border: none; border-radius: 4px;")
        btn_help.clicked.connect(self._open_help)
        btn_close = QPushButton("Close")
        btn_close.setStyleSheet("background-color: #333; color: #fff;")
        btn_close.clicked.connect(self._close)
        btn_layout.addWidget(self.btn_send, 3)
        btn_layout.addWidget(btn_help,      1)
        btn_layout.addWidget(btn_close,     1)
        main_layout.addLayout(btn_layout)

        if self.thread is None:
            self.btn_send.setEnabled(False)
            self.lbl_status.setText("Not connected.")

    def _send_all(self):
        """Send all parameters sequentially, each after the ACK of the previous one."""
        if self.thread is None or not self.thread.running:
            self.lbl_status.setText("Not connected.")
            return

        self._send_queue = []
        for params in self.PARAMS.values():
            for row in params:
                key = row[1]
                val = self.widgets[key].value()
                self._send_queue.append((key, val))

        self._pending = [key for key, _ in self._send_queue]
        self.btn_send.setEnabled(False)
        self.lbl_status.setText("Sending...")

        self._send_next()

    def _send_next(self):
        """Send the next queued command. CALIB_SWING is edited in mV, sent in ADC counts."""
        if not hasattr(self, '_send_queue') or not self._send_queue:
            return
        key, val = self._send_queue.pop(0)
        send_val = round(val / _ADC_MV) if key == "CALIB_SWING" else val
        self.thread.send({"cmd": "set", "key": key, "val": send_val})

    def on_ack(self, resp):
        """Handle an ACK/NAK: update status and cache, then send the next command."""
        ok  = 'ack' in resp
        key = resp.get('ack') if ok else resp.get('nak', '?')

        if ok and self.daq and key in self.widgets:
            self.daq.param_cache[key] = self.widgets[key].value()

        if key in self._pending:
            self._pending.remove(key)

        symbol = "✓" if ok else "✗"
        current = self.lbl_status.text().replace("Sending...", "").strip()
        self.lbl_status.setText(f"{current}  {symbol}{key}".strip())

        if self._pending:
            self._send_next()
        else:
            self.btn_send.setEnabled(True)

    def _open_help(self):
        dlg = HelpDialog(parent=self)
        dlg.exec_()

    def _resume_daq(self):
        if self.daq and not self._was_paused:
            self.daq.is_paused = False

    def _close(self):
        self._pending = []
        self.btn_send.setEnabled(True)
        self._resume_daq()
        self.accept()

    def closeEvent(self, event):
        self._resume_daq()
        event.accept()


# ════════════════════════════════════════════════════════════════════════════
# MAIN APPLICATION WINDOW
# ════════════════════════════════════════════════════════════════════════════

class PyVitalSignsDAQ(QMainWindow):
    """Main window: UI, serial thread lifecycle, waveform buffers, CSV writers, event markers."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Py Vital-signs — USB Monitor")
        self.resize(1200, 750)
        self.setStyleSheet("background-color: #000000; color: #FFFFFF;")
        self.setAttribute(Qt.WA_DeleteOnClose)

        # ── Recording state ───────────────────────────────────────────────────
        self.is_paused    = False
        self.is_recording = False
        self.event_flag   = False

        self.csv_vitals  = None
        self.csv_raw     = None
        self.file_vitals = None
        self.file_raw    = None
        self.start_time  = 0

        # Arduino "ts" of the first packet after REC start (zeroes TS_Arduino_ms)
        self.ts_origin_arduino = None
        self.ts_origin_wall    = None

        # Display buffers (max_points long, oldest dropped on append)
        self._PLOT_WARMUP = 150
        self._buffer_fill = self._PLOT_WARMUP
        self.max_points = 300           # ~3 s at 100 Hz
        # Long history for the PSD (~60 s); the four deques are appended together:
        #   fft_ts (ms) | fft_raw (raw IR) | fft_ir (IR as displayed) | fft_pz (mV)
        self._FFT_BUF_N = 6000
        self.fft_ts  = deque(maxlen=self._FFT_BUF_N)
        self.fft_raw = deque(maxlen=self._FFT_BUF_N)
        self.fft_ir  = deque(maxlen=self._FFT_BUF_N)
        self.fft_pz  = deque(maxlen=self._FFT_BUF_N)
        self.ir_data     = [0] * self.max_points
        self.pz_data     = [0] * self.max_points
        self.pz_raw_data = [0] * self.max_points   # raw mV (used by Resp. Tare)
        self._pz_dc      = None                    # DC estimate, set on first packet
        # X axis: Arduino ts (ms); 10 ms placeholder ramp until packets arrive.
        self.ts_data    = list(range(0, self.max_points * 10, 10))

        # Event markers: (InfiniteLine, ts_ms), pruned when scrolled out of view
        self.event_lines_ir = []
        self.event_lines_pz = []

        # {key: firmware value}, filled by {"cmd":"get"} on connect
        self.param_cache = {}

        self.serial_thread = None
        self.settings_dlg  = None
        self.fft_dlg       = None

        # IR display-filter state
        self._ir_dirty   = True    # rebuild on the next packet
        self._ir_sos     = None    # band-pass SOS (None = bypass)
        self._ir_zi      = None
        self._ir_dc      = None    # slow DC estimate
        self._ir_fs      = 100.0   # sampling rate of the current design
        self._ir_fs_cnt  = 0
        self._fw_avg     = 0       # checkForBeat DC register
        self._fw_buf     = deque([0] * 23, maxlen=23)
        self._ir_init    = False

        self.init_ui()

    # ── UI ────────────────────────────────────────────────────────────────────

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ── Top bar ───────────────────────────────────────────────────────────
        top_bar = QHBoxLayout()

        self.combo_ports = QComboBox()
        self.combo_ports.setStyleSheet(
            "background-color: #222; color: #fff; padding: 5px; font-size: 14px;")
        self.refresh_ports()

        self.btn_connect = QPushButton("CONNECT")
        self.btn_connect.setStyleSheet(
            "background-color: #0055a4; color: #fff; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.btn_connect.clicked.connect(self.toggle_connection)

        self.btn_settings = QPushButton("⚙ SETTINGS")
        self.btn_settings.setStyleSheet(
            "background-color: #333; color: #aaa; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.btn_settings.setEnabled(False)
        self.btn_settings.clicked.connect(self.open_settings)

        self.btn_fft = QPushButton("FFT")
        self.btn_fft.setStyleSheet(
            "background-color: #333; color: #aaa; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.btn_fft.setEnabled(False)
        self.btn_fft.setToolTip("Open live Power Spectral Density viewer\n"
                                "(IR-PPG red, Respiratory gold)")
        self.btn_fft.clicked.connect(self.open_fft)

        self.btn_tare = QPushButton("Resp. Tare")
        self.btn_tare.setStyleSheet(
            "background-color: #333; color: #aaa; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.btn_tare.setEnabled(False)
        self.btn_tare.setToolTip(
            "Zero the respiratory sensor baseline.\n"
            "Place the animal on the sensor first, then click.\n"
            "The resting offset is subtracted immediately so only\n"
            "respiratory variations are displayed.")
        self.btn_tare.clicked.connect(self.resp_tare)

        self.lbl_init_msg = QLabel("")
        self.lbl_init_msg.setStyleSheet(
            "color: #cccccc; font-size: 16px; font-weight: bold; margin-left: 30px;")

        self.btn_event = QPushButton("MARK EVENT")
        self.btn_event.setStyleSheet(
            "background-color: #555; color: #aaa; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.btn_event.setEnabled(False)
        self.btn_event.clicked.connect(self.mark_event)

        self.btn_play_pause = QPushButton("⏸ PAUSE")
        self.btn_play_pause.setStyleSheet(
            "background-color: #333; color: #fff; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.btn_play_pause.clicked.connect(self.toggle_pause)

        self.btn_record = QPushButton("⏺ REC")
        self.btn_record.setStyleSheet(
            "background-color: #500; color: #fff; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.btn_record.clicked.connect(self.toggle_record)

        self.lbl_rec_status = QLabel("")
        self.lbl_rec_status.setStyleSheet(
            "color: #ff4444; font-weight: bold; font-size: 14px;")

        top_bar.addWidget(QLabel("USB Port:"))
        top_bar.addWidget(self.combo_ports)
        top_bar.addWidget(self.btn_connect)
        top_bar.addWidget(self.btn_settings)
        top_bar.addWidget(self.btn_fft)
        top_bar.addWidget(self.btn_tare)
        top_bar.addWidget(self.lbl_init_msg)
        top_bar.addStretch()
        top_bar.addWidget(self.btn_event)
        top_bar.addWidget(self.lbl_rec_status)
        top_bar.addWidget(self.btn_play_pause)
        top_bar.addWidget(self.btn_record)
        main_layout.addLayout(top_bar)

        # ── Main body ─────────────────────────────────────────────────────────
        body_layout = QHBoxLayout()
        main_layout.addLayout(body_layout)

        sidebar = QVBoxLayout()
        sidebar.setContentsMargins(0, 0, 10, 0)
        self.val_hr   = self.create_box(sidebar, "Heart Rate",  "bpm", "#b30000")
        self.val_spo2 = self.create_box(sidebar, "SpO2",        "%",   "#cc6600")
        self.val_rr   = self.create_box(sidebar, "Resp. Rate",  "rpm", "#cca300")
        self.val_temp = self.create_box(sidebar, "Temperature", "deg C", "#800080")
        body_layout.addLayout(sidebar, stretch=1)

        charts_layout = QVBoxLayout()

        charts_layout.addLayout(self._build_ir_controls())

        self.plot_ir = pg.PlotWidget(
            title='<span style="color:#b30000;font-size:11pt;font-weight:bold;">'
                  'Photoplethysmography</span>')
        self.plot_ir.setBackground('#0d0d0d')
        self.plot_ir.showGrid(x=True, y=True, alpha=0.2)
        self.plot_ir.getAxis('bottom').setLabel('Time (ms)')
        self.curve_ir = self.plot_ir.plot(pen=pg.mkPen('#b30000', width=2))
        charts_layout.addWidget(self.plot_ir)

        self.plot_pz = pg.PlotWidget(
            title='<span style="color:#cca300;font-size:11pt;font-weight:bold;">'
                  'Respiratory Excursions</span>')
        self.plot_pz.setBackground('#0d0d0d')
        self.plot_pz.showGrid(x=True, y=True, alpha=0.2)
        self.plot_pz.getAxis('bottom').setLabel('Time (ms)')
        self.plot_pz.getAxis('left').setLabel('mV')
        self.curve_pz = self.plot_pz.plot(pen=pg.mkPen('#cca300', width=2))
        self.plot_pz.setXLink(self.plot_ir)
        charts_layout.addWidget(self.plot_pz)

        body_layout.addLayout(charts_layout, stretch=4)

    # ── IR display filter (visual only) ───────────────────────────────────────

    def _build_ir_controls(self):
        """Control bar above the IR plot. Processing is causal (sample by sample)."""
        style = ("QCheckBox, QLabel { color:#cccccc; font-size:12px; }"
                 "QDoubleSpinBox { background:#2a2a2a; color:#fff; "
                 "border:1px solid #444; border-radius:3px; padding:2px; }")
        box = QHBoxLayout()

        self.chk_ir_dc = QCheckBox("Remove DC")
        self.chk_ir_dc.setToolTip(
            "Subtract the slow IR baseline (IIR, fc ≈ 0.1 Hz).\n"
            "Respiratory and cardiac components are preserved.")

        self.chk_ir_bp = QCheckBox("Band-pass")
        self.chk_ir_bp.setToolTip(
            "Causal Butterworth filter (3rd order per section).\n"
            "HP = high-pass corner, LP = low-pass corner. 'off' = section bypassed.")

        self.spin_ir_hp = QDoubleSpinBox()
        self.spin_ir_hp.setRange(0.0, 20.0); self.spin_ir_hp.setSingleStep(0.5)
        self.spin_ir_hp.setDecimals(1);      self.spin_ir_hp.setValue(3.5)
        self.spin_ir_hp.setSpecialValueText("off")
        self.spin_ir_hp.setToolTip("High-pass corner (Hz). Rat cardiac ≈ 4–9 Hz.")

        self.spin_ir_lp = QDoubleSpinBox()
        self.spin_ir_lp.setRange(0.0, 50.0); self.spin_ir_lp.setSingleStep(0.5)
        self.spin_ir_lp.setDecimals(1);      self.spin_ir_lp.setValue(12.0)
        self.spin_ir_lp.setSpecialValueText("off")
        self.spin_ir_lp.setToolTip("Low-pass corner (Hz). Limited to 0.45 × fs.")

        self.chk_ir_fw = QCheckBox("checkForBeat view")
        self.chk_ir_fw.setToolTip(
            "Shows the signal exactly as the firmware beat detector sees it:\n"
            "DC estimator (α = 1/16) + 23-tap FIR low-pass (110 ms delay at 100 Hz).\n"
            "Overrides the other controls. Rising zero crossings = beats.")

        self.lbl_ir_filt = QLabel("")
        self.lbl_ir_filt.setStyleSheet("color:#888; font-size:11px;")

        for w in (self.chk_ir_dc, self.chk_ir_bp, self.spin_ir_hp,
                  self.spin_ir_lp, self.chk_ir_fw):
            w.setStyleSheet(style)
        lbl_hp = QLabel("HP"); lbl_lp = QLabel("LP")
        lbl_hz1 = QLabel("Hz"); lbl_hz2 = QLabel("Hz")
        for l in (lbl_hp, lbl_lp, lbl_hz1, lbl_hz2):
            l.setStyleSheet("color:#cccccc; font-size:12px;")

        for w in (self.chk_ir_dc, self.chk_ir_bp, self.chk_ir_fw):
            w.toggled.connect(self._on_ir_filter_changed)
        for w in (self.spin_ir_hp, self.spin_ir_lp):
            w.valueChanged.connect(self._on_ir_filter_changed)

        box.addWidget(self.chk_ir_dc)
        box.addSpacing(12)
        box.addWidget(self.chk_ir_bp)
        box.addWidget(lbl_hp); box.addWidget(self.spin_ir_hp); box.addWidget(lbl_hz1)
        box.addWidget(lbl_lp); box.addWidget(self.spin_ir_lp); box.addWidget(lbl_hz2)
        box.addSpacing(12)
        box.addWidget(self.chk_ir_fw)
        box.addSpacing(12)
        box.addWidget(self.lbl_ir_filt)
        box.addStretch()
        return box

    def _on_ir_filter_changed(self, *_):
        """Disable controls overridden by the firmware view; flag a rebuild."""
        fw = self.chk_ir_fw.isChecked()
        for w in (self.chk_ir_dc, self.chk_ir_bp, self.spin_ir_hp, self.spin_ir_lp):
            w.setEnabled(not fw)
        self._ir_dirty = True
        if self.is_paused:
            self._rebuild_ir_view()

    def _estimate_ir_fs(self):
        """Mean sampling rate (Hz) of the last <=100 real packets; 100 if too few."""
        if len(self.fft_ts) < 20:
            return 100.0
        n  = min(100, len(self.fft_ts))
        ts = np.fromiter((self.fft_ts[i] for i in range(-n, 0)), float, n)
        span = ts[-1] - ts[0]
        return 1000.0 * (n - 1) / span if span > 0 else 100.0

    def _fft_clear(self):
        """Drop the long PSD history (time gap, reconnect, Settings pause)."""
        self.fft_ts.clear(); self.fft_raw.clear()
        self.fft_ir.clear(); self.fft_pz.clear()
        self._reset_ir_state()

    def get_fft_buffers(self):
        """
        (ts_ms, ir, pz) for the PSD viewer, linearly interpolated onto a uniform
        grid at the mean packet period (Welch assumes uniform sampling). ir is the
        signal as displayed. Falls back to the short display buffers until enough
        packets exist.
        """
        n = len(self.fft_ts)
        if n < 50:
            return list(self.ts_data), list(self.ir_data), list(self.pz_data)
        ts = np.fromiter(self.fft_ts, float, n)
        ir = np.fromiter(self.fft_ir, float, n)
        pz = np.fromiter(self.fft_pz, float, n)
        span = ts[-1] - ts[0]
        if span <= 0:
            return list(self.ts_data), list(self.ir_data), list(self.pz_data)
        dt = span / (n - 1)
        tu = np.arange(ts[0], ts[-1] + 0.5 * dt, dt)
        return (tu.tolist(),
                np.interp(tu, ts, ir).tolist(),
                np.interp(tu, ts, pz).tolist())

    def _reset_ir_state(self):
        """Forget filter memory; states re-initialise on the next sample."""
        self._ir_init = False
        self._ir_zi   = None
        self._ir_dc   = None
        self._fw_avg  = 0
        self._fw_buf  = deque([0] * 23, maxlen=23)

    def _design_ir_filter(self):
        """Design the band-pass SOS from HP/LP at the measured fs (None = bypass)."""
        self._ir_sos = None
        self._ir_fs  = self._estimate_ir_fs()
        fs  = self._ir_fs
        msg = f"fs ≈ {fs:.0f} Hz"
        if fs < 80:
            msg += "  ⚠ low rate — timing unreliable"
        if self.chk_ir_bp.isChecked() and not self.chk_ir_fw.isChecked():
            hp  = self.spin_ir_hp.value()
            lp  = self.spin_ir_lp.value()
            lp  = min(lp, 0.45 * fs) if lp > 0 else 0.0
            nyq = fs / 2.0
            secs = []
            if hp > 0 and lp > 0 and hp >= lp:
                msg += "  ⚠ HP must be < LP"
            else:
                try:
                    if hp > 0:
                        secs.append(butter(_IR_FILT_ORDER, hp / nyq, 'highpass', output='sos'))
                    if lp > 0:
                        secs.append(butter(_IR_FILT_ORDER, lp / nyq, 'lowpass',  output='sos'))
                except ValueError:
                    secs = []
            if secs:
                self._ir_sos = np.vstack(secs)
        self.lbl_ir_filt.setText(msg)

    def _rebuild_ir_view(self):
        """Redesign the filter and re-filter the whole raw history."""
        self._ir_dirty = False
        self._design_ir_filter()
        self._reset_ir_state()
        out = [self._process_ir(x) for x in self.fft_raw]
        self.fft_ir = deque(out, maxlen=self._FFT_BUF_N)
        tail = out[-self.max_points:]
        self.ir_data = [0.0] * (self.max_points - len(tail)) + tail
        fw = self.chk_ir_fw.isChecked()
        ac = fw or self.chk_ir_dc.isChecked() or self._ir_sos is not None
        self.plot_ir.getAxis('left').setLabel(
            'counts (checkForBeat AC)' if fw else ('counts (AC)' if ac else 'counts'))
        self.plot_ir.enableAutoRange(axis='y')
        if self._buffer_fill >= self._PLOT_WARMUP:
            self.curve_ir.setData(self.ts_data, self.ir_data)

    def _process_ir(self, x):
        """
        Causal per-sample IR processing (display only).
        checkForBeat view: IIR DC (alpha 1/16) -> x - DC -> 23-tap FIR >> 15, the
        same integer arithmetic as the SparkFun library. Otherwise: optional slow
        DC removal, then optional band-pass. States initialise from the first
        non-zero sample.
        """
        if not self._ir_init:
            if x <= 0:
                return 0.0
            self._ir_init = True
            self._ir_dc   = float(x)
            self._fw_avg  = int(x) << 15

        if self.chk_ir_fw.isChecked():
            xi = int(x)
            self._fw_avg += ((xi << 15) - self._fw_avg) >> 4
            self._fw_buf.append(xi - (self._fw_avg >> 15))
            z = int(np.dot(_FW_FIR_TAPS, np.asarray(self._fw_buf, dtype=np.int64)))
            return float(z >> 15)

        v = float(x)
        if self.chk_ir_dc.isChecked():
            a = 1.0 - np.exp(-2.0 * np.pi * _IR_DC_FC_HZ / self._ir_fs)
            self._ir_dc += a * (v - self._ir_dc)
            v -= self._ir_dc
        if self._ir_sos is not None:
            if self._ir_zi is None:
                self._ir_zi = sosfilt_zi(self._ir_sos) * v
            out, self._ir_zi = sosfilt(self._ir_sos, [v], zi=self._ir_zi)
            v = float(out[0])
        return v

    def create_box(self, layout, title, unit, color):
        """Numeric readout card; returns the value QLabel."""
        frame = QFrame()
        frame.setStyleSheet(f"""
            QFrame {{
                background-color: #111;
                border: none;
                border-left: 6px solid {color};
                border-radius: 4px;
            }}
        """)
        vbox = QVBoxLayout(frame)

        lbl_title = QLabel(title.upper())
        lbl_title.setStyleSheet(
            "color: #666; font-size: 12px; font-weight: bold; "
            "letter-spacing: 1px; border: none;")

        lbl_val = QLabel("--")
        lbl_val.setStyleSheet(
            "color: #ccc; font-size: 52px; font-weight: bold; border: none;")
        lbl_val.setAlignment(Qt.AlignRight)

        lbl_unit = QLabel(unit)
        lbl_unit.setStyleSheet("color: #444; font-size: 16px; border: none;")
        lbl_unit.setAlignment(Qt.AlignRight)

        vbox.addWidget(lbl_title)
        vbox.addWidget(lbl_val)
        vbox.addWidget(lbl_unit)
        layout.addWidget(frame)
        return lbl_val

    # ── Connection ────────────────────────────────────────────────────────────

    def refresh_ports(self):
        self.combo_ports.clear()
        for port in serial.tools.list_ports.comports():
            self.combo_ports.addItem(port.device)

    def toggle_connection(self):
        if self.serial_thread is None or not self.serial_thread.running:
            port = self.combo_ports.currentText()
            if not port:
                return
            self.serial_thread = SerialThread(port)
            self.serial_thread.data_received.connect(self._on_data)
            self.serial_thread.connection_error.connect(self.on_connection_error)
            self.serial_thread.start()

            self.lbl_init_msg.setText("Initializing...")
            self.btn_connect.setText("DISCONNECT")
            # Read current firmware parameters once the boot messages have passed.
            QTimer.singleShot(1500, self._request_params)
            self.btn_connect.setStyleSheet(
                "background-color: #ff8c00; color: #fff; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            self.btn_settings.setEnabled(True)
            self.btn_settings.setStyleSheet(
                "background-color: #1a5276; color: #fff; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            self.btn_fft.setEnabled(True)
            self.btn_fft.setStyleSheet(
                "background-color: #1a3a1a; color: #6c6; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            self.btn_tare.setEnabled(True)
            self.btn_tare.setStyleSheet(
                "background-color: #2a2a1a; color: #cc4; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
        else:
            self.serial_thread.stop()
            self.serial_thread = None
            self.ts_origin_arduino = None
            self.ts_origin_wall    = None
            self.lbl_init_msg.setText("")
            self.btn_connect.setText("CONNECT")
            self.btn_connect.setStyleSheet(
                "background-color: #0055a4; color: #fff; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            self.btn_settings.setEnabled(False)
            self.btn_settings.setStyleSheet(
                "background-color: #333; color: #aaa; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            self.btn_fft.setEnabled(False)
            self.btn_fft.setStyleSheet(
                "background-color: #333; color: #aaa; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            self.btn_tare.setEnabled(False)
            self.btn_tare.setStyleSheet(
                "background-color: #333; color: #aaa; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            if self.fft_dlg and self.fft_dlg.isVisible():
                self.fft_dlg.close()

    def _request_params(self):
        """Request all firmware parameters; the replies fill param_cache."""
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.send({"cmd": "get"})

    def on_connection_error(self, err_msg):
        self.btn_connect.setText("ERROR — TRY AGAIN")
        self.btn_connect.setStyleSheet(
            "background-color: #cc0000; color: #fff; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.lbl_init_msg.setText("")
        print(f"Serial error: {err_msg}")

    def resp_tare(self):
        """
        Set the DC estimate to the mean of the last 50 raw samples instead of
        waiting ~200 s for the IIR. Session-only; resets on reconnect.
        """
        recent = [v for v in self.pz_raw_data[-50:] if v != 0]
        if len(recent) < 10:
            return
        self._pz_dc = sum(recent) / len(recent)

        self.btn_tare.setStyleSheet(
            "background-color: #006600; color: #fff; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        QTimer.singleShot(800, lambda: self.btn_tare.setStyleSheet(
            "background-color: #2a2a1a; color: #cc4; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;"))

    def open_fft(self):
        """Open the (non-modal, single-instance) PSD viewer."""
        if self.fft_dlg is None or not self.fft_dlg.isVisible():
            self.fft_dlg = FFTVitalSignsWindow(daq_window=self, parent=self)
            self.fft_dlg.setModal(False)
            self.fft_dlg.show()
        else:
            self.fft_dlg.raise_()
            self.fft_dlg.activateWindow()

    # ── Settings ──────────────────────────────────────────────────────────────

    def open_settings(self):
        """Open the modal Settings dialog (display pauses while it is open)."""
        if self.settings_dlg is not None and self.settings_dlg.isVisible():
            self.settings_dlg.raise_()
            self.settings_dlg.activateWindow()
            return

        thread = self.serial_thread if (self.serial_thread and
                                        self.serial_thread.running) else None
        self.settings_dlg = SettingsDialog(
            serial_thread=thread,
            param_cache=self.param_cache,
            daq_window=self,
            parent=self)
        self.settings_dlg.setModal(True)
        self.settings_dlg.exec_()
        # After the pause restart display and PSD buffers, so no zero-to-data step
        # or connecting line across the gap is drawn.
        self.ir_data = [0] * self.max_points
        self.pz_data = [0] * self.max_points
        self._fft_clear()
        last_ts = self.ts_data[-1]
        self.ts_data = list(range(last_ts, last_ts + self.max_points * 10, 10))
        self.curve_ir.setData([], [])
        self.curve_pz.setData([], [])
        self._buffer_fill = 0
        self.is_paused = False

    # ── Playback ──────────────────────────────────────────────────────────────

    def toggle_pause(self):
        """Pause / resume the display and CSV logging."""
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.btn_play_pause.setText("▶ PLAY")
            self.btn_play_pause.setStyleSheet(
                "background-color: #28a745; color: #fff; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
        else:
            self.btn_play_pause.setText("⏸ PAUSE")
            self.btn_play_pause.setStyleSheet(
                "background-color: #333; color: #fff; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")

    # ── Recording ─────────────────────────────────────────────────────────────

    def toggle_record(self):
        if not self.is_recording:
            base, _ = QFileDialog.getSaveFileName(
                self, "Save Surgical Record", "", "CSV Files (*.csv)",
                options=QFileDialog.Options())
            if not base:
                return
            if base.endswith('.csv'):
                base = base[:-4]
            try:
                self.file_vitals = open(f"{base}_vitals.csv", 'w', newline='')
                self.file_raw    = open(f"{base}_raw.csv",    'w', newline='')
                self.csv_vitals  = csv.writer(self.file_vitals)
                self.csv_raw     = csv.writer(self.file_raw)

                self.csv_vitals.writerow([
                    'Time_Seconds', 'HeartRate_BPM', 'SpO2_%',
                    'RespRate_RPM', 'Temperature_C', 'Event_Marker'])
                self.csv_raw.writerow([
                    'Time_Seconds', 'TS_Arduino_ms',
                    'IR_Raw', 'Resp_mV', 'Event_Marker'])

                self.start_time        = time.time()
                self.ts_origin_arduino = None
                self.ts_origin_wall    = None
                self.is_recording      = True

                self.btn_record.setText("⏹ STOP REC")
                self.btn_record.setStyleSheet(
                    "background-color: #ff0000; color: #fff; padding: 8px; "
                    "font-weight: bold; border: none; border-radius: 4px;")
                self.lbl_rec_status.setText("RECORDING...")
                self.btn_event.setEnabled(True)
                self.btn_event.setStyleSheet(
                    "background-color: #e6b800; color: #000; padding: 8px; "
                    "font-weight: bold; border-radius: 4px;")
                self.clear_event_lines()
            except Exception as e:
                print(f"Error creating CSV files: {e}")
        else:
            self.is_recording = False
            self.btn_record.setText("⏺ REC")
            self.btn_record.setStyleSheet(
                "background-color: #500; color: #fff; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            self.lbl_rec_status.setText("")
            self.btn_event.setEnabled(False)
            self.btn_event.setStyleSheet(
                "background-color: #555; color: #aaa; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            if self.file_vitals: self.file_vitals.close()
            if self.file_raw:    self.file_raw.close()
            self.csv_vitals = None
            self.csv_raw    = None

    # ── Event markers ─────────────────────────────────────────────────────────

    def mark_event(self):
        if self.is_recording:
            self.event_flag = True
            self.draw_event_line()

    def draw_event_line(self):
        ts_now   = self.ts_data[-1] if self.ts_data else 0
        line_pen = pg.mkPen(color=(255, 255, 255, 100), width=2)
        vLineIR  = pg.InfiniteLine(pos=ts_now, angle=90, movable=False, pen=line_pen)
        vLinePZ  = pg.InfiniteLine(pos=ts_now, angle=90, movable=False, pen=line_pen)
        self.plot_ir.addItem(vLineIR)
        self.plot_pz.addItem(vLinePZ)
        self.event_lines_ir.append((vLineIR, ts_now))
        self.event_lines_pz.append((vLinePZ, ts_now))

    def clear_event_lines(self):
        for line, _ in self.event_lines_ir:
            self.plot_ir.removeItem(line)
        for line, _ in self.event_lines_pz:
            self.plot_pz.removeItem(line)
        self.event_lines_ir.clear()
        self.event_lines_pz.clear()

    # ── Data path (100 Hz) ────────────────────────────────────────────────────

    def _on_data(self, data):
        """Route ACK/NAK to param_cache and the Settings dialog; packets to update_gui()."""
        if 'ack' in data or 'nak' in data:
            key = data.get('ack')
            if key and 'val' in data:
                val_cached = data['val']
                # CALIB_SWING: counts on the firmware, mV in the panel
                if key == 'CALIB_SWING':
                    val_cached = round(val_cached * _ADC_MV, 1)
                self.param_cache[key] = val_cached
            if self.settings_dlg is not None:
                self.settings_dlg.on_ack(data)
            return
        self.update_gui(data)

    def update_gui(self, data):
        """Process one data packet: readouts, buffers, plots, event lines, CSV."""
        if self.lbl_init_msg.text():
            self.lbl_init_msg.setText("")

        if self.is_paused:
            return

        hr = data.get('hr', 0)
        sp = data.get('sp', 0)
        rr = data.get('rr', 0)
        t  = data.get('t',  0)
        ir = data.get('ir', 0)
        pz_raw = data.get('pz', 0) * _ADC_MV

        self.pz_raw_data.append(pz_raw)
        self.pz_raw_data.pop(0)

        # Respiratory conditioning: slow IIR DC estimate (alpha 0.005, ~0.08 Hz);
        # AC about DC with polarity per _INVERT_RESP; DC added back so the trace
        # stays positive at rest.
        if self._pz_dc is None:
            self._pz_dc = pz_raw
        self._pz_dc += 0.005 * (pz_raw - self._pz_dc)
        pz_ac = (self._pz_dc - pz_raw) if _INVERT_RESP else (pz_raw - self._pz_dc)
        pz    = pz_ac + self._pz_dc

        # Arduino timestamp; +10 ms fallback for firmware without "ts".
        ts_raw = data.get('ts', None)
        ts_ms  = int(ts_raw) if ts_raw is not None else (self.ts_data[-1] + 10)

        self.val_hr.setText(str(hr)      if hr > 0 else "--")
        self.val_spo2.setText(str(sp)    if sp > 0 else "--")
        self.val_rr.setText(str(rr)      if rr > 0 else "--")
        self.val_temp.setText(f"{t:.1f}" if t  > 0 else "--.-")

        # A time gap (pause, USB stall) or time going backwards (Arduino reset on
        # reconnect) breaks uniform sampling: restart the PSD history, and the
        # display buffers on a reset.
        if self.fft_ts:
            gap = ts_ms - self.fft_ts[-1]
            if gap > 100 or gap < 0:
                self._fft_clear()
        if ts_ms < self.ts_data[-1]:
            self.ts_data = [ts_ms - 10 * (self.max_points - 1 - i)
                            for i in range(self.max_points)]
            self.ir_data = [0] * self.max_points
            self.pz_data = [0] * self.max_points
            self._buffer_fill = 0
            self.clear_event_lines()

        self.ts_data.append(ts_ms); self.ts_data.pop(0)
        self.pz_data.append(pz);    self.pz_data.pop(0)

        # The CSV always receives the raw ir; fft_ir holds the displayed value.
        self.fft_ts.append(ts_ms)
        self.fft_pz.append(pz)
        self.fft_raw.append(ir)
        if self._ir_dirty:
            self._rebuild_ir_view()
        else:
            v_ir = self._process_ir(ir)
            self.ir_data.append(v_ir); self.ir_data.pop(0)
            self.fft_ir.append(v_ir)
            # Every ~1 s: redesign if the measured fs drifted > 5 % from the design.
            self._ir_fs_cnt += 1
            if self._ir_fs_cnt >= 100:
                self._ir_fs_cnt = 0
                fs_now = self._estimate_ir_fs()
                if abs(fs_now - self._ir_fs) / self._ir_fs > 0.05:
                    self._ir_dirty = True

        self._buffer_fill += 1
        if self._buffer_fill >= self._PLOT_WARMUP:
            self.curve_ir.setData(self.ts_data, self.ir_data)
            self.curve_pz.setData(self.ts_data, self.pz_data)

        self.plot_ir.setXRange(self.ts_data[0], self.ts_data[-1], padding=0)

        visible_start = self.ts_data[0]
        for collection, plot in [(self.event_lines_ir, self.plot_ir),
                                  (self.event_lines_pz, self.plot_pz)]:
            for line, ts in list(collection):
                if ts < visible_start:
                    plot.removeItem(line)
            collection[:] = [(l, ts) for l, ts in collection if ts >= visible_start]

        if self.is_recording:
            if self.ts_origin_arduino is None:
                self.ts_origin_arduino = ts_ms
                self.ts_origin_wall    = time.time()

            ts_arduino_ms = ts_ms - self.ts_origin_arduino
            current_time  = round(time.time() - self.start_time, 3)

            if self.event_flag:
                event_vital_str = "Event Start"
                event_raw_val   = 1
                self.event_flag = False
            else:
                event_vital_str = ""
                event_raw_val   = 0

            self.csv_vitals.writerow(
                [current_time, hr, sp, rr, t, event_vital_str])
            self.csv_raw.writerow(
                [current_time, ts_arduino_ms, ir, pz, event_raw_val])

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        if self.is_recording:
            self.toggle_record()
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()
        event.accept()


# ════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    # Reuse an existing QApplication (Spyder / IPython).
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    window = PyVitalSignsDAQ()
    window.show()
    app.exec_()