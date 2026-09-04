"""
Py_VitalSigns_DAQ.py
====================
PROJECT : Vital-signs monitor
VERSION : 3.0
AUTHOR  : Flávio Mourão — Mar, 2026
Last update - Set 4, 2026

════════════════════════════════════════════════════════════════════════════════
MODULE RESPONSIBILITY
════════════════════════════════════════════════════════════════════════════════
Standalone USB data-acquisition and real-time display application for the
rodent vital-signs monitor. Receives a 100 Hz JSON stream from the ATmega2560
over USB-Serial and provides:

  - Real-time scrolling waveform display (IR-PPG and piezo respiratory belt)
  - Numeric readouts: HR, SpO2, respiratory rate, core temperature
  - Intra-session event marking with visual timeline markers on both waveforms
  - CSV recording split into two files:
      *_vitals.csv — low-rate physiological values, wall-clock time axis
      *_raw.csv    — high-rate raw waveform samples, Arduino timestamp axis
  - Settings panel for runtime parameter adjustment via bidirectional Serial
    protocol — no recompilation required

════════════════════════════════════════════════════════════════════════════════
TEMPORAL ALIGNMENT ARCHITECTURE
════════════════════════════════════════════════════════════════════════════════
The firmware produces two signals with fundamentally different time domains:

  Piezo (pz): captured by TIMER1_COMPA_vect — hardware ISR at exactly 100 Hz,
              fully deterministic.

  IR (ir):    read from the MAX30102 FIFO inside loop() — cooperative
              scheduling, subject to ±1–5 ms jitter.

Both signals are packed into the same JSON packet. The firmware includes a
"ts" field (ATmega2560 millis()) used as the common X-axis for both signals,
eliminating USB transport jitter from the recorded waveforms.

════════════════════════════════════════════════════════════════════════════════
SETTINGS WORKFLOW
════════════════════════════════════════════════════════════════════════════════
Parameter changes are applied in real time via Serial:
  1. Click Settings → adjust values → click Send to Arduino.
  2. Arduino receives each command, applies immediately, and confirms with ACK.
  3. Parameters are saved to EEPROM on the Arduino — survive power cycles.
  4. On connect, Python sends {"cmd":"get"} to read current Arduino values.

════════════════════════════════════════════════════════════════════════════════
JSON PACKET FORMAT  (defined in Py_Vital-Signs.ino)
════════════════════════════════════════════════════════════════════════════════
  {"hr":NNN,"sp":NNN,"rr":NNN,"pz":NNN,"t":NN.N,"ir":NNNNNN,"ts":NNNNNN}

  hr  — heart rate (BPM, int). 0 = no valid signal.
  sp  — SpO2 (%, int). 0 = no valid signal.
  rr  — respiratory rate (rpm, int). 0 = apnoea or no signal.
  pz  — piezo ADC filtered value (10-bit counts, 0-1023).
  t   — core temperature (deg C, one decimal). 0 = probe disconnected.
  ir  — raw IR photodetector count from MAX30102 (uint32, 0-262143).
  ts  — ATmega2560 millis() at packet serialisation time (ms, uint32).

BIDIRECTIONAL PROTOCOL  (Python → Arduino, on demand)
  Command: SOH(0x01) + {"cmd":"set","key":"KEY","val":VALUE} + newline
  Get all: SOH(0x01) + {"cmd":"get"} + newline
  ACK:     {"ack":"KEY","val":APPLIED_VALUE}
  NAK:     {"nak":"reason"}

════════════════════════════════════════════════════════════════════════════════
CSV OUTPUT FORMAT
════════════════════════════════════════════════════════════════════════════════
  *_vitals.csv
    Time_Seconds, HeartRate_BPM, SpO2_%, RespRate_RPM, Temperature_C,
    Event_Marker

  *_raw.csv
    Time_Seconds, TS_Arduino_ms, IR_Raw, Piezo_Raw, Event_Marker
    USE TS_Arduino_ms (not Time_Seconds) as the time axis for signal analysis.

════════════════════════════════════════════════════════════════════════════════
DEPENDENCIES
════════════════════════════════════════════════════════════════════════════════
  pyserial   >= 3.5   — pip install pyserial
  PyQt5      >= 5.15  — pip install PyQt5
  pyqtgraph  >= 0.13  — pip install pyqtgraph

  Compatible with Spyder IDE: uses QApplication.instance() to avoid creating
  a second QApplication when IPython already manages one.

════════════════════════════════════════════════════════════════════════════════
HARDWARE CONTEXT
════════════════════════════════════════════════════════════════════════════════
  Board  : RobotDyn Mega+WiFi (ATmega2560 + ESP8266 on-board co-processor)
  USB    : ATmega2560 USB-Serial bridge
  Baud   : 115200
  Stream : JSON lines at 100 Hz, terminated by newline
"""

import sys
import json
import time
import csv
import queue
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFileDialog, QFrame, QComboBox,
    QDialog, QDoubleSpinBox, QSpinBox, QGroupBox, QGridLayout,
    QLineEdit
)
from PyQt5.QtCore import QThread, QTimer, pyqtSignal, Qt
import pyqtgraph as pg


# ════════════════════════════════════════════════════════════════════════════
# SERIAL READING THREAD
# ════════════════════════════════════════════════════════════════════════════

class SerialThread(QThread):
    """
    Background QThread that owns the serial port exclusively.
    Follows the same architecture as conditioning_setup_dark.py:
      - All port access (read AND write) happens inside run(), in one thread.
      - send(cmd_dict): called from any thread; places command in a thread-safe
        queue. _drain_tx() writes it to the port inside run().
      - data_received: emitted for every valid JSON line (data packets AND ACKs).
        The Settings dialog connects to this signal to receive ACK responses.
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
        """
        Queue a command for transmission. Safe to call from any thread.
        Prefixes each command with SOH (0x01) to signal the Arduino to halt
        its outbound 100 Hz stream before receiving the command bytes.
        Protocol: SOH + JSON + newline
        """
        payload = '\x01' + json.dumps(cmd_dict, separators=(',', ':')) + '\n'
        self._tx_queue.put(payload)

    def _drain_tx(self):
        """
        Write queued commands to the port. Called only from run().

        Since _send_all() now sends one command at a time (waiting for ACK
        before sending the next via on_ack → _send_next), the queue normally
        holds at most one command at a time.

        SOH protocol per command:
          1. Send SOH byte (0x01) — Arduino halts outbound stream immediately.
          2. Wait 60 ms — Arduino finishes any in-progress Serial.print() and
             its UART TX buffer drains. Reset OS RX buffer to discard data
             packets that arrived during the wait.
          3. Send the JSON command bytes. Arduino processes and sends ACK.
          4. ACK arrives via _read_lines() → data_received → _on_data → on_ack.
          5. on_ack() calls _send_next() to enqueue the next command.
          6. Arduino resumes 100 Hz stream after cmdInProgress clears.
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
        """Read all available lines and emit data_received for valid JSON."""
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
    """
    Reference panel describing each configurable parameter.
    Opened from the Settings dialog via the Help button.
    """

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

<h2>Piezo / Respiratory Rate</h2>
<ul>
  <li><span class="key">Calib Min Swing</span> — Minimum ADC envelope swing (counts) required
      for the calibration to be considered valid. Increase in noisy environments;
      decrease for animals with shallow breathing.
      <span class="unit">Range: 20–400 counts</span></li>
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

        from PyQt5.QtGui import QTextOption
        from PyQt5.QtWidgets import QTextBrowser
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
    Runtime parameter configuration panel — sends parameters via Serial,
    exactly like conditioning_setup_dark.py sends trial data to the DUE.

    Workflow:
      1. User adjusts values and clicks Send.
      2. Dialog calls serial_thread.send({"cmd":"set","key":K,"val":V})
         for each parameter — thread-safe via queue.Queue.
      3. Arduino receives, applies, and responds with {"ack":K,"val":V}.
      4. data_received signal routes the ACK back to on_ack(), which
         updates the status label and the param_cache.

    The DAQ stream (100 Hz data packets) continues during Send — the
    serial thread reads and emits them normally. update_gui() in the main
    window is paused (is_paused=True) so the display freezes, but no
    data is lost and the port is never shared between threads.
    """
    
    # (label, key, type, min, max, step, decimals, default, unit)

    PARAMS = {
        "Heart Rate": [
            ("HR Min",          "HR_MIN",    "float", 20,   780,  10,   0, 40,    "BPM"),
            ("HR Max",          "HR_MAX",    "float", 50,   800,  10,   0, 200,   "BPM"),
            ("Outlier Frac",    "HR_OUTLIER","float", 0.10, 0.70, 0.05, 2, 0.35,  ""),
        ],
        "SpO2": [
            ("Min Valid SpO2",  "SPO2_MIN",  "float", 50,   95,   5,    0, 80,    "%"),
            ("Decim Ratio",     "DECIM_RATIO","int",  1,    4,    1,    0, 1,     ""),
        ],
        "Sensor Setup": [
            ("LED Brightness",  "LED",       "int",   10,   255,  10,   0, 80,    ""),
            ("ADC Range",       "ADC_RANGE", "int",   2048, 16384,2048, 0, 16384, ""),
            ("Sample Average",  "SAMPLE_AVG","int",   1,    32,   1,    0, 4,     ""),
        ],
        "Piezo / Resp Rate": [
            ("Calib Min Swing", "CALIB_SWING","int",  20,   400,  10,   0, 120,   "counts"),
            ("Insp Threshold",  "THRESH_INSP","float",0.40, 0.85, 0.05, 2, 0.65,  ""),
            ("Exp Threshold",   "THRESH_EXP", "float",0.20, 0.70, 0.05, 2, 0.45,  ""),
        ],
        "Temperature": [
            ("Temp Min",        "TEMP_MIN",  "float", 20,   35,   1,    1, 30,    "deg C"),
            ("Temp Max",        "TEMP_MAX",  "float", 35,   50,   1,    1, 45,    "deg C"),
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
            for row, (label, key, wtype, mn, mx, step, dec, default, unit) in enumerate(params):
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
                if unit:
                    spin.setSuffix(f"  {unit}")
                self.widgets[key] = spin
                grid.addWidget(QLabel(label), row, 0)
                grid.addWidget(spin,          row, 1)
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
        """
        Send parameters to the Arduino one at a time, waiting for each ACK
        before sending the next command. This matches the pattern used in
        conditioning_setup_dark.py and avoids UART buffer overflow.

        Flow:
          1. Build the ordered list of (key, val) pairs.
          2. Send the first command via _send_next().
          3. on_ack() calls _send_next() again after each confirmed ACK.
          4. When the list is empty, re-enable the Send button.
        """
        if self.thread is None or not self.thread.running:
            self.lbl_status.setText("Not connected.")
            return

        # Build ordered queue of parameters to send
        self._send_queue = []
        for params in self.PARAMS.values():
            for row in params:
                key = row[1]
                val = self.widgets[key].value()
                self._send_queue.append((key, val))

        self._pending = [key for key, _ in self._send_queue]
        self.btn_send.setEnabled(False)
        self.lbl_status.setText("Sending...")

        # Send the first command — subsequent ones triggered by on_ack()
        self._send_next()

    def _send_next(self):
        """Send the next command from _send_queue, if any remain."""
        if not hasattr(self, '_send_queue') or not self._send_queue:
            return
        key, val = self._send_queue.pop(0)
        self.thread.send({"cmd": "set", "key": key, "val": val})

    def on_ack(self, resp):
        """
        Called by the main window when an ACK/NAK arrives from the Arduino.
        Updates the status label and param_cache for confirmed parameters.

        resp contains either {"ack": key, "val": applied_value}
                          or {"nak": reason}
        """
        ok  = 'ack' in resp
        key = resp.get('ack') if ok else resp.get('nak', '?')

        if ok and self.daq and key in self.widgets:
            # Cache the widget value so reopening the dialog shows current state.
            self.daq.param_cache[key] = self.widgets[key].value()

        if key in self._pending:
            self._pending.remove(key)

        symbol = "✓" if ok else "✗"
        current = self.lbl_status.text().replace("Sending...", "").strip()
        self.lbl_status.setText(f"{current}  {symbol}{key}".strip())

        if self._pending:
            # More commands waiting — send the next one now
            self._send_next()
        else:
            self.btn_send.setEnabled(True)

    def _open_help(self):
        """Open the parameter reference dialog."""
        dlg = HelpDialog(parent=self)
        dlg.exec_()

    def _resume_daq(self):
        if self.daq and not self._was_paused:
            self.daq.is_paused = False

    def _close(self):
        # Always re-enable send button and clear pending state before closing,
        # so the dialog can be closed even if ACKs never arrived.
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
    """
    Main application window. Owns the UI layout, the serial thread lifecycle,
    waveform buffers, CSV writers, and the event marker system.

    Layout:
        Top bar      — port selector, connect, settings, event marker, pause, REC
        Left sidebar — four numeric readout cards (HR, SpO2, RR, Temp)
        Right area   — two stacked pyqtgraph PlotWidgets with linked X axes
    """

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

        # ── Timestamp alignment state ─────────────────────────────────────────
        # ts_origin_arduino: "ts" value of the first packet after REC start.
        # Used to normalise TS_Arduino_ms to 0 at the start of each session.
        self.ts_origin_arduino = None
        self.ts_origin_wall    = None

        # ── Waveform display buffers ──────────────────────────────────────────
        # Three parallel lists of length max_points (oldest discarded on append).
        self.max_points = 300           # ~3 s of history at 100 Hz
        self.ir_data    = [0] * self.max_points
        self.pz_data    = [0] * self.max_points
        # ts_data: Arduino timestamps (ms) for the X axis.
        # Initialised to a 10 ms ramp so pyqtgraph has valid X before first packet.
        self.ts_data    = list(range(0, self.max_points * 10, 10))

        # ── Event marker state ────────────────────────────────────────────────
        # Each entry is (InfiniteLine, ts_ms). Lines are pruned when their
        # timestamp scrolls out of the visible ts_data window.
        self.event_lines_ir = []
        self.event_lines_pz = []

        # param_cache: {key: current firmware value}.
        # Populated from Arduino via {"cmd":"get"} on connect.
        self.param_cache = {}

        self.serial_thread = None
        self.settings_dlg  = None

        self.init_ui()

    # ─────────────────────────────────────────────────────────────────────────
    # UI CONSTRUCTION
    # ─────────────────────────────────────────────────────────────────────────

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

        self.lbl_init_msg = QLabel("")
        self.lbl_init_msg.setStyleSheet(
            "color: #cccccc; font-size: 16px; font-weight: bold; margin-left: 30px;")

        self.btn_event = QPushButton("⚡ MARK EVENT")
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

        # Left sidebar — numeric readout cards
        sidebar = QVBoxLayout()
        sidebar.setContentsMargins(0, 0, 10, 0)
        self.val_hr   = self.create_box(sidebar, "Heart Rate",  "bpm", "#b30000")
        self.val_spo2 = self.create_box(sidebar, "SpO2",        "%",   "#cc6600")
        self.val_rr   = self.create_box(sidebar, "Resp. Rate",  "rpm", "#cca300")
        self.val_temp = self.create_box(sidebar, "Temperature", "deg C", "#800080")
        body_layout.addLayout(sidebar, stretch=1)

        # Right area — waveform charts
        charts_layout = QVBoxLayout()

        # IR-PPG chart: X axis is Arduino timestamp for true temporal alignment.
        self.plot_ir = pg.PlotWidget(
            title='<span style="color:#b30000;font-size:11pt;font-weight:bold;">'
                  'PPG — IR Optical (MAX30102)</span>')
        self.plot_ir.setBackground('#0d0d0d')
        self.plot_ir.showGrid(x=True, y=True, alpha=0.2)
        self.plot_ir.getAxis('bottom').setLabel('Time (ms, Arduino clock)')
        self.curve_ir = self.plot_ir.plot(pen=pg.mkPen('#b30000', width=2))
        charts_layout.addWidget(self.plot_ir)

        # Piezo chart: linked X axis — panning/zooming one moves both.
        self.plot_pz = pg.PlotWidget(
            title='<span style="color:#cca300;font-size:11pt;font-weight:bold;">'
                  'RESP — Piezo Belt</span>')
        self.plot_pz.setBackground('#0d0d0d')
        self.plot_pz.showGrid(x=True, y=True, alpha=0.2)
        self.plot_pz.getAxis('bottom').setLabel('Time (ms, Arduino clock)')
        self.curve_pz = self.plot_pz.plot(pen=pg.mkPen('#cca300', width=2))
        self.plot_pz.setXLink(self.plot_ir)
        charts_layout.addWidget(self.plot_pz)

        body_layout.addLayout(charts_layout, stretch=4)

    def create_box(self, layout, title, unit, color):
        """Build a numeric readout card and return the value QLabel."""
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

    # ─────────────────────────────────────────────────────────────────────────
    # CONNECTION MANAGEMENT
    # ─────────────────────────────────────────────────────────────────────────

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
            # Request current parameter values from Arduino after a short
            # delay to allow the firmware init message to clear the UART.
            QTimer.singleShot(1500, self._request_params)
            self.btn_connect.setStyleSheet(
                "background-color: #ff8c00; color: #fff; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")
            self.btn_settings.setEnabled(True)
            self.btn_settings.setStyleSheet(
                "background-color: #1a5276; color: #fff; padding: 8px; "
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

    def _request_params(self):
        """
        Send {"cmd":"get"} to the Arduino to retrieve all current g_* values.
        Called automatically 1.5 s after connecting, giving the firmware time
        to finish its init message and start the 100 Hz stream.
        The Arduino responds with one {"ack":"KEY","val":V} per parameter.
        _on_data() routes each ACK to the param_cache via on_ack() on any
        open SettingsDialog, or directly to param_cache here.
        """
        if self.serial_thread and self.serial_thread.running:
            self.serial_thread.send({"cmd": "get"})

    def on_connection_error(self, err_msg):
        self.btn_connect.setText("ERROR — TRY AGAIN")
        self.btn_connect.setStyleSheet(
            "background-color: #cc0000; color: #fff; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.lbl_init_msg.setText("")
        print(f"Serial error: {err_msg}")

    # ─────────────────────────────────────────────────────────────────────────
    # SETTINGS
    # ─────────────────────────────────────────────────────────────────────────

    def open_settings(self):
        """
        Open the Settings dialog modally. The DAQ stream pauses automatically
        inside SettingsDialog.__init__ and resumes on close.
        Only one instance at a time.
        """
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
        # exec_() blocks until dialog closes.
        # Clear buffers and disconnect curves before resuming so the display
        # starts clean without the sigmoidal IR artefact (zeros → live data)
        # or the double-line artefact (pyqtgraph connects last old point to
        # first new point across the pause gap).
        self.ir_data = [0] * self.max_points
        self.pz_data = [0] * self.max_points
        # Keep ts_data continuous from where it left off
        last_ts = self.ts_data[-1]
        self.ts_data = list(range(last_ts, last_ts + self.max_points * 10, 10))
        # Disconnect curves (empty data) so pyqtgraph draws no connecting line
        self.curve_ir.setData([], [])
        self.curve_pz.setData([], [])
        self.is_paused = False

    # ─────────────────────────────────────────────────────────────────────────
    # PLAYBACK CONTROL
    # ─────────────────────────────────────────────────────────────────────────

    def toggle_pause(self):
        """Pause or resume waveform display and CSV logging."""
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

    # ─────────────────────────────────────────────────────────────────────────
    # RECORDING MANAGEMENT
    # ─────────────────────────────────────────────────────────────────────────

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
                    'IR_Raw', 'Piezo_Raw', 'Event_Marker'])

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

    # ─────────────────────────────────────────────────────────────────────────
    # EVENT MARKER
    # ─────────────────────────────────────────────────────────────────────────

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

    # ─────────────────────────────────────────────────────────────────────────
    # MAIN UPDATE — called at 100 Hz by SerialThread signal
    # ─────────────────────────────────────────────────────────────────────────

    def _on_data(self, data):
        """
        Slot connected to SerialThread.data_received.
        Routes ACK/NAK responses to the Settings dialog;
        forwards data packets to update_gui().
        """
        if 'ack' in data or 'nak' in data:
            # Update param_cache with confirmed firmware value.
            # Covers both {"cmd":"set"} ACKs and {"cmd":"get"} responses.
            key = data.get('ack')
            if key and 'val' in data:
                self.param_cache[key] = data['val']
            # Forward to open Settings dialog for status label update.
            if self.settings_dlg is not None:
                self.settings_dlg.on_ack(data)
            return
        self.update_gui(data)

    def update_gui(self, data):
        """
        Process one incoming JSON data packet.

        Steps per packet:
          1. Clear "Initializing..." label on first arrival.
          2. Extract fields; apply defaults for missing keys.
          3. Extract and validate the "ts" timestamp.
          4. Update numeric readout labels.
          5. Append to waveform buffers; discard oldest sample.
          6. Redraw both waveform curves with real-time X axis.
          7. Update visible X range (plot_pz follows via XLink).
          8. Prune event lines that have scrolled out of view.
          9. If recording: write one row to each CSV file.
        """
        # Step 1
        if self.lbl_init_msg.text():
            self.lbl_init_msg.setText("")

        if self.is_paused:
            return

        # Step 2
        hr = data.get('hr', 0)
        sp = data.get('sp', 0)
        rr = data.get('rr', 0)
        t  = data.get('t',  0)
        ir = data.get('ir', 0)
        pz = data.get('pz', 0)

        # Step 3 — "ts" is Arduino capture time; fall back to +10 ms increment
        # for firmware without the ts field (maintains display without alignment).
        ts_raw = data.get('ts', None)
        ts_ms  = int(ts_raw) if ts_raw is not None else (self.ts_data[-1] + 10)

        # Step 4
        self.val_hr.setText(str(hr)      if hr > 0 else "--")
        self.val_spo2.setText(str(sp)    if sp > 0 else "--")
        self.val_rr.setText(str(rr)      if rr > 0 else "--")
        self.val_temp.setText(f"{t:.1f}" if t  > 0 else "--.-")

        # Step 5
        self.ir_data.append(ir);    self.ir_data.pop(0)
        self.pz_data.append(pz);    self.pz_data.pop(0)
        self.ts_data.append(ts_ms); self.ts_data.pop(0)

        # Step 6
        self.curve_ir.setData(self.ts_data, self.ir_data)
        self.curve_pz.setData(self.ts_data, self.pz_data)

        # Step 7
        self.plot_ir.setXRange(self.ts_data[0], self.ts_data[-1], padding=0)

        # Step 8
        visible_start = self.ts_data[0]
        for collection, plot in [(self.event_lines_ir, self.plot_ir),
                                  (self.event_lines_pz, self.plot_pz)]:
            for line, ts in list(collection):
                if ts < visible_start:
                    plot.removeItem(line)
            collection[:] = [(l, ts) for l, ts in collection if ts >= visible_start]

        # Step 9
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

    # ─────────────────────────────────────────────────────────────────────────
    # APPLICATION SHUTDOWN
    # ─────────────────────────────────────────────────────────────────────────

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
    # Spyder / IPython: reuse existing QApplication to avoid RuntimeError.
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    window = PyVitalSignsDAQ()
    window.show()
    app.exec_()
