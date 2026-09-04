"""
Py_VitalSigns_DAQ.py
====================
PROJECT : Vital-signs monitor — small rodents (rat / mouse)
VERSION : 1.0
AUTHOR  : Flávio Mourão — Mar, 2026

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

════════════════════════════════════════════════════════════════════════════════
TEMPORAL ALIGNMENT ARCHITECTURE
════════════════════════════════════════════════════════════════════════════════
The firmware produces two signals with fundamentally different time domains:

  Piezo (pz): captured by TIMER1_COMPA_vect — hardware ISR at exactly 100 Hz,
              fully deterministic. Each sample occurs at a precise instant
              relative to the ATmega2560 clock.

  IR (ir):    read from the MAX30102 FIFO inside loop() — cooperative
              scheduling, variable execution time. Nominally 100 Hz but
              subject to ±1–5 ms jitter from display (~8 ms) and NTC tasks.

Both signals are packed into the same JSON packet, so within a given packet
they were captured in the same loop() iteration. The firmware includes a "ts"
field containing millis() at packet serialisation time. This timestamp is the
common temporal reference for ir and pz within that packet.

On the Python side, "ts" is used as the X-axis for both waveform plots and as
the primary time column (TS_Arduino_ms) in the raw CSV. This eliminates USB
transport jitter — which can reach 1–15 ms per packet due to OS scheduling and
pyserial buffering — from the recorded waveforms.

time.time() is retained for the vitals CSV, where wall-clock time is more
meaningful for session management (drug timing, procedure duration, etc.).

Firmware compatibility: packets without "ts" (older firmware) fall back to a
synthesised 10 ms increment per packet. The display remains functional but
without temporal alignment correction.

════════════════════════════════════════════════════════════════════════════════
JSON PACKET FORMAT  (defined in surgery_monitor_vitalsigns.ino)
════════════════════════════════════════════════════════════════════════════════
  {"hr":NNN,"sp":NNN,"rr":NNN,"pz":NNN,"t":NN.N,"ir":NNNNNN,"ts":NNNNNN}

  hr  — heart rate (BPM, int). 0 = no valid signal.
  sp  — SpO2 (%, int). 0 = no valid signal.
  rr  — respiratory rate (rpm, int). 0 = apnoea or no signal.
  pz  — piezo ADC filtered value (10-bit counts, 0–1023).
  t   — core temperature (°C, one decimal). 0 = probe disconnected.
  ir  — raw IR photodetector count from MAX30102 (uint32, 0–262143).
  ts  — ATmega2560 millis() at packet serialisation time (ms, uint32).
        Wraps at 2^32 ms (~49.7 days). Not normalised in the firmware.

════════════════════════════════════════════════════════════════════════════════
CSV OUTPUT FORMAT
════════════════════════════════════════════════════════════════════════════════
  *_vitals.csv  (one row per packet, 100 Hz)
    Time_Seconds   — wall-clock elapsed time since REC start (s, float, 3 dp)
    HeartRate_BPM  — HR from MAX30102 ring buffer (BPM, int). 0 = no signal.
    SpO2_%         — SpO2 from Maxim algorithm (%, int). 0 = no signal.
    RespRate_RPM   — respiratory rate from piezo detector (rpm, int). 0 = apnoea.
    Temperature_C  — NTC rectal temperature (°C, float). 0 = probe disconnected.
    Event_Marker   — "Event Start" on event rows, empty string otherwise.

  *_raw.csv  (one row per packet, 100 Hz)
    Time_Seconds   — wall-clock elapsed time (s, float) — for cross-referencing
                     with the vitals file. Do NOT use for waveform alignment.
    TS_Arduino_ms  — Arduino millis() normalised to 0 at REC start (ms, int).
                     USE THIS COLUMN as the time axis for signal processing.
                     Reflects capture time, not USB arrival time.
    IR_Raw         — raw IR count from MAX30102 FIFO (counts, uint32).
    Piezo_Raw      — filtered piezo ADC value (counts, 0–1023).
    Event_Marker   — 1 on event rows, 0 otherwise.

════════════════════════════════════════════════════════════════════════════════
POST-PROCESSING EXAMPLE (NumPy / pandas)
════════════════════════════════════════════════════════════════════════════════
  import pandas as pd
  import matplotlib.pyplot as plt

  df = pd.read_csv('session_raw.csv')
  t  = df['TS_Arduino_ms'] / 1000.0   # ms → seconds

  fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
  ax1.plot(t, df['IR_Raw'],    label='IR PPG')
  ax2.plot(t, df['Piezo_Raw'], label='Piezo RESP')
  plt.show()

  # Do NOT use Time_Seconds for waveform alignment — it reflects USB arrival
  # time, not capture time. TS_Arduino_ms is the correct time axis.

════════════════════════════════════════════════════════════════════════════════
DEPENDENCIES
════════════════════════════════════════════════════════════════════════════════
  pyserial   >= 3.5   — pip install pyserial
  PyQt5      >= 5.15  — pip install PyQt5
  pyqtgraph  >= 0.13  — pip install pyqtgraph

  Compatible with Spyder IDE: uses QApplication.instance() so a second
  QApplication is not created when IPython already manages one.

════════════════════════════════════════════════════════════════════════════════
HARDWARE CONTEXT
════════════════════════════════════════════════════════════════════════════════
  Board  : RobotDyn Mega+WiFi (ATmega2560 + ESP8266 on-board co-processor)
  USB    : ATmega2560 USB-Serial bridge (same port used for Arduino IDE upload)
  Baud   : 115200 — must match Serial.begin() in surgery_monitor.ino
  Stream : JSON lines at 100 Hz, each terminated by '\\n'
"""

import sys
import json
import time
import csv
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QFileDialog,
                             QFrame, QComboBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import pyqtgraph as pg


# ════════════════════════════════════════════════════════════════════════════
# SERIAL READING THREAD
# ════════════════════════════════════════════════════════════════════════════

class SerialThread(QThread):
    """
    Background QThread that reads the USB-Serial port and emits one dict per
    valid JSON packet. Runs independently of the Qt event loop so the GUI
    remains responsive at 100 Hz stream rate.

    Signals:
        data_received(dict)   — emitted for every successfully parsed packet.
        connection_error(str) — emitted on serial open failure or read error.

    Thread lifecycle:
        start()  — opens the port and enters the read loop.
        stop()   — sets self.running = False; the run() loop exits on the next
                   iteration and the finally block closes the port cleanly.
        wait()   — called from closeEvent() to ensure the thread has exited
                   before the application terminates.
    """

    data_received    = pyqtSignal(dict)
    connection_error = pyqtSignal(str)

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port     = port
        self.baudrate = baudrate
        self.ser      = None
        self.running  = True

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            # Flush any bytes accumulated in the OS receive buffer before this
            # session started — avoids processing stale packets from a previous
            # firmware run or Arduino IDE Serial Monitor session.
            self.ser.reset_input_buffer()

            while self.running:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    # Basic structural check before attempting JSON parse.
                    # Rejects partial lines from board resets mid-packet.
                    if line.startswith('{') and line.endswith('}'):
                        try:
                            data = json.loads(line)
                            self.data_received.emit(data)
                        except json.JSONDecodeError:
                            pass   # Silently discard malformed packets.

        except Exception as e:
            self.connection_error.emit(str(e))
        finally:
            # Guaranteed port close regardless of how the thread exits
            # (normal stop, exception, or external termination).
            if self.ser and self.ser.is_open:
                self.ser.close()

    def stop(self):
        """Signal the run() loop to exit on its next iteration."""
        self.running = False


# ════════════════════════════════════════════════════════════════════════════
# MAIN APPLICATION WINDOW
# ════════════════════════════════════════════════════════════════════════════

class PyVitalSignsDAQ(QMainWindow):
    """
    Main application window. Owns the UI layout, the serial thread lifecycle,
    waveform buffers, CSV writers, and the event marker system.

    Layout:
        Top bar   — port selector, connect/disconnect, event marker, pause, REC
        Left sidebar — four numeric readout cards (HR, SpO2, RR, Temp)
        Right area   — two stacked pyqtgraph PlotWidgets with linked X axes
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Py Vital-signs — USB Monitor")
        self.resize(1200, 750)
        self.setStyleSheet("background-color: #000000; color: #FFFFFF;")

        # WA_DeleteOnClose ensures Qt destroys the window object when closed,
        # releasing all child widgets and preventing memory leaks on re-open.
        self.setAttribute(Qt.WA_DeleteOnClose)

        # ── Recording state ───────────────────────────────────────────────────
        self.is_paused    = False
        self.is_recording = False
        self.event_flag   = False   # Set by mark_event(); consumed in update_gui()

        self.csv_vitals  = None
        self.csv_raw     = None
        self.file_vitals = None
        self.file_raw    = None
        self.start_time  = 0       # wall-clock time at REC start (time.time())

        # ── Timestamp alignment state ─────────────────────────────────────────
        # The firmware sends "ts" = millis() at packet serialisation time.
        # Both ir and pz in a given packet share this timestamp as their
        # common capture reference (they were read in the same loop() iteration).
        #
        # ts_origin_arduino: "ts" value of the first packet after REC start.
        #   Used to normalise TS_Arduino_ms to 0 at the beginning of each
        #   recording session. Set to None until the first packet arrives.
        #
        # ts_origin_wall: time.time() captured at the same moment as
        #   ts_origin_arduino. Stored for post-processing cross-reference
        #   between Arduino time and wall-clock time if needed.
        self.ts_origin_arduino = None
        self.ts_origin_wall    = None

        # ── Waveform display buffers ──────────────────────────────────────────
        # Three parallel lists, always the same length (max_points).
        # Older samples are discarded via pop(0) as new ones arrive,
        # producing a scrolling window of the last max_points packets.
        self.max_points = 300          # ~3 s of history at 100 Hz
        self.ir_data    = [0] * self.max_points
        self.pz_data    = [0] * self.max_points

        # ts_data: Arduino timestamps (ms) for the X axis of both plots.
        # Initialised to a 10 ms linear ramp so pyqtgraph has valid X data
        # before the first real packet arrives and does not divide by zero.
        self.ts_data = list(range(0, self.max_points * 10, 10))  # 0..2990 ms

        # ── Event marker state ────────────────────────────────────────────────
        # Each element is a tuple (InfiniteLine, ts_ms) where ts_ms is the
        # Arduino timestamp at which the event was marked. Lines are pruned
        # when their timestamp scrolls out of the visible ts_data window.
        self.event_lines_ir = []
        self.event_lines_pz = []
        self.data_index     = 0    # Running packet counter (used for bookkeeping)

        self.serial_thread = None

        self.init_ui()

    # ─────────────────────────────────────────────────────────────────────────
    # UI CONSTRUCTION
    # ─────────────────────────────────────────────────────────────────────────

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

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

        # Shown between connect and the stretch — cleared on first data packet.
        # Provides visual feedback during the ~3 s firmware initialisation delay.
        self.lbl_init_msg = QLabel("")
        self.lbl_init_msg.setStyleSheet(
            "color: #cccccc; font-size: 16px; font-weight: bold; margin-left: 30px;")

        # Event marker — enabled only during active recording.
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
        self.val_temp = self.create_box(sidebar, "Temperature", "°C",  "#800080")

        body_layout.addLayout(sidebar, stretch=1)

        # Right area — waveform charts
        charts_layout = QVBoxLayout()

        # IR-PPG chart.
        # X axis uses Arduino timestamps (ms) for true temporal alignment.
        # The pyqtgraph X range is updated every packet via setXRange() on
        # plot_ir only — plot_pz inherits the range via setXLink().
        self.plot_ir = pg.PlotWidget(
            title='<span style="color:#b30000;font-size:11pt;font-weight:bold;">'
                  'PPG — IR Optical (MAX30102)</span>')
        self.plot_ir.setBackground('#0d0d0d')
        self.plot_ir.showGrid(x=True, y=True, alpha=0.2)
        self.plot_ir.getAxis('bottom').setLabel('Time (ms, Arduino clock)')
        self.curve_ir = self.plot_ir.plot(pen=pg.mkPen('#b30000', width=2))
        charts_layout.addWidget(self.plot_ir)

        # Piezo respiratory chart.
        # Linked to plot_ir so panning or zooming either chart moves both.
        # This makes temporal misalignment between the two signals immediately
        # visible as a lateral offset between the waveform peaks.
        self.plot_pz = pg.PlotWidget(
            title='<span style="color:#cca300;font-size:11pt;font-weight:bold;">'
                  'RESP — Piezo Belt</span>')
        self.plot_pz.setBackground('#0d0d0d')
        self.plot_pz.showGrid(x=True, y=True, alpha=0.2)
        self.plot_pz.getAxis('bottom').setLabel('Time (ms, Arduino clock)')
        self.curve_pz = self.plot_pz.plot(pen=pg.mkPen('#cca300', width=2))
        charts_layout.addWidget(self.plot_pz)

        # Link X axes — one setXRange() call on plot_ir controls both panels.
        self.plot_pz.setXLink(self.plot_ir)

        body_layout.addLayout(charts_layout, stretch=4)

    def create_box(self, layout, title, unit, color):
        """
        Build a numeric readout card and add it to layout.
        Returns the QLabel that displays the value so callers can update it.

        Visual structure:
            ┌─ 6px left border (color) ──────────────────┐
            │ TITLE (12px, uppercase, muted)              │
            │                               VALUE (52px)  │
            │                                UNIT (16px)  │
            └────────────────────────────────────────────┘
        """
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
        """Populate the port combo box with currently available serial ports."""
        self.combo_ports.clear()
        for port in serial.tools.list_ports.comports():
            self.combo_ports.addItem(port.device)

    def toggle_connection(self):
        """
        Connect or disconnect the serial thread.

        On connect: creates a new SerialThread, wires its signals, starts it,
        and shows the "Initializing..." label. The label is cleared in
        update_gui() when the first data packet arrives.

        On disconnect: stops the thread, resets the timestamp alignment state
        so the next session starts with a clean epoch.
        """
        if self.serial_thread is None or not self.serial_thread.running:
            port = self.combo_ports.currentText()
            if port:
                self.serial_thread = SerialThread(port)
                self.serial_thread.data_received.connect(self.update_gui)
                self.serial_thread.connection_error.connect(self.on_connection_error)
                self.serial_thread.start()

                self.lbl_init_msg.setText("Initializing...")
                self.btn_connect.setText("DISCONNECT")
                self.btn_connect.setStyleSheet(
                    "background-color: #ff8c00; color: #fff; padding: 8px; "
                    "font-weight: bold; border: none; border-radius: 4px;")
        else:
            self.serial_thread.stop()
            self.serial_thread = None

            # Reset alignment epoch — the next connect will re-establish it
            # from the first packet received in the new session.
            self.ts_origin_arduino = None
            self.ts_origin_wall    = None

            self.lbl_init_msg.setText("")
            self.btn_connect.setText("CONNECT")
            self.btn_connect.setStyleSheet(
                "background-color: #0055a4; color: #fff; padding: 8px; "
                "font-weight: bold; border: none; border-radius: 4px;")

    def on_connection_error(self, err_msg):
        """Slot called when SerialThread encounters a port open or read error."""
        self.btn_connect.setText("ERROR — TRY AGAIN")
        self.btn_connect.setStyleSheet(
            "background-color: #cc0000; color: #fff; padding: 8px; "
            "font-weight: bold; border: none; border-radius: 4px;")
        self.lbl_init_msg.setText("")
        print(f"Serial error: {err_msg}")

    # ─────────────────────────────────────────────────────────────────────────
    # PLAYBACK CONTROL
    # ─────────────────────────────────────────────────────────────────────────

    def toggle_pause(self):
        """
        Pause or resume the waveform display and CSV logging.
        The serial thread continues reading — packets received during pause
        are discarded in update_gui() without updating buffers or files.
        """
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
        """
        Start or stop CSV recording.

        On start: opens two CSV files, writes headers, resets the Arduino
        timestamp epoch (ts_origin_arduino) so TS_Arduino_ms starts at 0,
        and enables the event marker button.

        On stop: closes both files and nullifies the csv.writer references
        to prevent accidental writes to closed file objects.

        File naming: the user selects a base path without extension.
        The two files are created as <base>_vitals.csv and <base>_raw.csv.
        """
        if not self.is_recording:
            base_filepath, _ = QFileDialog.getSaveFileName(
                self, "Save Surgical Record", "", "CSV Files (*.csv)",
                options=QFileDialog.Options())

            if base_filepath:
                if base_filepath.endswith('.csv'):
                    base_filepath = base_filepath[:-4]
                try:
                    self.file_vitals = open(
                        f"{base_filepath}_vitals.csv", mode='w', newline='')
                    self.file_raw = open(
                        f"{base_filepath}_raw.csv", mode='w', newline='')

                    self.csv_vitals = csv.writer(self.file_vitals)
                    self.csv_raw    = csv.writer(self.file_raw)

                    # Vitals header — wall-clock time axis
                    self.csv_vitals.writerow([
                        'Time_Seconds', 'HeartRate_BPM', 'SpO2_%',
                        'RespRate_RPM', 'Temperature_C', 'Event_Marker'])

                    # Raw header — Arduino timestamp as primary time axis.
                    # TS_Arduino_ms is normalised to 0 at REC start and must be
                    # used (not Time_Seconds) for waveform alignment in analysis.
                    self.csv_raw.writerow([
                        'Time_Seconds', 'TS_Arduino_ms',
                        'IR_Raw', 'Piezo_Raw', 'Event_Marker'])

                    self.start_time         = time.time()
                    self.ts_origin_arduino  = None   # Set on first packet
                    self.ts_origin_wall     = None

                    self.is_recording = True
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
                    self.data_index = 0

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
            # Nullify writers to prevent accidental writes to closed file objects.
            self.csv_vitals = None
            self.csv_raw    = None

    # ─────────────────────────────────────────────────────────────────────────
    # EVENT MARKER
    # ─────────────────────────────────────────────────────────────────────────

    def mark_event(self):
        """
        Flag the current moment as a surgical event (drug injection, stimulus,
        incision, etc.). Sets event_flag for the next CSV row and draws a
        vertical marker line on both waveform plots.
        """
        if self.is_recording:
            self.event_flag = True
            self.draw_event_line()

    def draw_event_line(self):
        """
        Place a vertical InfiniteLine at the current rightmost timestamp on
        both waveform plots. The line is stored with its ts_ms value.

        Because the X axis is real Arduino time (ts_data), the line remains
        visually anchored to that timestamp as the waveform scrolls — no
        manual position update is needed in update_gui(). Lines are pruned
        when their timestamp exits the visible ts_data window.
        """
        ts_now = self.ts_data[-1] if self.ts_data else 0
        line_pen = pg.mkPen(color=(255, 255, 255, 100), width=2)

        vLineIR = pg.InfiniteLine(pos=ts_now, angle=90, movable=False, pen=line_pen)
        vLinePZ = pg.InfiniteLine(pos=ts_now, angle=90, movable=False, pen=line_pen)

        self.plot_ir.addItem(vLineIR)
        self.plot_pz.addItem(vLinePZ)

        self.event_lines_ir.append((vLineIR, ts_now))
        self.event_lines_pz.append((vLinePZ, ts_now))

    def clear_event_lines(self):
        """Remove all event marker lines from both plots and reset the lists."""
        for line, _ in self.event_lines_ir:
            self.plot_ir.removeItem(line)
        for line, _ in self.event_lines_pz:
            self.plot_pz.removeItem(line)
        self.event_lines_ir.clear()
        self.event_lines_pz.clear()

    # ─────────────────────────────────────────────────────────────────────────
    # MAIN UPDATE — called at 100 Hz by SerialThread signal
    # ─────────────────────────────────────────────────────────────────────────

    def update_gui(self, data):
        """
        Process one incoming JSON packet. Called in the main thread via Qt
        signal/slot (SerialThread emits data_received → this slot).

        Execution order per packet:
          1. Clear "Initializing..." label on first arrival.
          2. Extract all fields; apply defaults for missing keys.
          3. Extract and validate the "ts" timestamp.
          4. Update numeric readout labels.
          5. Append to waveform buffers; discard oldest sample.
          6. Redraw both waveform curves with the updated ts_data X axis.
          7. Update the visible X range on plot_ir (plot_pz follows via XLink).
          8. Prune event lines that have scrolled out of view.
          9. If recording: write one row to each CSV file.
        """
        # Step 1 — clear the "Initializing..." label on first data arrival.
        if self.lbl_init_msg.text():
            self.lbl_init_msg.setText("")

        if self.is_paused:
            return

        # Step 2 — field extraction with safe defaults.
        hr = data.get('hr', 0)
        sp = data.get('sp', 0)
        rr = data.get('rr', 0)
        t  = data.get('t',  0)
        ir = data.get('ir', 0)
        pz = data.get('pz', 0)

        # Step 3 — timestamp extraction.
        # "ts" is millis() from the ATmega2560 — capture time, not arrival time.
        # Packets from firmware without "ts" use a synthesised 10 ms increment
        # so the display remains functional without alignment correction.
        ts_raw = data.get('ts', None)
        if ts_raw is None:
            ts_ms = (self.ts_data[-1] + 10) if self.ts_data else 0
        else:
            ts_ms = int(ts_raw)

        # Step 4 — numeric readout labels.
        self.val_hr.setText(str(hr)      if hr > 0 else "--")
        self.val_spo2.setText(str(sp)    if sp > 0 else "--")
        self.val_rr.setText(str(rr)      if rr > 0 else "--")
        self.val_temp.setText(f"{t:.1f}" if t  > 0 else "--.-")

        # Step 5 — waveform buffer update (sliding window, oldest discarded).
        self.ir_data.append(ir);    self.ir_data.pop(0)
        self.pz_data.append(pz);    self.pz_data.pop(0)
        self.ts_data.append(ts_ms); self.ts_data.pop(0)

        # Step 6 — redraw waveforms with real-time X axis.
        # setData(x, y) maps each sample to its Arduino capture timestamp,
        # aligning ir and pz to the same clock regardless of USB latency.
        self.curve_ir.setData(self.ts_data, self.ir_data)
        self.curve_pz.setData(self.ts_data, self.pz_data)

        # Step 7 — update the visible X range.
        # plot_pz follows via setXLink — only one call needed.
        self.plot_ir.setXRange(self.ts_data[0], self.ts_data[-1], padding=0)

        # Step 8 — prune event lines that have scrolled out of the visible window.
        visible_start = self.ts_data[0]
        for collection, plot in [(self.event_lines_ir, self.plot_ir),
                                  (self.event_lines_pz, self.plot_pz)]:
            for line, ts in list(collection):
                if ts < visible_start:
                    plot.removeItem(line)
            collection[:] = [(l, ts) for l, ts in collection if ts >= visible_start]

        # Step 9 — CSV logging.
        if self.is_recording:
            # Capture the Arduino epoch on the first packet of this session.
            # ts_origin_arduino is reset to None in toggle_record() start branch.
            if self.ts_origin_arduino is None:
                self.ts_origin_arduino = ts_ms
                self.ts_origin_wall    = time.time()

            # TS_Arduino_ms: normalised to 0 at REC start.
            # This is the authoritative time axis for waveform analysis.
            ts_arduino_ms = ts_ms - self.ts_origin_arduino

            # Wall-clock elapsed time: for the vitals file and cross-referencing.
            current_time = round(time.time() - self.start_time, 3)

            # Consume the event flag — reset immediately so only one row is marked.
            if self.event_flag:
                event_vital_str = "Event Start"
                event_raw_val   = 1
                self.event_flag = False
            else:
                event_vital_str = ""
                event_raw_val   = 0

            # Vitals CSV — wall-clock time axis (session management reference).
            self.csv_vitals.writerow(
                [current_time, hr, sp, rr, t, event_vital_str])

            # Raw CSV — Arduino timestamp axis (waveform alignment reference).
            # Do NOT replace ts_arduino_ms with current_time for signal analysis.
            self.csv_raw.writerow(
                [current_time, ts_arduino_ms, ir, pz, event_raw_val])

    # ─────────────────────────────────────────────────────────────────────────
    # APPLICATION SHUTDOWN
    # ─────────────────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        """
        Intercept the window close event to ensure clean shutdown:
          1. Stop recording and flush CSV files if a session is active.
          2. Signal the serial thread to exit its read loop.
          3. Block until the thread has exited (wait()) before accepting the
             close event — prevents the OS from killing the thread mid-write.
        """
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
    # Spyder / IPython compatibility: reuse the existing QApplication instance
    # if one is already running. Creating a second QApplication raises a
    # RuntimeError in PyQt5 and crashes the kernel.
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    window = PyVitalSignsDAQ()
    window.show()
    app.exec_()