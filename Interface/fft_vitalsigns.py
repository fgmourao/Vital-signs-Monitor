"""
fft_vitalsigns.py
=================
PROJECT : Vital-signs monitor — small rodents (rat / mouse)
AUTHOR  : Flávio Mourão — Sep, 2026

════════════════════════════════════════════════════════════════════════════════
MODULE RESPONSIBILITY
════════════════════════════════════════════════════════════════════════════════
Live power spectral density viewer for the two physiological waveforms
acquired by the vital-signs monitor:

  IR-PPG  (red)  — MAX30102 optical signal. The dominant spectral peak
                   corresponds to heart rate. Expected range:
                     Human:     0.7–3.3 Hz  (40–200 BPM)
                     Rat:       3.3–8.3 Hz  (200–500 BPM)
                     Mouse:     4.2–13.3 Hz (250–800 BPM)

  Resp.   (gold) — Respiratory sensor signal. The dominant
                   peak corresponds to respiratory rate. Signal is in mV,
                   DC-baseline removed and inverted (inspiration = positive
                   peak) — same processing as the main display.
                   Expected range:
                     Rat anaest.:   1.2–1.5 Hz (70–90 rpm)
                     Mouse anaest.: 1.3–2.0 Hz (80–120 rpm)

No filtering is applied before the PSD — the signals are taken raw from
the same buffers used by the main waveform display.

════════════════════════════════════════════════════════════════════════════════
DESIGN
════════════════════════════════════════════════════════════════════════════════
This window is a standalone QDialog that reads directly from the parent
PyVitalSignsDAQ window's live buffers (ir_data, pz_data, ts_data).
It does NOT open its own serial connection — exactly one source of truth.

PSD method: Welch (scipy.signal.welch), matching the offline MATLAB
pwelch() workflow used in the analysis scripts for this project.
Detrend (linear) is applied by default to suppress slow DC drift from
inflating the lowest-frequency bins.

X axis is limited to 0–30 Hz, covering:
  - HR up to 1800 BPM (well above any biological range)
  - RR up to 1800 rpm (well above any biological range)
Frequency resolution: Δf ≈ 1 / Segment(s).

════════════════════════════════════════════════════════════════════════════════
REQUIREMENTS
════════════════════════════════════════════════════════════════════════════════
  numpy, scipy, pyqtgraph, PyQt5  (all required by Py_VitalSigns_DAQ.py)
"""

import numpy as np
from scipy.signal import welch, detrend as sp_detrend
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QPushButton
)
from PyQt5.QtCore import QTimer


class FFTVitalSignsWindow(QDialog):
    """
    Live PSD viewer for IR-PPG (heart rate) and piezo (respiratory rate).

    Two curves are always shown simultaneously:
      Red  — IR-PPG power spectrum
      Gold — Respiratory sensor power spectrum

    Controls:
      Window (s)      — history length fed into Welch's method. More history
                        → more segments averaged → smoother PSD estimate.
      Update (ms)     — redraw interval.
      Segment (s)     — nperseg in seconds. Sets frequency resolution:
                        Δf ≈ 1 / Segment(s). Must be ≤ Window(s).
      Spectral window — FFT window function per segment (Hamming default,
                        matching MATLAB pwelch()).
      Detrend         — remove linear trend before PSD to suppress DC drift.
      Log Y           — logarithmic Y axis (recommended for PSD comparison).
      X max (Hz)      — upper frequency limit of the displayed X axis.

    Attributes:
        daq : PyVitalSignsDAQ
            Reference to the parent DAQ window. Buffers are read directly:
            daq.ir_data, daq.pz_data, daq.ts_data (all lists, max_points long).
    """

    def __init__(self, daq_window, parent=None):
        super().__init__(parent)
        self.daq = daq_window
        self.setWindowTitle("Live PSD — Heart Rate & Respiratory Rate")
        self.resize(800, 540)
        self.setStyleSheet("""
            QDialog    { background-color: #111111; color: #cccccc; }
            QLabel     { color: #aaaaaa; }
            QCheckBox  { color: #aaaaaa; }
            QDoubleSpinBox, QSpinBox, QComboBox {
                background-color: #222; color: #fff;
                border: 1px solid #444; border-radius: 3px; padding: 2px; }
            QPushButton {
                padding: 5px 14px; border-radius: 4px;
                font-weight: bold; border: none; }
        """)

        layout = QVBoxLayout(self)

        # ── Controls ──────────────────────────────────────────────────────────
        ctrl = QHBoxLayout()

        ctrl.addWidget(QLabel("Window (s):"))
        self.spin_window = QDoubleSpinBox()
        self.spin_window.setRange(1.0, 120.0)
        self.spin_window.setSingleStep(1.0)
        self.spin_window.setValue(5.0)
        self.spin_window.setToolTip(
            "How much signal history (seconds) is fed into Welch's method.\n"
            "More history → more segments averaged → smoother PSD.\n"
            "Must be ≥ Segment (s).")
        ctrl.addWidget(self.spin_window)

        ctrl.addWidget(QLabel("Update (ms):"))
        self.spin_update = QSpinBox()
        self.spin_update.setRange(100, 5000)
        self.spin_update.setSingleStep(100)
        self.spin_update.setValue(500)
        self.spin_update.valueChanged.connect(
            lambda v: self.timer.setInterval(v))
        ctrl.addWidget(self.spin_update)

        ctrl.addWidget(QLabel("Segment (s):"))
        self.spin_segment = QDoubleSpinBox()
        self.spin_segment.setRange(0.5, 30.0)
        self.spin_segment.setSingleStep(0.5)
        self.spin_segment.setValue(3.0)
        self.spin_segment.setToolTip(
            "Length of each FFT segment inside Welch's method (nperseg).\n"
            "Sets frequency resolution: Δf ≈ 1 / Segment(s).\n"
            "Longer → finer resolution but noisier (fewer segments averaged).\n"
            "Shorter → coarser resolution but smoother estimate.")
        ctrl.addWidget(self.spin_segment)

        ctrl.addWidget(QLabel("Window fn:"))
        self.combo_winfn = QComboBox()
        self.combo_winfn.addItems(["hamming", "hann", "blackman", "boxcar"])
        self.combo_winfn.setToolTip(
            "Window function applied to each segment before its FFT.\n"
            "Hamming matches MATLAB pwelch() default.")
        ctrl.addWidget(self.combo_winfn)

        ctrl.addWidget(QLabel("X max (Hz):"))
        self.spin_xmax = QDoubleSpinBox()
        self.spin_xmax.setRange(1.0, 30.0)
        self.spin_xmax.setSingleStep(1.0)
        self.spin_xmax.setValue(20.0)
        self.spin_xmax.setToolTip(
            "Upper frequency limit of the X axis.\n"
            "Human HR: up to ~3 Hz | Rat HR: up to ~8 Hz | Mouse HR: up to ~13 Hz\n"
            "Respiratory rate: typically < 2 Hz for anaesthetised rodents.")
        self.spin_xmax.valueChanged.connect(self._apply_xrange)
        ctrl.addWidget(self.spin_xmax)

        self.chk_detrend = QCheckBox("Detrend")
        self.chk_detrend.setChecked(True)
        self.chk_detrend.setToolTip(
            "Remove linear trend before PSD computation.\n"
            "Prevents slow DC drift from inflating the lowest-frequency bins.")
        ctrl.addWidget(self.chk_detrend)

        self.chk_log = QCheckBox("Log Y")
        self.chk_log.setChecked(True)
        self.chk_log.stateChanged.connect(self._apply_log)
        ctrl.addWidget(self.chk_log)

        ctrl.addStretch()
        layout.addLayout(ctrl)

        # ── Plot ──────────────────────────────────────────────────────────────
        self.plot = pg.PlotWidget()
        self.plot.setBackground('#0d0d0d')
        self.plot.setLabel('bottom', 'Frequency', 'Hz')
        self.plot.setLabel('left', 'PSD')
        # Signal units per channel:
        #   IR   — raw 18-bit counts from MAX30102 photodetector.
        #          PSD units: counts²/Hz.
        #   Resp. — mV after DC removal and polarity correction (see update_gui Step 2b).
        #           PSD units: mV²/Hz.
        # Both are suitable for spectral peak detection; absolute PSD magnitudes
        # are not directly comparable between channels due to different units.
        self.plot.getAxis('left').enableAutoSIPrefix(False)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend(offset=(10, 10))

        self.curve_ir = self.plot.plot(
            pen=pg.mkPen('#b30000', width=2), name='IR-PPG (Heart Rate)')
        self.curve_pz = self.plot.plot(
            pen=pg.mkPen('#cca300', width=2), name='Respiratory Rate')

        layout.addWidget(self.plot, 1)

        # ── Status label ──────────────────────────────────────────────────────
        self.lbl_status = QLabel("Waiting for data...")
        self.lbl_status.setStyleSheet("color: #666; font-size: 11px; padding: 2px;")
        layout.addWidget(self.lbl_status)

        self._apply_log()
        self._apply_xrange()

        # ── Update timer ──────────────────────────────────────────────────────
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update)
        self.timer.start(self.spin_update.value())

    # ─────────────────────────────────────────────────────────────────────────

    def _apply_log(self):
        self.plot.setLogMode(x=False, y=self.chk_log.isChecked())

    def _apply_xrange(self):
        self.plot.setXRange(0, self.spin_xmax.value(), padding=0)

    def _compute_psd(self, values, fs):
        """
        Compute Welch PSD for a 1-D signal at sampling rate fs.
        Returns (freq, pxx) arrays, or (None, None) if insufficient data.
        """
        v = np.asarray(values, dtype=float)
        if len(v) < 10:
            return None, None
        if self.chk_detrend.isChecked():
            v = sp_detrend(v, type='linear')
        nperseg = int(round(self.spin_segment.value() * fs))
        nperseg = max(min(nperseg, len(v)), 4)
        freq, pxx = welch(
            v, fs=fs,
            window=self.combo_winfn.currentText(),
            nperseg=nperseg
        )
        return freq, pxx

    def _update(self):
        """
        Recompute PSD for both channels and redraw.
        Reads directly from the DAQ window's live buffers — no data copy,
        no serial access.
        """
        ts   = self.daq.ts_data    # Arduino timestamps (ms)
        ir   = self.daq.ir_data    # IR raw counts from MAX30102
        pz   = self.daq.pz_data    # Respiratory sensor in mV, DC-removed, polarity-corrected

        # Need at least a few samples to estimate fs and compute a PSD.
        if len(ts) < 20:
            self.lbl_status.setText("Waiting for data...")
            return

        # Derive sampling rate from the Arduino timestamp differences (ms → Hz).
        ts_arr = np.asarray(ts, dtype=float)
        diffs  = np.diff(ts_arr)
        diffs  = diffs[diffs > 0]   # guard against duplicate timestamps
        if len(diffs) == 0:
            self.lbl_status.setText("Waiting for valid timestamps...")
            return
        fs = 1000.0 / np.median(diffs)   # median is robust to occasional gaps

        # Select the most recent window_s seconds of data.
        window_s  = self.spin_window.value()
        t_end     = ts_arr[-1]
        mask      = ts_arr >= (t_end - window_s * 1000.0)   # ts is in ms
        idx_start = int(np.where(mask)[0][0]) if mask.any() else 0

        ir_win = ir[idx_start:]
        pz_win = pz[idx_start:]

        if len(ir_win) < 10:
            self.lbl_status.setText("Not enough samples in the window yet...")
            return

        freq_ir, pxx_ir = self._compute_psd(ir_win, fs)
        freq_pz, pxx_pz = self._compute_psd(pz_win, fs)

        if freq_ir is None or freq_pz is None:
            return

        self.curve_ir.setData(freq_ir, pxx_ir)
        self.curve_pz.setData(freq_pz, pxx_pz)

        # Peak frequency for each channel — most likely the dominant rhythm.
        xmax = self.spin_xmax.value()
        mask_f_ir = freq_ir <= xmax
        mask_f_pz = freq_pz <= xmax

        peak_ir_hz = (freq_ir[mask_f_ir][np.argmax(pxx_ir[mask_f_ir])]
                      if mask_f_ir.any() else 0.0)
        peak_pz_hz = (freq_pz[mask_f_pz][np.argmax(pxx_pz[mask_f_pz])]
                      if mask_f_pz.any() else 0.0)

        delta_f = fs / int(round(self.spin_segment.value() * fs))
        self.lbl_status.setText(
            f"fs ≈ {fs:.1f} Hz  |  N = {len(ir_win)} samples  |  "
            f"Δf ≈ {delta_f:.3f} Hz  |  "
            f"IR peak: {peak_ir_hz:.2f} Hz ({peak_ir_hz * 60:.0f} BPM)  |  "
            f"Resp. peak: {peak_pz_hz:.2f} Hz ({peak_pz_hz * 60:.0f} rpm)  "
            f"[IR: counts²/Hz  |  Resp.: mV²/Hz]")

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)
