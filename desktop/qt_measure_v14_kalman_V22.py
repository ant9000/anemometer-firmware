#!/usr/bin/env python3

import os, sys, getopt, time, io, queue
from datetime import datetime
import json, threading, traceback
import paho.mqtt.client as mqtt
from scipy import stats as st
import numpy as np
from scipy.interpolate import interp1d
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

DIST = { # in micron
    'x0': 102000, 'x1': 102000,
    'y0': 102000, 'y1': 102000,
    'z0': 102000, 'z1': 102000,
}
SAMPLE = { # in samples
    'x0': 28, 'x1': 28,
    'y0': 28, 'y1': 28,
    'z0': 28, 'z1': 28,
}

try:
    raise Exception("")
    from matplotlib.backends.backend_qtagg import FigureCanvas
    from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
except:
    from matplotlib.backends.backend_qt5agg import FigureCanvas
    from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.backends.qt_compat import QtCore, QtGui, QtWidgets
from matplotlib.figure import Figure
import matplotlib.dates as dates

class MQTTClient:
    def __init__(self, host, port, channel, parse_message):
        self.client = mqtt.Client()
        self.client.message_callback_add(channel, self.on_message)
        self.client.connect(host, port)
        self.client.subscribe(channel)
        self.channel = channel
        self.parse_message = parse_message
    def start(self):
        self._t = threading.Thread(target=self.client.loop_forever, daemon=True)
        self._t.start()
    def stop(self):
        self.client.disconnect()
    def publish(self, message):
        self.client.publish(self.channel, message)
    def on_message(self, client, userdata, msg):
        try:
            payload = str(msg.payload.decode("utf-8"))
            data = json.loads(payload)
            self.parse_message(data)
        except Exception as e:
            import traceback
            print(f"ERROR: {e}")
            traceback.print_exc()

def polar(i, q):
    SAMPLES = 80
    c = np.array(i) + np.array(q) * 1j
    c = np.pad(c, (0, SAMPLES), mode="constant", constant_values=0)[:SAMPLES]
    return (np.abs(c), np.angle(c, deg=False))

def polar2(i,q):
    rho = np.sqrt(i**2 + q**2)
    phi = np.arctan2(q, i)
    return rho, phi


def trova_punto_flesso(interp_rho, interp_phi):
    """
    Trova il punto di flesso nella parte crescente della curva.
    Si assume che il punto di flesso corrisponda al punto di massima pendenza,
    che per molte curve sigmoidi avviene intorno a metà ampiezza.
    
    Parameters:
        interp_rho (numpy.array): Vettore delle ampiezze.
        interp_phi (numpy.array): Vettore delle fasi corrispondenti.
    
    Returns:
        idx_inflex (int): Indice del punto di flesso.
        fase_inflex (float): Fase corrispondente al punto di flesso.
    """
    # Trova l'indice del massimo valore dell'ampiezza (picco)
    indice_max = np.argmax(interp_rho)
    
    # Considera solo la parte crescente, dall'inizio fino al picco
    rho_rising = interp_rho[:indice_max+1]
    phi_rising = interp_phi[:indice_max+1]
    
    # Calcola la derivata prima (differenze successive) della parte crescente
    d_rho = np.diff(rho_rising)
    
    # Il punto di massima pendenza, ovvero il punto di flesso,
    # è dato dall'indice in cui d_rho è massimo.
    # Poiché np.diff restituisce un array con lunghezza ridotta di 1,
    # aggiungiamo +1 per mappare sull'array originale.
    idx_inflex = np.argmax(d_rho) + 1
    fase_inflex = phi_rising[idx_inflex]
    
    return idx_inflex, fase_inflex

class KalmanFilter:
    def __init__(self, A=1, H=1, R=4):
        self.A = A
        self.H = H
        self.R = R
        self.x_est = None
    def update(self, x, Q):
        # Prediction
        if self.x_est is None or np.isnan(self.x_est):
            self.x_est = x
            self.P = 1.0
        else:
            x_pred = self.A * self.x_est
            P_pred = self.A * self.P * self.A + Q
            # Kalman Gain
            K = P_pred * self.H / (self.H * P_pred * self.H + self.R)
            
            # Update
            self.x_est = x_pred + K * (x - self.H * x_pred)
            self.P = (1 - K * self.H) * P_pred
        return self.x_est


class ApplicationWindow(QtWidgets.QMainWindow):
    log = QtCore.Signal(str)
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DIET Sapienza - 3D Anemometer")
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        self.canvas_kalman = FigureCanvas(Figure(figsize=(10, 3)))
        self.canvas_amp = FigureCanvas(Figure(figsize=(10, 3)))
        self.host = QtWidgets.QLineEdit()
        self.host.setPlaceholderText("Anemometer name or IP")
        self.host.setText("localhost")
        self.port = QtWidgets.QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(1883)
        self.connect = QtWidgets.QPushButton()
        self.connect.setText("Connect")
        self.connect.clicked.connect(self.on_click_connect)
        self.connect.setDefault(True)
        self.axes_sel = {}
        for axis in "xyz":
            self.axes_sel[axis] = QtWidgets.QCheckBox(axis)
            self.axes_sel[axis].setCheckState(QtCore.Qt.CheckState.Checked)
            self.axes_sel[axis].clicked.connect(self.init_canvas)
        self.duration = QtWidgets.QSpinBox()
        self.duration.setRange(60, 3600)
        self.duration.setValue(120)
        self.autoscroll = QtWidgets.QCheckBox("Autoscroll")
        self.autoscroll.setCheckState(QtCore.Qt.CheckState.Checked)
        self.logfile = QtWidgets.QLineEdit()
        self.logfile.setReadOnly(True)
        self.logfile.setPlaceholderText("Save output to file")
        self.choose = QtWidgets.QPushButton()
        self.choose.setText("Choose")
        self.choose.clicked.connect(self.choose_logfile)
        self.q_label = QtWidgets.QLabel("Kalman Q:")
        self.q_input = QtWidgets.QDoubleSpinBox()
        self.q_input.setDecimals(6)
        self.q_input.setRange(0.000001, 100.0)
        self.q_input.setValue(0.05)  # Default Kalman Q
        self.q_input.setSingleStep(0.0001)
        self.q_kalman = self.q_input.value()
        self.q_input.valueChanged.connect(self.update_q_kalman)
        # Pannello per n campioni
        self.n_label = QtWidgets.QLabel("N campioni:")
        self.n_input = QtWidgets.QSpinBox()
        self.n_input.setRange(1, 1000)
        self.n_input.setValue(30)
        self.n_input.setSingleStep(1)
        self.n_campioni = self.n_input.value()
        self.n_input.valueChanged.connect(self.update_n_campioni)
        self.main_button = QtWidgets.QPushButton()
        self.main_button.setText("Start")
        self.main_button.clicked.connect(self.on_click)
        self.main_button.setDisabled(True)
        self.output = QtWidgets.QPlainTextEdit()
        self.output.centerOnScroll()
        self.output.setReadOnly(True)
        self.output.setCenterOnScroll(False)
        self.output.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        self.output.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.NoTextInteraction)
        self.output.setOverwriteMode(False)
        self.log.connect(self.appendText)
        layout = QtWidgets.QGridLayout(self._main)
        host_layout = QtWidgets.QHBoxLayout()
        host_layout.addWidget(self.host, stretch=3)
        host_layout.addWidget(self.port, stretch=1)
        host_layout.addWidget(self.connect, stretch=1)
        self.ymin_input = QtWidgets.QDoubleSpinBox()
        self.ymin_input.setDecimals(2)
        self.ymin_input.setRange(-1000, 1000)
        self.ymin_input.setValue(-0.5)  # Default Ymin
        self.ymin_input.setSingleStep(0.1)
        self.ymin_input.setPrefix("Ymin: ")

        self.ymax_input = QtWidgets.QDoubleSpinBox()
        self.ymax_input.setDecimals(2)
        self.ymax_input.setRange(-1000, 1000)
        self.ymax_input.setValue(0.5)   # Default Ymax
        self.ymax_input.setSingleStep(0.1)
        self.ymax_input.setPrefix("Ymax: ")

        self.ymin_input.valueChanged.connect(self.update_canvas)
        self.ymax_input.valueChanged.connect(self.update_canvas)
        graph_layout = QtWidgets.QHBoxLayout()
        graph_layout.addWidget(QtWidgets.QLabel("Axes: "))
        for axis in "xyz":
            graph_layout.addWidget(self.axes_sel[axis])
        graph_layout.addWidget(QtWidgets.QLabel("Duration: "))
        graph_layout.addWidget(self.duration)
        graph_layout.addSpacing(20)
        q_desc = QtWidgets.QLabel("Kalman Q: ")
        graph_layout.addWidget(q_desc)
        graph_layout.addWidget(self.q_input)
        graph_layout.addSpacing(20)
        graph_layout.addWidget(self.n_label)
        graph_layout.addWidget(self.n_input)
        graph_layout.addSpacing(20)
        graph_layout.addWidget(self.ymin_input)
        graph_layout.addWidget(self.ymax_input)
        graph_layout.addStretch()
        layout.addLayout(host_layout, 0, 0, 1, 3)
        layout.addLayout(graph_layout, 0, 3)
        layout.addWidget(self.output, 1, 0, 1, 3)
        layout.addWidget(self.autoscroll, 2, 0)
        layout.addWidget(self.logfile, 2, 1)
        layout.addWidget(self.choose, 2, 2)
        layout.addWidget(self.main_button, 3, 0, 1, 3)
        self.std_panel = QtWidgets.QPlainTextEdit()
        self.std_panel.setReadOnly(True)
        font = self.std_panel.font()
        font.setPointSize(14)
        self.std_panel.setFont(font)
        self.std_panel.setMinimumHeight(150)
        layout.addWidget(self.canvas_kalman, 1, 3, 1, 1)
        layout.addWidget(self.canvas_amp, 2, 3, 1, 1)
        layout.addWidget(self.std_panel, 3, 3, 3, 1)
        layout.setColumnStretch(0,5)
        layout.setColumnStretch(1,20)
        layout.setColumnStretch(2,5)
        layout.setColumnStretch(3,60)
        self.clear()
        self.drawing_timer = self.canvas_kalman.new_timer(20)
        self.drawing_timer.add_callback(self.update_canvas)
        self.drawing_timer.start()
        shortcut = QtGui.QKeySequence("Ctrl+W")
        try:
            self.shortcut = QtGui.QShortcut(shortcut, self) # PyQt6
        except:
            self.shortcut = QtWidgets.QShortcut(shortcut, self) # PyQt5
        self.shortcut.activated.connect(self.close)
        self.main_button.setFocus()
        self.showMaximized()
       

    def update_n_campioni(self, value):
        self.n_campioni = value

    def update_q_kalman(self, value):
        self.q_kalman = value

    def clear(self):
        self.logfile.setText("")
        self.host.setDisabled(False)
        self.port.setDisabled(False)
        for axis in "xyz":
            self.axes_sel[axis].setDisabled(False)
        self.duration.setDisabled(False)
        self.choose.setDisabled(False)
        self.main_button.setText("Start")
        self.main_button.setDisabled(True)
        self.output_len = 0
        self.output.clear()
        self.init_canvas()
        self.state = None
        self.CALIBRATION = {}
        self.CALIBRATION_INPUT = {}
        self.TOF = {}
        for axis in self.get_axes():
            for sensor in [0,1]:
                self.CALIBRATION_INPUT[f"dist_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"peak_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"phi_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"T_{axis}{sensor}"] = []
                self.CALIBRATION_INPUT[f"f_{axis}{sensor}"] = np.nan
        self.v_air_history = {axis: [np.nan]*10 for axis in "xyz"}
        self.deltaphi_history = {axis: {"0": [np.nan] * 3, "1": [np.nan] * 3} for axis in "xyz"}
        self.v_sound_history = {axis: [np.nan]*3 for axis in "xyz"}
        self.v_air_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.v_sound_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.phi0_history = {axis: {"0": [np.nan] * 15} for axis in "xyz"}
        self.phi1_history = {axis: {"1": [np.nan] * 15} for axis in "xyz"}
        self.phi00_history = {axis: {"0": [np.nan] * 2} for axis in "xyz"}
        self.phi11_history = {axis: {"1": [np.nan] * 2} for axis in "xyz"}
        self.tof00_history = {axis: {"0": [np.nan] * 2} for axis in "xyz"}
        self.tof11_history = {axis: {"1": [np.nan] * 2} for axis in "xyz"}
        self.count0 = {axis: 0 for axis in "xyz"}
        self.count1 = {axis: 0 for axis in "xyz"}
        self.Autocal = {axis: 0 for axis in "xyz"}
        self.queue = queue.Queue()
        if getattr(self,"mqtt", None):
            self.mqtt.stop()
        self.mqtt = None
        self.connect.setText("Connect")
        # Inizializza la history per ampiezza e fase raw
        self.rho_history = {axis: {"0": [], "1": []} for axis in "xyz"}
        self.fase_history = {axis: {"0": [], "1": []} for axis in "xyz"}

    def get_axes(self):
        return [axis for axis in "xyz" if self.axes_sel[axis].isChecked()]

    def on_click_connect(self):
        if getattr(self,"mqtt", None):
            self.clear()
        else:
            try:
                self.mqtt = MQTTClient(self.host.text(), self.port.value(), "anemometer", self.parse_message)
                self.mqtt.start()
                self.connect.setText("Disconnect")
                self.main_button.setDisabled(False)
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", str(e))
                return

    def on_click(self):
        if not self.state:
            self.mqtt.publish(json.dumps({"state": "calibration"}))
        elif self.state == "calibration":
            self.mqtt.publish(json.dumps({"state": "measure"}))
        elif self.state == "measure":
            self.mqtt.publish(json.dumps({"state": "stopped"}))
        elif self.state == "stopped":
            self.clear()

    def choose_logfile(self):
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save log")
        if filename:
            self.logfile.setText(filename)

    def print(self, *args, **kwargs):
        with io.StringIO() as out:
            print(*args, file=out, **kwargs)
            text = out.getvalue()
            logfile = self.logfile.text()
            if logfile:
                with open(logfile, "a") as f:
                    f.write(text)
            self.log.emit(text)

    def appendText(self, text):
        self.output.insertPlainText(text)
        self.output_len += len(text)
        if self.autoscroll.isChecked():
            cursor = self.output.textCursor()
            cursor.setPosition(self.output_len)
            self.output.setTextCursor(cursor)

    def get_data(self):
        try:
            data = self.queue.get_nowait()
        except queue.Empty:
            data = None
        return data

    def parse_message(self, message):
        if "state" in message:
            if message["state"] == "calibration":
                self.host.setDisabled(True)
                self.port.setDisabled(True)
                for axis in "xyz":
                    self.axes_sel[axis].setDisabled(True)
                self.duration.setDisabled(True)
                self.choose.setDisabled(True)
                self.init_canvas()
                self.state = "calibration"
                self.print(f"[{datetime.now().isoformat(' ')}] Calibration start.")
                self.main_button.setText("End calibration")
            elif message["state"] == "measure":
                self.state = "measure"
                self.main_button.setText("Stop")
                self.CALIBRATION = self.compute_calibration()
                self.print("")
                self.print(f"[{datetime.now().isoformat(' ')}] Calibration end.")
                for axis in self.get_axes():
                    for sensor in [0,1]:
                        for var in ["dist0", "sample0", "phi0", "T0", "tof0"]:
                            key = f"{var}_{axis}{sensor}"
                            val = self.CALIBRATION[key]
                            self.print(f"{key}: {val:.4f} ", end="")
                        key = f"f_{axis}{sensor}"
                        val = self.CALIBRATION[key]
                        self.print(f"{key}: {val}")
                self.print(f"[{datetime.now().isoformat(' ')}] Measure start.")
            elif message["state"] == "stopped":
                self.state = "stopped"
                self.main_button.setText("Clear")
                if getattr(self,"mqtt", None):
                    self.mqtt.stop()
                self.print(f"[{datetime.now().isoformat(' ')}] Measure end.")
            else:
                print(f'Unknown state "{message["state"]}"')
            return
        if not self.state:
            pass
        elif self.state == "calibration":
            self.parse_measure_calibration(message)
        elif self.state == "measure":
            self.parse_measure(message)

    def parse_measure_calibration(self, measure):
        for i, axis in enumerate(self.get_axes()):
            for item in measure.get(axis, []):
                if "ch101" in item:
                    sensor = item["ch101"]
                    rho, phi = polar(item["i"], item["q"])
                    
                    peak = rho.argmax()
                    self.CALIBRATION_INPUT[f"dist_{axis}{sensor}"].append(item["range_mm"])
                    self.CALIBRATION_INPUT[f"peak_{axis}{sensor}"].append(peak)
                    self.CALIBRATION_INPUT[f"phi_{axis}{sensor}"].append(phi)
                    self.CALIBRATION_INPUT[f"f_{axis}{sensor}"] = item["freq"]
                elif "hdc3020" in item:
                    sensor = item["hdc3020"]
                    self.CALIBRATION_INPUT[f"T_{axis}{sensor}"].append(item["temp"])
        self.print(".", end="", flush=True)

    def compute_calibration(self):
        # calibration output - per axis, sensor
        # phi0 - phase avg
        # T0   - temperature avg
        # tof0 - base time of flight
        # f    - frequency
        output = {}
        for axis in self.get_axes():
            for sensor in [0,1]:
                # TODO: use precomputed values for now
                # dist = self.CALIBRATION_INPUT[f"dist_{axis}{sensor}"]
                # if not len(dist):
                #     self.init_canvas()
                #     self.axes_sel[axis].setChecked(False)
                #     continue
                # dist0 = np.mean(dist) * 1000 # distance in micron
                dist0 = DIST[f"{axis}{sensor}"]

            
                peak = self.CALIBRATION_INPUT[f"peak_{axis}{sensor}"]
          
                sample0 = st.mode(peak, keepdims=True).mode[0] - 2
             

                phi_array = np.array([phi[sample0] for phi in self.CALIBRATION_INPUT[f"phi_{axis}{sensor}"]])
                if not len(phi_array):
                    self.init_canvas()
                    self.axes_sel[axis].setChecked(False)
                    continue

                T = self.CALIBRATION_INPUT[f"T_{axis}{sensor}"]
                f = self.CALIBRATION_INPUT[f"f_{axis}{sensor}"]


                mean_phi = np.mean(phi_array)
                std_phi = np.std(phi_array)
                # Definisci i limiti (ad esempio, 2 deviazioni standard dalla media)
                lower_bound = mean_phi - std_phi
                upper_bound = mean_phi + std_phi
                # Filtra gli outlier
                filtered_phi = phi_array[(phi_array >= lower_bound) & (phi_array <= upper_bound)]

                phi0 = np.mean(filtered_phi)

                T0 = np.mean(T)
                v_sound = np.sqrt(1.4 * 287 * (T0 + 273.15))
                try:
                    tof0 = dist0 / v_sound
                except ZeroDivisionError:
                    self.init_canvas()
                    self.axes_sel[axis].setChecked(False)
                    continue
                output[f"dist0_{axis}{sensor}"] = dist0
                output[f"sample0_{axis}{sensor}"] = sample0
                output[f"phi0_{axis}{sensor}"] = phi0
                output[f"T0_{axis}{sensor}"] = T0
                output[f"tof0_{axis}{sensor}"] = tof0
                output[f"f_{axis}{sensor}"] = f
        return output

    def parse_measure(self, measure):
        if not len(self.CALIBRATION):
            # calibration output not yet available, skip
            return
        v_air = {}
        v_sound = {}
        temp_sonica = {}
        v_air_filtered = {}
        v_sound_filtered = {}
        # INIZIALIZZA TUTTE LE CHIAVI PER OGNI ASSE!
        for axis in self.get_axes():
            v_air[axis] = np.nan
            v_sound[axis] = np.nan
            v_air_filtered[axis] = np.nan
            v_sound_filtered[axis] = np.nan
            temp_sonica[axis] = np.nan    
        for i, axis in enumerate(self.get_axes()):
            
            g = 0
            s = 0 
            f0 = 0
            f1 = 0
            tof0_0 = 0
            tof0_1 = 0
            for item in measure.get(axis, []):
                if "hdc3020" in item:
                    sensor = item["hdc3020"]
                    # Sensore 0
                    if g == 0:
                        g = g + 1
                        dist0 = DIST[f"{axis}{sensor}"]
                        T0_now = item["temp"]
                        v_sound_cal = np.sqrt(1.4 * 287 * (T0_now + 273.15))
                        tof0_0 = dist0 / v_sound_cal
                        
                    # Sensore 1
                    else:
                        dist1 = DIST[f"{axis}{sensor}"]
                        T1_now = item["temp"]
                        Tmean = (T1_now + T0_now)/2
                        v_sound_cal = np.sqrt(1.4 * 287 * (Tmean+ 273.15))
                        tof0_1 = dist1 / v_sound_cal
                        tof0_0 = tof0_1

                elif "ch101" in item:
                    sensor = item["ch101"]

                    rho, phi = polar(item["i"], item["q"])

                    sample_measure = rho.argmax()-2


                    sample0 = self.CALIBRATION[f"sample0_{axis}{sensor}"]
                    tof0 = self.CALIBRATION[f"tof0_{axis}{sensor}"]
                    phi0 = self.CALIBRATION[f"phi0_{axis}{sensor}"]
                    f = self.CALIBRATION[f"f_{axis}{sensor}"]
    
                    amp_max = rho[sample0]
                    # print(amp_max, sample0)
                    fase_max = phi[sample0]
                    self.rho_history[axis][str(sensor)].append(amp_max)
                    self.fase_history[axis][str(sensor)].append(fase_max)
                    self.rho_history[axis][str(sensor)] = self.rho_history[axis][str(sensor)][-self.n_campioni:]
                    self.fase_history[axis][str(sensor)] = self.fase_history[axis][str(sensor)][-self.n_campioni:]
                    # print(self.rho_history[axis][str(sensor)], len(self.rho_history[axis][str(sensor)]))
                    # print('la')

                    # fase_hist = self.fase_history[axis][str(sensor)][-self.n_campioni:]
                    # fase_hist = [v for v in fase_hist if not np.isnan(v)]
                    # std_fase = np.std(fase_hist) if fase_hist else float('nan')
                    # std_fase = std_fase * 180 / np.pi

                    rho_hist = self.rho_history[axis][str(sensor)][-self.n_campioni:]
                    rho_hist = [v for v in rho_hist if not np.isnan(v)]
                    std_rho = np.std(rho_hist) if rho_hist else float('nan')
                    

                    #Sensore 0: primo ciclo in cui si hanno i dati del sensore 0 inerenti alla misura attuale, vengono registrati
                    if s == 0:
                        #Unwrap fase
                        if abs(phi[sample0] - self.phi0_history[axis]["0"][-1]) >= np.pi:
                            phi[sample0] = phi[sample0] - np.sign(phi[sample0])*2*np.pi
                        # Registrazione fase solo iniziale
                        if np.isnan(self.phi00_history[axis][f"{sensor}"][-1]):
                            self.phi00_history[axis][f"{sensor}"].append(phi[sample0])
                            self.phi00_history[axis][f"{sensor}"] = self.phi00_history[axis][f"{sensor}"][1:]
                        # Registrazione tof solo iniziale
                        if np.isnan(self.tof00_history[axis][f"{sensor}"][-1]):
                            self.tof00_history[axis][f"{sensor}"].append(tof0_0)
                            self.tof00_history[axis][f"{sensor}"] = self.tof00_history[axis][f"{sensor}"][1:]
                        f0 = f
                        s = s + 1
                        if sample_measure==sample0:                          
                            delta_fase0 = phi[sample0] - self.phi00_history[axis][f"{sensor}"][-1]
                            self.deltaphi_history[axis][f"{sensor}"].append(delta_fase0)
                            self.deltaphi_history[axis][f"{sensor}"] = self.deltaphi_history[axis][f"{sensor}"][1:]

                            self.phi0_history[axis]["0"].append(phi[sample0])
                            self.phi0_history[axis]["0"] = self.phi0_history[axis]["0"][1:]

                            if std_rho < 40:
                                self.count0[axis] +=1

                                if self.count0[axis] == 15:
                                    self.Autocal[axis] = 1
                            else:
                                    self.count0[axis] = 0
                        else:
                            delta_fase0 = self.deltaphi_history[axis][f"{sensor}"][-1]
                            self.deltaphi_history[axis][f"{sensor}"].append(delta_fase0)
                            self.deltaphi_history[axis][f"{sensor}"] = self.deltaphi_history[axis][f"{sensor}"][1:]



                    #Sensore 1: acquisizione dei dati sul sensore 1 relativi alla misura attuale
                    else:
                        if abs(phi[sample0] - self.phi1_history[axis]["1"][-1]) >= np.pi:
                            phi[sample0] = phi[sample0] - np.sign(phi[sample0])*2*np.pi

                        if np.isnan(self.phi11_history[axis][f"{sensor}"][-1]):
                            self.phi11_history[axis][f"{sensor}"].append(phi[sample0])
                            self.phi11_history[axis][f"{sensor}"] = self.phi11_history[axis][f"{sensor}"][1:]
                            
                        if np.isnan(self.tof11_history[axis][f"{sensor}"][-1]):
                            self.tof11_history[axis][f"{sensor}"].append(tof0_1)
                            self.tof11_history[axis][f"{sensor}"] = self.tof11_history[axis][f"{sensor}"][1:]
                        f1 = f
                        if sample_measure==sample0:
                            delta_fase1 = phi[sample0] - self.phi11_history[axis][f"{sensor}"][-1]
                            self.deltaphi_history[axis][f"{sensor}"].append(delta_fase1)
                            self.deltaphi_history[axis][f"{sensor}"] = self.deltaphi_history[axis][f"{sensor}"][1:]

                            self.phi1_history[axis]["1"].append(phi[sample0])
                            self.phi1_history[axis]["1"] = self.phi1_history[axis]["1"][1:]

                            if std_rho < 40:
                                self.count1[axis] +=1
                                if self.count1[axis] == 15:
                                    self.Autocal[axis] = 1
                            else:
                                    self.count1[axis] = 0
                        else:
                            delta_fase1 = self.deltaphi_history[axis][f"{sensor}"][-1]
                            self.deltaphi_history[axis][f"{sensor}"].append(delta_fase1)
                            self.deltaphi_history[axis][f"{sensor}"] = self.deltaphi_history[axis][f"{sensor}"][1:]


                        if self.Autocal[axis] == 1:
                            self.Autocal[axis] = 0
                            # min e max e prendere media DA AGGIUNGERE
                            phi00_mean = np.mean(self.phi0_history[axis]["0"][-10:])
                            phi11_mean = np.mean(self.phi1_history[axis]["1"][-10:])


                            self.phi00_history[axis]["0"].append(phi00_mean)
                            self.phi00_history[axis]["0"] = self.phi00_history[axis]["0"][1:]
                            self.tof00_history[axis]["0"].append(tof0_0)
                            self.tof00_history[axis]["0"] = self.tof00_history[axis]["0"][1:] 

                            self.phi11_history[axis]["1"].append(phi11_mean)
                            self.phi11_history[axis]["1"] = self.phi11_history[axis]["1"][1:]
                            self.tof11_history[axis]["1"].append(tof0_1)
                            self.tof11_history[axis]["1"] = self.tof11_history[axis]["1"][1:] 
                            self.count0[axis] = 0
                            self.count1[axis] = 0

                           # Valutare
                            delta_fase0 = 0
                            delta_fase1 = 0 


                        if abs(delta_fase0) > np.pi:
                                delta_fase0 = -1 * np.sign(delta_fase0) * (2 * np.pi - abs(delta_fase0))
                        if abs(delta_fase1) > np.pi:
                                delta_fase1 = -1 * np.sign(delta_fase1) * (2 * np.pi - abs(delta_fase1))

                        # # Filtro Mario
                        if delta_fase0 * delta_fase1 > 0:  # concordi
                            alpha = min(abs(delta_fase0), abs(delta_fase1))
                            segno = 1 if delta_fase0 > 0 else -1
                            alpha *= segno
                            delta_fase0 -= alpha
                            delta_fase1 -= alpha

                        if delta_fase0 * delta_fase1 < 0:  # discordi
                            delta_phi_avg = (delta_fase0 - delta_fase1) / 2
                            delta_fase0 = delta_phi_avg
                            delta_fase1 = -delta_phi_avg
  

            tof00 = self.tof00_history[axis]["0"][-1] -  (1000000 * delta_fase0) / (2 * np.pi * f0)
            self.TOF[f"{axis}0"] = tof00

            tof11 = self.tof11_history[axis]["1"][-1] -  (1000000 * delta_fase1) / (2 * np.pi * f1)
            self.TOF[f"{axis}1"] = tof11

            # Calcola la velocità del flusso d'aria se entrambi i TOF sono disponibili
            if f"{axis}0" in self.TOF and f"{axis}1" in self.TOF:
                dist = [self.CALIBRATION[f"dist0_{axis}{sensor}"] for sensor in [0,1]]
                v_air[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] - dist[1] / self.TOF[f"{axis}1"])
                v_sound[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] + dist[1] / self.TOF[f"{axis}1"])

            # Aggiorna lo storico dei valori precedenti
            self.v_air_history[axis].append(v_air[axis])
            self.v_air_history[axis] = self.v_air_history[axis][1:]
   
            #Aggiorna il filtro di Kalman
            Q = self.q_kalman
            v_air_filtered[axis] = self.v_air_filter[axis].update(v_air[axis], Q)
            v_sound_filtered[axis] = self.v_sound_filter[axis].update(v_sound[axis], Q)

            # Per la temperatura usiamo il valore di velocità uscente dal filtro di Kalman
            temp_sonica[axis]=v_sound_filtered[axis]*v_sound_filtered[axis]/(1.4*287)-273.15


        if len(v_air) == len(self.get_axes()):
            v_air['timestamp'] = measure["timestamp_end"]
            for i, axis in enumerate(self.get_axes()):
                v_air[f'{axis}_kalman'] = v_air_filtered.get(axis, np.nan)
            self.print(f'{v_air["timestamp"]:.4f} ', end="")
            for i, axis in enumerate(self.get_axes()):
                self.print(f" {v_air_filtered.get(axis, np.nan):+6.4f}", end="")
            for i, axis in enumerate(self.get_axes()):
                self.print(f" {temp_sonica[axis]:+3.2f}", end="")
            self.print("")
            self.queue.put(v_air)

    def init_canvas(self):
        self.fig_kalman = self.canvas_kalman.figure
        self.fig_amp = self.canvas_amp.figure
        self.fig_kalman.clf()
        self.fig_kalman.set_tight_layout(True)
        self.fig_amp.clf()
        self.fig_amp.set_tight_layout(True)
        self.lines = {}
        # Primo grafico: Kalman
        self.ax_kalman = self.fig_kalman.add_subplot(111)
        self.fig_kalman.suptitle("Filtro Kalman + Ampiezza", color="red")
        # Secondo grafico: Solo Ampiezza
        self.ax_amp = self.fig_amp.add_subplot(111)
        self.fig_amp.suptitle("Filtro Ampiezza", color="red")
        t, x = [datetime.now()], [np.nan]
        fmt = dates.DateFormatter('%H:%M:%S')
        self.ax_kalman.xaxis.set_major_formatter(fmt)
        self.ax_amp.xaxis.set_major_formatter(fmt)
        self.ax_kalman.set_ylim(-1, 1)
        self.ax_amp.set_ylim(-1, 1)
        colors_kalman = {'x': 'C6', 'y': 'C7', 'z': 'C8'}
        colors_amp = {'x': 'C3', 'y': 'C4', 'z': 'C5'}
        for i, axis in enumerate(self.get_axes()):
            self.lines[f"{axis}.kalman"], = self.ax_kalman.plot(t, x, '-', linewidth=1.2, label=f"v_{axis} Filtro Kalman + Filtro Ampiezza", color=colors_kalman[axis])
            self.lines[f"{axis}.amp"], = self.ax_amp.plot(t, x, '-', linewidth=1.2, label=f"v_{axis} Filtro Ampiezza", color=colors_amp[axis])
        self.ax_kalman.legend(loc='upper left')
        self.ax_amp.legend(loc='upper left')
        self.fig_kalman.canvas.draw_idle()
        self.fig_amp.canvas.draw_idle()
        self.ax_kalman.legend(loc='upper left')
        self.ax_amp.legend(loc='upper left')
        self.fig_kalman.canvas.draw_idle()
        self.fig_amp.canvas.draw_idle()


    def update_canvas(self):
        data = []
        while True:
            item = self.get_data()
            if not item:
                break
            data.append(item)
        if not data:
            return
        y_lim_kalman = [[],[]]
        y_lim_amp = [[],[]]
            # Calcolo deviazione standard fase e ampiezza
        std_text = "Deviazione standard (ultimi {} campioni):\n".format(self.n_campioni)
        for axis in self.get_axes():
            for sensor in ["0", "1"]:
                # n campioni di fase 
                fase_hist = self.fase_history[axis][sensor][-self.n_campioni:]
                fase_hist = [v for v in fase_hist if not np.isnan(v)]
                std_fase = np.std(fase_hist) if fase_hist else float('nan')
                #n campioni di ampiezza 
                rho_hist = self.rho_history[axis][sensor][-self.n_campioni:]
                rho_hist = [v for v in rho_hist if not np.isnan(v)]
                std_rho = np.std(rho_hist) if rho_hist else float('nan')
                std_fase = std_fase*180/np.pi
                std_text += f"Asse {axis} Sensore {sensor}: Fase={std_fase:.4f}  Ampiezza={std_rho:.4f}\n"
        self.std_panel.setPlainText(std_text)
        for i, axis in enumerate(self.get_axes()):
            t_kalman, _ = self.lines[f"{axis}.kalman"].get_data(True)
            t_amp, _ = self.lines[f"{axis}.amp"].get_data(True)
            y_kalman = self.lines[f"{axis}.kalman"].get_ydata(True)
            y_amp = self.lines[f"{axis}.amp"].get_ydata(True)
            for v_air in data:
                t_kalman = np.hstack((t_kalman, datetime.fromtimestamp(v_air["timestamp"])));
                y_kalman = np.hstack((y_kalman, v_air.get(f'{axis}_kalman', np.nan)))
                t_amp = np.hstack((t_amp, datetime.fromtimestamp(v_air["timestamp"])));
                y_amp = np.hstack((y_amp, v_air.get(axis, np.nan)))  # Solo ampiezza
            t_min = datetime.fromtimestamp(t_kalman[-1].timestamp() - self.duration.value())
            mask_kalman = t_kalman >= t_min
            mask_amp = t_amp >= t_min
            t_kalman = t_kalman[mask_kalman]
            y_kalman = y_kalman[mask_kalman]
            t_amp = t_amp[mask_amp]
            y_amp = y_amp[mask_amp]
            y_lim_kalman[0].append(np.nanmin(y_kalman))
            y_lim_kalman[1].append(np.nanmax(y_kalman))
            y_lim_amp[0].append(np.nanmin(y_amp))
            y_lim_amp[1].append(np.nanmax(y_amp))
            self.lines[f"{axis}.kalman"].set_data(t_kalman, y_kalman)
            self.lines[f"{axis}.amp"].set_data(t_amp, y_amp)
        self.ax_kalman.set_xlim(t_min, t_kalman[-1])
        self.ax_amp.set_xlim(t_min, t_amp[-1])
        self.ax_kalman.axhline(0, color='black', linestyle='-', linewidth=0.5)
        self.ax_amp.axhline(0, color='black', linestyle='-', linewidth=0.5)
        y_min = self.ymin_input.value()
        y_max = self.ymax_input.value()
        self.ax_kalman.set_ylim(y_min-0.02*abs(y_min), y_max+0.02*abs(y_max))
        self.ax_amp.set_ylim(y_min-0.02*abs(y_min), y_max+0.02*abs(y_max))
        self.ax_kalman.set_title("Filtro Kalman + Ampiezza")
        self.ax_amp.set_title("Filtro Ampiezza")
        self.fig_kalman.canvas.draw_idle()
        self.fig_amp.canvas.draw_idle()

if __name__ == "__main__":
    qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow()
    app.show()
    app.activateWindow()
    app.raise_()
    qapp.exec()
    app.raise_()
    qapp.exec()
