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
            print(f"ERROR: {e}")

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
        # print(Q)
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
            # print(self.x_est)
        return self.x_est


class ApplicationWindow(QtWidgets.QMainWindow):
    log = QtCore.Signal(str)
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DIET Sapienza - 3D Anemometer")
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        canvas = FigureCanvas(Figure(figsize=(15, 15)))
        self.host = QtWidgets.QLineEdit()
        self.host.setPlaceholderText("Anemometer name or IP")
        #self.host.setText("foxd27.local")
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
        self.q_input.setValue(0.01)  # Default Kalman Q
        self.q_input.setSingleStep(0.0001)
        self.q_kalman = self.q_input.value()
        self.q_input.valueChanged.connect(self.update_q_kalman)
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
        layout.addWidget(canvas, 1, 3, 2, 1)
        layout.addWidget(NavigationToolbar(canvas, self), 3, 3)
        layout.setColumnStretch(0,5)
        layout.setColumnStretch(1,20)
        layout.setColumnStretch(2,5)
        layout.setColumnStretch(3,60)
        self.fig = canvas.figure
        self.clear()
        self.drawing_timer = canvas.new_timer(20)
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
        self.phi_history = {axis: {"0": [np.nan] * 10, "1": [np.nan] * 10} for axis in "xyz"}
        self.peakMax_history = {axis: {"0": [np.nan] * 3, "1": [np.nan] * 3} for axis in "xyz"}
        self.phi_history = {axis: {"0": [np.nan] * 3, "1": [np.nan] * 3} for axis in "xyz"}
        self.delta_fase_history = {axis: {"0": [np.nan] * 3, "1": [np.nan] * 3} for axis in "xyz"}
        self.v_sound_history = {axis: [np.nan]*3 for axis in "xyz"}
        self.v_air_filter = { axis: KalmanFilter() for axis in "xyz" }
        self.v_air_kalman_only_history = {axis: [np.nan]*10 for axis in "xyz"}
        self.v_air_kalman_only_filter = {axis: KalmanFilter() for axis in "xyz"}
        self.queue = queue.Queue()
        if getattr(self,"mqtt", None):
            self.mqtt.stop()
        self.mqtt = None
        self.connect.setText("Connect")

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
        delta_fase_pre = {}
        temp_sonica = {}
        v_air_filtered = {}
        v_air_kalman_only = {}
        v_air_nofilt = {}

        # INIZIALIZZA TUTTE LE CHIAVI PER OGNI ASSE!
        for axis in self.get_axes():
            v_air[axis] = np.nan
            v_sound[axis] = np.nan
            v_air_filtered[axis] = np.nan
            v_air_kalman_only[axis] = np.nan
            v_air_nofilt[axis] = np.nan
            temp_sonica[axis] = np.nan
        
        for i, axis in enumerate(self.get_axes()):
            delta_fase_pre[axis] = {}

            n = measure["counter"]
            m = 0  
            f0 = 0
            f1 = 0 
            for item in measure.get(axis, []):
                if "ch101" in item:
                    sensor = item["ch101"]

                    if f"{sensor}" not in delta_fase_pre[axis]:
                        delta_fase_pre[axis][f"{sensor}"] = 0.0

                    rho, phi = polar(item["i"], item["q"])
                    
                    sample_measure = rho.argmax()-2
                    
                

                    dist0 = self.CALIBRATION[f"dist0_{axis}{sensor}"]
                    sample0 = self.CALIBRATION[f"sample0_{axis}{sensor}"]
                    tof0 = self.CALIBRATION[f"tof0_{axis}{sensor}"]
                    phi0 = self.CALIBRATION[f"phi0_{axis}{sensor}"]
                    #print("sample_measure:",sample_measure,'axis:',axis,'sensor:',sensor,'sample0_',sample0)

                    f = self.CALIBRATION[f"f_{axis}{sensor}"]

                    if m == 0:
                        delta_fase0_nofilt = phi[sample0] - phi0

                        #Filtro di unwrap della fase 
                        peak0 = rho.argmax()
                        if abs(phi[sample0] - self.phi_history[axis]["0"][-1]) >= np.pi and abs(peak0 - self.peakMax_history[axis]["0"][-1]) < 2:
                            phi[sample0] = phi[sample0] - np.sign(phi[sample0])*2*np.pi
                        self.phi_history[axis]["0"].append(phi[sample0])
                        self.phi_history[axis]["0"] = self.phi_history[axis]["0"][1:]
                        
                        delta_fase0 = phi[sample0] - phi0
                        self.delta_fase_history[axis][f"{sensor}"].append(delta_fase0)
                        self.delta_fase_history[axis][f"{sensor}"] = self.delta_fase_history[axis][f"{sensor}"][1:]
                        self.peakMax_history[axis][f"{sensor}"].append(peak0)
                        self.peakMax_history[axis][f"{sensor}"] = self.peakMax_history[axis][f"{sensor}"][1:]
                        f0 = f
                        m = m + 1
                    #Sensore 0 e 1: secondo ciclo in cui si hanno i dati del sensore 1 inerenti alla misurazione attuale e i dati registrati del sensore 0
                    #registrati nel ciclo precedente, ma sempre riguardanti la misurazione corrente
                    else:
                        delta_fase1_nofilt = phi[sample0] - phi0

                        f1 = f
                        peak1 = rho.argmax()
                        #Filtro di unwrap della fase 
                        if abs(phi[sample0] - self.phi_history[axis]["1"][-1]) >= np.pi and abs(peak1 - self.peakMax_history[axis]["1"][-1]) < 2:
                            phi[sample0] = phi[sample0] - np.sign(phi[sample0])*2*np.pi
                        #Prima parte dell'if: inizializza l'if nel momento in cui i valori del delta di fase del sensore 1 cominciano ad essere acquisiti
                        #Seconda parte dell'if: Filtro 1
                        # print('Verifica',abs(abs(self.delta_fase_history[axis]["1"][-1]) - abs(self.delta_fase_history[axis]["0"][-2]))*180/np.pi, 'picco1Att:',peak1, 'picco1Pre:',self.peakMax_history[axis]["1"][-1])
                        if all(not np.isnan(v) for v in self.delta_fase_history[axis]["1"]) and abs(abs(self.delta_fase_history[axis]["1"][-1]) - abs(self.delta_fase_history[axis]["0"][-2])) >= np.pi/10:
                            #if del Filtro 2
                            # print('filtro')
                            if abs(peak1 - self.peakMax_history[axis]["1"][-1]) > 2 or abs(self.peakMax_history[axis]["0"][-1] - self.peakMax_history[axis]["0"][-2]) > 2:
                                #Filtro 1
                                self.delta_fase_history[axis]["0"][-2] = self.delta_fase_history[axis]["0"][-3]
                                delta_fase0 = self.delta_fase_history[axis]["0"][-2]
                                self.delta_fase_history[axis]["1"][-1] = self.delta_fase_history[axis]["1"][-2]
                                delta_fase1 = self.delta_fase_history[axis]["1"][-1]
                                self.delta_fase_history[axis]["1"].append(delta_fase1)
                                self.delta_fase_history[axis]["1"] = self.delta_fase_history[axis][f"{sensor}"][1:]
                                #Filtro 2
                                self.peakMax_history[axis]["0"][-1] = self.peakMax_history[axis]["0"][-2]
                                peak1 = self.peakMax_history[axis]["1"][-1]
                                self.peakMax_history[axis]["1"].append(peak1)
                                self.peakMax_history[axis]["1"] = self.peakMax_history[axis]["1"][1:]
                                #Registro fase precedente perché se un sensore sballa, tutta la misura va scartata
                                self.phi_history[axis]["0"][-1] = self.phi_history[axis]["0"][-2]
                                phi[sample0] = self.phi_history[axis]["1"][-1]
                                self.phi_history[axis]["1"].append(phi[sample0])
                                self.phi_history[axis]["1"] = self.phi_history[axis]["1"][1:]

                            else:
                                #Filtro 1 
                                self.delta_fase_history[axis]["0"][-2] = (self.delta_fase_history[axis]["0"][-1] + self.delta_fase_history[axis]["0"][-3])/2
                                delta_fase0 = self.delta_fase_history[axis]["0"][-2]
                                self.delta_fase_history[axis]["1"][-1] = (phi[sample0]-phi0 + self.delta_fase_history[axis]["1"][-2])/2
                                delta_fase1 = self.delta_fase_history[axis]["1"][-1]
                                self.delta_fase_history[axis]["1"].append(delta_fase1)
                                self.delta_fase_history[axis]["1"] = self.delta_fase_history[axis][f"{sensor}"][1:]   
                                #Se non si applica il Filtro 2
                                self.peakMax_history[axis]["1"].append(peak1)
                                self.peakMax_history[axis]["1"] = self.peakMax_history[axis]["1"][1:]
                                #Registro fase
                                self.phi_history[axis]["1"].append(phi[sample0])
                                self.phi_history[axis]["1"] = self.phi_history[axis]["1"][1:]
                        else:
                            #Se non si applica il Filtro 1
                            delta_fase1 = phi[sample0] - phi0
                            self.delta_fase_history[axis]["1"].append(delta_fase1)
                            self.delta_fase_history[axis]["1"] = self.delta_fase_history[axis][f"{sensor}"][1:]
                            self.peakMax_history[axis]["1"].append(peak1)
                            self.peakMax_history[axis]["1"] = self.peakMax_history[axis]["1"][1:]
                            self.phi_history[axis]["1"].append(phi[sample0])
                            self.phi_history[axis]["1"] = self.phi_history[axis]["1"][1:]
                            #I delta di fase che vengono utilizzati si riferiscono alla misura precedente e non a quella attuale
                            delta_fase0 = self.delta_fase_history[axis]["0"][-2]
                            delta_fase1 = self.delta_fase_history[axis]["1"][-2]
                        
                        if abs(delta_fase0) > np.pi:
                                delta_fase0 = -1 * np.sign(delta_fase0) * (2 * np.pi - abs(delta_fase0))
                        if abs(delta_fase1) > np.pi:
                                delta_fase1 = -1 * np.sign(delta_fase1) * (2 * np.pi - abs(delta_fase1))
                        # # print('misura')
                        # print('Deltafase0:',delta_fase0*180/np.pi, 'Deltafase1:',delta_fase1*180/np.pi, 'picco0:',self.peakMax_history[axis]["0"][-1], 'picco1:',peak1)
                        tof00 = tof0 -  (1000000 * delta_fase0) / (2 * np.pi * f0)
                        self.TOF[f"{axis}0"] = tof00
                        tof11 = tof0 -  (1000000 * delta_fase1) / (2 * np.pi * f1)
                        self.TOF[f"{axis}1"] = tof11

                        if abs(delta_fase0_nofilt) > np.pi:
                            delta_fase0_nofilt = -1 * np.sign(delta_fase0_nofilt) * (2 * np.pi - abs(delta_fase0_nofilt))
                        if abs(delta_fase1_nofilt) > np.pi:
                            delta_fase1_nofilt = -1 * np.sign(delta_fase1_nofilt) * (2 * np.pi - abs(delta_fase1_nofilt))
                        # Calcolo TOF senza filtri
                        tof00_nofilt = tof0 - (1000000 * delta_fase0_nofilt) / (2 * np.pi * f0)
                        self.TOF[f"{axis}0_nofilt"] = tof00_nofilt
                        tof11_nofilt = tof0 - (1000000 * delta_fase1_nofilt) / (2 * np.pi * f1)
                        self.TOF[f"{axis}1_nofilt"] = tof11_nofilt

              
            # print('tof00:',tof00, 'tof11:',tof11,)

            if f"{axis}0" in self.TOF and f"{axis}1" in self.TOF:
                dist = [self.CALIBRATION[f"dist0_{axis}{sensor}"] for sensor in [0,1]]
                v_air[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] - dist[1] / self.TOF[f"{axis}1"])
                v_sound[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] + dist[1] / self.TOF[f"{axis}1"])

            if f"{axis}0_nofilt" in self.TOF and f"{axis}1_nofilt" in self.TOF:
                dist0_0 = self.CALIBRATION[f"dist0_{axis}0"]
                dist0_1 = self.CALIBRATION[f"dist0_{axis}1"]
                v_air_nofilt[axis] = 0.5 * (dist0_0 / self.TOF[f"{axis}0_nofilt"] - dist0_1 / self.TOF[f"{axis}1_nofilt"])
                # v_sound_nofilt = 0.5 * (dist0_0 / self.TOF[f"{axis}0_nofilt"] + dist0_1 / self.TOF[f"{axis}1_nofilt"])
   
            # Aggiorna lo storico dei valori precedenti
            media_vel_pre=np.mean(self.v_air_history[axis])
            self.v_air_history[axis].append(v_air[axis])
            self.v_air_history[axis] = self.v_air_history[axis][1:]
        
            self.v_air_kalman_only_history[axis].append(v_air_kalman_only[axis])
            self.v_air_kalman_only_history[axis] = self.v_air_kalman_only_history[axis][1:]
            # print('mean',np.mean(self.v_air_history[axis]))
            # print('std',np.std(self.v_air_history[axis]))

            # Aggiorna il filtro di Kalman
            Q = self.q_kalman
            v_air_filtered[axis] = self.v_air_filter[axis].update(v_air[axis], Q)
            v_air_kalman_only[axis] = self.v_air_kalman_only_filter[axis].update(v_air_nofilt[axis], Q)

            # print('v_air_filtered:',v_air_filtered[axis])
            #if (abs(media_vel_pre)<0.10 and abs(v_air[axis]>0.1)):
            #        v_air[axis]=self.phi_history[axis][f"{sensor}"][-2]
            #        self.phi_history[axis][f"{sensor}"][-1]=v_air[axis]
            #v_air[axis]=np.nanmean(self.v_air_history[axis])

            ##faccio stampare la velocità dell'istante prima
            #v_air[axis]=self.phi_history[axis][f"{sensor}"][-2]

            self.v_sound_history[axis].append(v_sound[axis])
            self.v_sound_history[axis] = self.v_sound_history[axis][1:]
            #v_sound[axis]=np.mean(self.v_sound_history[axis])

            temp_sonica[axis]=v_sound[axis]*v_sound[axis]/(1.4*287)-273.15

            # Stampa i nuovi valori dopo l'aggiornamento
            #print(f"Nuovi valori per {axis}: v_air_pre3 = {self.v_air_pre3[axis]}, v_air_pre2 = {self.v_air_pre2[axis]}, v_air_pre1 = {self.v_air_pre1[axis]}")

        if len(v_air) == len(self.get_axes()):
            v_air['timestamp'] = measure["timestamp_end"]
            for i, axis in enumerate(self.get_axes()):
                v_air[f'{axis}_filtered'] = v_air_filtered.get(axis, np.nan)
                v_air[f'{axis}_kalman_only'] = v_air_kalman_only.get(axis, np.nan)
            self.print(f'{v_air["timestamp"]:.4f} ', end="")
            #for i, axis in enumerate(self.get_axes()):
            #   self.print(f" {v_air[axis]:+6.4f}", end="")
            for i, axis in enumerate(self.get_axes()):
                self.print(f" {v_air_filtered.get(axis, np.nan):+6.4f}", end="")
            for i, axis in enumerate(self.get_axes()):
                self.print(f" {temp_sonica[axis]:+3.2f}", end="")
            self.print("")
            self.queue.put(v_air)

    def init_canvas(self):
        self.fig.clf()
        self.fig.set_tight_layout(True)
        self.lines = {}
        self.ax_kalman = self.fig.add_subplot(311)
        self.ax_raw = self.fig.add_subplot(312, sharex=self.ax_kalman)
        self.ax_kalman_only = self.fig.add_subplot(313, sharex=self.ax_kalman)
        self.fig.suptitle("Velocità dell'aria", color="red")
        t, x = [datetime.now()], [np.nan]
        fmt = dates.DateFormatter('%H:%M:%S')
        self.ax_kalman.xaxis.set_major_formatter(fmt)
        self.ax_raw.xaxis.set_major_formatter(fmt)
        self.ax_kalman_only.xaxis.set_major_formatter(fmt)
        self.ax_kalman.set_ylim(-1, 1)
        self.ax_raw.set_ylim(-1, 1)
        self.ax_kalman_only.set_ylim(-1, 1)
        colors = {'x': 'C0', 'y': 'C1', 'z': 'C2'}
        colors_filtered = {'x': 'C3', 'y': 'C4', 'z': 'C5'}
        colors_kalman_only = {'x': 'C6', 'y': 'C7', 'z': 'C8'}
        for i, axis in enumerate(self.get_axes()):
            self.lines[f"{axis}.filtered"], = self.ax_kalman.plot(t, x, '-', linewidth=1.2, label=f"v_{axis} Kalman + Filtri", color=colors_filtered[axis])
            self.lines[f"{axis}.raw"], = self.ax_raw.plot(t, x, '-', linewidth=1.2, label=f"v_{axis} Solo Filtri", color=colors[axis])
            self.lines[f"{axis}.kalman_only"], = self.ax_kalman_only.plot(t, x, '-', linewidth=1.2, label=f"v_{axis} Solo Kalman", color=colors_kalman_only[axis])
        self.ax_kalman.legend(loc='upper left')
        self.ax_raw.legend(loc='upper left')
        self.ax_kalman_only.legend(loc='upper left')
        self.fig.canvas.draw_idle()


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
        y_lim_raw = [[],[]]
        for i, axis in enumerate(self.get_axes()):
            t, _ = self.lines[f"{axis}.filtered"].get_data(True)
            t_raw, _ = self.lines[f"{axis}.raw"].get_data(True)
            t_kalman_only, _ = self.lines[f"{axis}.kalman_only"].get_data(True)
            y1 = self.lines[f"{axis}.filtered"].get_ydata(True)
            y_raw = self.lines[f"{axis}.raw"].get_ydata(True)
            y_kalman_only = self.lines[f"{axis}.kalman_only"].get_ydata(True)
            for v_air in data:
                t = np.hstack((t, datetime.fromtimestamp(v_air["timestamp"])))
                y_raw = np.hstack((y_raw, v_air[axis]))
                y1 = np.hstack((y1, v_air[f'{axis}_filtered']))
                y_kalman_only = np.hstack((y_kalman_only, v_air.get(f'{axis}_kalman_only', np.nan)))
            t_min = datetime.fromtimestamp(t[-1].timestamp() - self.duration.value())
            mask = t >= t_min
            t = t[mask]
            y1 = y1[mask]
            y_raw = y_raw[mask]
            y_kalman_only = y_kalman_only[mask]
            y_lim_kalman[0].append(np.nanmin(y1))
            y_lim_kalman[1].append(np.nanmax(y1))
            y_lim_raw[0].append(np.nanmin(y_raw))
            y_lim_raw[1].append(np.nanmax(y_raw))
            self.lines[f"{axis}.filtered"].set_data(t, y1)
            self.lines[f"{axis}.raw"].set_data(t, y_raw)
            self.lines[f"{axis}.kalman_only"].set_data(t, y_kalman_only)
        self.ax_kalman.set_xlim(t_min, t[-1])
        self.ax_raw.set_xlim(t_min, t[-1])
        self.ax_kalman_only.set_xlim(t_min, t[-1])
        self.ax_kalman.axhline(0, color='black', linestyle='-', linewidth=0.5)
        self.ax_raw.axhline(0, color='black', linestyle='-', linewidth=0.5)
        self.ax_kalman_only.axhline(0, color='black', linestyle='-', linewidth=0.5)
        y_min = self.ymin_input.value()
        y_max = self.ymax_input.value()
        self.ax_kalman.set_ylim(y_min-0.02*abs(y_min), y_max+0.02*abs(y_max))
        self.ax_raw.set_ylim(y_min-0.02*abs(y_min), y_max+0.02*abs(y_max))
        self.ax_kalman_only.set_ylim(y_min-0.02*abs(y_min), y_max+0.02*abs(y_max))
        self.ax_kalman.set_title("Filtri + Kalman")
        self.ax_raw.set_title("Solo Filtri")
        self.ax_kalman_only.set_title("Solo Kalman")
        self.fig.canvas.draw_idle()

if __name__ == "__main__":
    qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow()
    app.show()
    app.activateWindow()
    app.raise_()
    qapp.exec()

