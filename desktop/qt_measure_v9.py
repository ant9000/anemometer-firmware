#!/usr/bin/env python3

import os, sys, getopt, time, io, queue
from datetime import datetime
import json, threading
import paho.mqtt.client as mqtt

import numpy as np

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

class MQTTFetcher:
    def __init__(self, host, port, channel, parse_message):
        self.client = mqtt.Client()
        self.client.message_callback_add(channel, self.on_message)
        self.client.connect(host, port)
        self.client.subscribe(channel)
        self.parse_message = parse_message
    def start(self):
        self._t = threading.Thread(target=self.client.loop_forever, daemon=True)
        self._t.start()
    def stop(self):
        self.client.disconnect()
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

class ApplicationWindow(QtWidgets.QMainWindow):
    log = QtCore.Signal(str)
    def __init__(self):
        super().__init__()
        self.v_suono_pre1 = {axis: 0.0 for axis in "xyz"}
        self.v_suono_pre2 = {axis: 0.0 for axis in "xyz"}
        self.v_suono_pre3 = {axis: 0.0 for axis in "xyz"}
        self.v_air_pre1 = {axis: 0.0 for axis in "xyz"}
        self.v_air_pre2 = {axis: 0.0 for axis in "xyz"}
        self.v_air_pre3 = {axis: 0.0 for axis in "xyz"}
        self.v_air_pre4 = {axis: 0.0 for axis in "xyz"}
        self.v_air_pre5 = {axis: 0.0 for axis in "xyz"}
        self.first_run = {axis: True for axis in "xyz"}
        self.setWindowTitle("DIET Sapienza - 3D Anemometer")
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        canvas = FigureCanvas(Figure(figsize=(15, 15)))
        self.host = QtWidgets.QLineEdit()
        self.host.setPlaceholderText("Anemometer name or IP")
        self.host.setText("foxd27.local")
        self.port = QtWidgets.QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(1883)
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
        self.main_button = QtWidgets.QPushButton()
        self.main_button.setText("Start")
        self.main_button.setDefault(True)
        self.main_button.clicked.connect(self.on_click)
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
        graph_layout = QtWidgets.QHBoxLayout()
        graph_layout.addWidget(QtWidgets.QLabel("Axes: "))
        for axis in "xyz":
            graph_layout.addWidget(self.axes_sel[axis])
        graph_layout.addWidget(QtWidgets.QLabel("Duration: "))
        graph_layout.addWidget(self.duration)
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

    def clear(self):
        self.logfile.setText("")
        self.host.setDisabled(False)
        self.port.setDisabled(False)
        for axis in "xyz":
            self.axes_sel[axis].setDisabled(False)
        self.duration.setDisabled(False)
        self.choose.setDisabled(False)
        self.main_button.setText("Start")
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
        self.queue = queue.Queue()
        if getattr(self,"fetcher", None):
            self.fetcher.stop()
        self.fetcher = None

    def get_axes(self):
        return [axis for axis in "xyz" if self.axes_sel[axis].isChecked()]

    def on_click(self):
        if not self.state:
            if getattr(self,"fetcher", None):
                self.fetcher.stop()
            try:
                self.fetcher = MQTTFetcher(self.host.text(), self.port.value(), "anemometer", self.parse_message)
                self.fetcher.start()
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", str(e))
                return
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
        elif self.state == "calibration":
            self.state = "measure"
            self.main_button.setText("Stop")
            self.CALIBRATION = self.compute_calibration()
            self.print("")
            for axis in self.get_axes():
                for sensor in [0,1]:
                    for var in ["dist0", "sample0", "phi0", "T0", "tof0"]:
                        key = f"{var}_{axis}{sensor}"
                        val = self.CALIBRATION[key]
                        self.print(f"{key}: {val:.4f} ", end="")
                    key = f"f_{axis}{sensor}"
                    val = self.CALIBRATION[key]
                    self.print(f"{key}: {val}")
            self.print(f"[{datetime.now().isoformat(' ')}] Calibration end.")
        elif self.state == "measure":
            self.state = "stopped"
            self.main_button.setText("Clear")
            if getattr(self,"fetcher", None):
                self.fetcher.stop()
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
                dist = self.CALIBRATION_INPUT[f"dist_{axis}{sensor}"]
                if not len(dist):
                    self.init_canvas()
                    self.axes_sel[axis].setChecked(False)
                    continue
                dist0 = np.mean(dist) * 1000 # distance in micron

                peak = self.CALIBRATION_INPUT[f"peak_{axis}{sensor}"]
                sample0 = np.max(peak) + 1

                phi_array = np.array([phi[sample0] for phi in self.CALIBRATION_INPUT[f"phi_{axis}{sensor}"]])
                T = self.CALIBRATION_INPUT[f"T_{axis}{sensor}"]
                f = self.CALIBRATION_INPUT[f"f_{axis}{sensor}"]
                #print("old")
                #print(phi_array*180/np.pi)

                mean_phi = np.mean(phi_array)
                std_phi = np.std(phi_array)
                # Definisci i limiti (ad esempio, 2 deviazioni standard dalla media)
                lower_bound = mean_phi - std_phi
                upper_bound = mean_phi + std_phi
                # Filtra gli outlier
                filtered_phi = phi_array[(phi_array >= lower_bound) & (phi_array <= upper_bound)]
                #print("new")
                #print(filtered_phi*180/np.pi)
                # Calcola la media senza outlier
                #phi0_array = np.mean(filtered_phi)
                #phi0 = np.mean(phi)
                #print(phi0*180/np.pi)
                #print(phi0_array*180/np.pi)
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
        v_suono = {}
        v_suono_mean = {}
        delta_fase_pre = {}
        v_mean = {}
        v_std = {}
        vel_lower_bound = {}
        vel_upper_bound = {}
        values = {}
        values_suono = {}
        temp_sonica = {}

        for i, axis in enumerate(self.get_axes()):

            if self.first_run[axis]:  # Se è la prima volta, inizializza a zero
                print("dentro")
                self.v_air_pre1[axis] = 0.0
                self.v_air_pre2[axis] = 0.0
                self.v_air_pre3[axis] = 0.0
                self.v_air_pre4[axis] = 0.0
                self.v_air_pre5[axis] = 0.0

                self.v_suono_pre1[axis] = 0.0
                self.v_suono_pre2[axis] = 0.0
                self.v_suono_pre3[axis] = 0.0


                # Flag prima esecuzione diventa False dopo il primo ciclo
                self.first_run[axis] = False

        for i, axis in enumerate(self.get_axes()):
            delta_fase_pre[axis] = {}

            n = measure["counter"]
            for item in measure.get(axis, []):
                if "ch101" in item:
                    sensor = item["ch101"]

                    if f"{sensor}" not in delta_fase_pre[axis]:
                        delta_fase_pre[axis][f"{sensor}"] = 0.0

                    rho, phi = polar(item["i"], item["q"])

                    dist0 = self.CALIBRATION[f"dist0_{axis}{sensor}"]
                    sample0 = self.CALIBRATION[f"sample0_{axis}{sensor}"]
                    tof0 = self.CALIBRATION[f"tof0_{axis}{sensor}"]
                    phi0 = self.CALIBRATION[f"phi0_{axis}{sensor}"]

                    f = self.CALIBRATION[f"f_{axis}{sensor}"]
                    # Calcolo della differenza di fase in radianti
                    delta_fase = phi[sample0] - phi0

                    # Correzione della differenza di fase se supera pi radianti
                    if abs(delta_fase) > np.pi:
                        delta_fase = -1 * np.sign(delta_fase) * (2 * np.pi - abs(delta_fase))

                    #if 0 <= abs(delta_fase) <= np.pi*0.00001:
                    #    delta_fase = delta_fase_pre[axis].get(f"{sensor}")  # Usa il valore precedente se necessario

                    #if abs(delta_fase) > np.pi*0.2:
                    #    delta_fase=(delta_fase_pre[axis][f"{sensor}"]+delta_fase)/2
                    rapporto=delta_fase/np.pi

                    if abs(delta_fase) <= np.pi*0.8:
                        delta_fase=(delta_fase_pre[axis][f"{sensor}"]*(rapporto)+delta_fase*(1-rapporto))/2
                    elif abs(delta_fase) > np.pi*0.8:
                        delta_fase=delta_fase_pre[axis][f"{sensor}"]

                    delta_fase_pre[axis][f"{sensor}"] = delta_fase

                    tof = tof0 -  (1000000 * delta_fase) / (2 * np.pi * f)
                    self.TOF[f"{axis}{sensor}"] = tof

            # Calcola la velocità del flusso d'aria se entrambi i TOF sono disponibili
            if f"{axis}0" in self.TOF and f"{axis}1" in self.TOF:
                dist = [self.CALIBRATION[f"dist0_{axis}{sensor}"] for sensor in [0,1]]
                v_air[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] - dist[1] / self.TOF[f"{axis}1"])
                v_suono[axis] = 0.5 * (dist[0] / self.TOF[f"{axis}0"] + dist[1] / self.TOF[f"{axis}1"])
            # Aggiungi i valori in una lista per calcolare la media e la deviazione standard
            #values[axis] = [v_air[axis], self.v_air_pre1[axis], self.v_air_pre2[axis], self.v_air_pre3[axis], self.v_air_pre4[axis], self.v_air_pre5[axis]]
            values[axis] = [v_air[axis], self.v_air_pre1[axis], self.v_air_pre2[axis], self.v_air_pre3[axis]]
            v_mean[axis] = np.mean(values[axis])
            v_std[axis] = np.std(values[axis])
            print(values)

            values_suono[axis] = [v_suono[axis], self.v_suono_pre1[axis], self.v_suono_pre2[axis], self.v_suono_pre3[axis]]
            #v_suono_mean[axis] = np.mean(values_suono[axis])
            #vel_lower_bound[axis] = v_mean[axis] - 1.8 * v_std[axis]
            #vel_upper_bound[axis] = v_mean[axis] + 1.8 * v_std[axis]
            #print(vel_lower_bound)
            #print(vel_upper_bound)
            # Se la velocità è fuori dai limiti, ripristina il valore precedente
            #if v_air[axis] > vel_upper_bound[axis] or v_air[axis] < vel_lower_bound[axis]:
            #    print(f"Valore fuori dai limiti: v_air[axis] = {v_air[axis]}, ripristinato a v_air_pre1[axis] = {self.v_air_pre1[axis]}")
            #    v_air[axis] = self.v_air_pre1[axis]

            # Stampa i valori di v_air e dei precedenti prima di aggiornare
            #print(f"Valori correnti per {axis}: v_air = {v_air[axis]}, v_air_pre1 = {self.v_air_pre1[axis]}, v_air_pre2 = {self.v_air_pre2[axis]}, v_air_pre3 = {self.v_air_pre3[axis]}")

            #if abs(v_air[axis]-self.v_air_pre1[axis])>0.3:
            #    v_air[axis]=v_mean[axis]

            # Aggiorna i valori precedenti
            self.v_air_pre5[axis] = self.v_air_pre4[axis]
            self.v_air_pre4[axis] = self.v_air_pre3[axis]
            self.v_air_pre3[axis] = self.v_air_pre2[axis]
            self.v_air_pre2[axis] = self.v_air_pre1[axis]
            self.v_air_pre1[axis] = v_air[axis]
            v_air[axis]=v_mean[axis]


            self.v_suono_pre3[axis] = self.v_suono_pre2[axis]
            self.v_suono_pre2[axis] = self.v_suono_pre1[axis]
            self.v_suono_pre1[axis] = v_suono[axis]
            #v_suono[axis]=v_suono_mean[axis]


            temp_sonica[axis]=v_suono[axis]*v_suono[axis]/(1.4*287)-273.15

            # Stampa i nuovi valori dopo l'aggiornamento
            #print(f"Nuovi valori per {axis}: v_air_pre3 = {self.v_air_pre3[axis]}, v_air_pre2 = {self.v_air_pre2[axis]}, v_air_pre1 = {self.v_air_pre1[axis]}")

        if len(v_air) == len(self.get_axes()):
            v_air['timestamp'] = measure["timestamp_end"]
            self.print(f'{v_air["timestamp"]:.4f} ', end="")
            for i, axis in enumerate(self.get_axes()):
                self.print(f" {v_air[axis]:+6.4f}", end="")
            for i, axis in enumerate(self.get_axes()):
                self.print(f" {temp_sonica[axis]:+3.2f}", end="")
            self.print("")
            self.queue.put(v_air)

    def init_canvas(self):
        self.fig.clf()
        self.fig.set_tight_layout(True)
        self.lines = {}
        self.ax = self.fig.subplots()
        self.fig.suptitle("Vair")
        t, x = [datetime.now()], [np.nan]
        fmt = dates.DateFormatter('%H:%M:%S')
        self.ax.xaxis.set_major_formatter(fmt)
        self.ax.set_ylim(-1, 1)
        for i, axis in enumerate(self.get_axes()):
            self.lines[f"{axis}.line"], = self.ax.plot(t, x, label=f"v_{axis}")
            self.lines[f"{axis}.dots"], = self.ax.plot(t, x, 'o', markersize=3, label="_")
        self.ax.legend(loc='upper left')
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
        y_lim = [[],[]]
        for i, axis in enumerate(self.get_axes()):
            t, y = self.lines[f"{axis}.line"].get_data(True)
            for v_air in data:
                t = np.hstack((t, datetime.fromtimestamp(v_air["timestamp"])))
                y = np.hstack((y, v_air[axis]))
            t_min = datetime.fromtimestamp(t[-1].timestamp() - self.duration.value())
            t = t[t >= t_min]
            y = y[-t.size:]
            y_lim[0].append(np.nanmin(y))
            y_lim[1].append(np.nanmax(y))
            self.lines[f"{axis}.line"].set_data(t, y)
            self.lines[f"{axis}.dots"].set_data(t, y)
        self.ax.set_xlim(t_min, t[-1])
        #y_min, y_max = np.min(y_lim[0]), np.max(y_lim[1])
        y_min, y_max = -1.0, 1.0
        self.ax.set_ylim(y_min-0.02*abs(y_min), y_max+0.02*abs(y_max))
        self.fig.canvas.draw_idle()

if __name__ == "__main__":
    qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow()
    app.show()
    app.activateWindow()
    app.raise_()
    qapp.exec()

