#!/usr/bin/python3

import os, sys, getopt, queue
from datetime import datetime, timedelta
import logging, json, threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as dates
from matplotlib.animation import FuncAnimation
import paho.mqtt.client as mqtt

PLOTS = ["AmpPhasePlot", "PeakPhasesPlot", "PhaseDifferencePlot"]
DEFAULTS = {"axes": "xyz", "delta": -1, "num": 150, "plot": 0, "qsize": 1000}

def usage():
    me = os.path.basename(sys.argv[0])
    usage = f"""
{me} [options] host

-a AXES      only measure along AXES, a non empty substring of "xyz"; defaults to "{DEFAULTS["axes"]}"
-d DELTA     consider phase at peak + DELTA samples; defaults to {DEFAULTS["delta"]}
-n NUM       graph data points up to NUM samples (for AmpPhasePlot) or seconds (for PeakPhasesPlot); defaults to {DEFAULTS["num"]}
-p PLOT_NUM  use plot PLOT_NUM to graph data; defaults to {DEFAULTS["plot"]}; values are {", ".join([f"{i}={n}" for i,n in enumerate(PLOTS)])}
-q QSIZE     use an incoming data queue size of QSIZE; defaults to {DEFAULTS["qsize"]}; use 0 for unbounded
"""
    print(usage)

options = DEFAULTS.copy()
try:
    optlists, args = getopt.getopt(sys.argv[1:], 'a:d:n:p:q:')
    if not len(args):
        raise getopt.GetoptError(f"missing host")
    host = args[0]
    optdict = dict(optlists)
    options["axes"] = "".join(sorted(optdict.get("-a", DEFAULTS["axes"])))
    options["delta"] = optdict.get("-d", DEFAULTS["delta"])
    options["num"] = optdict.get("-n", DEFAULTS["num"])
    options["plot"] = optdict.get("-p", DEFAULTS["plot"])
    options["qsize"] = optdict.get("-q", DEFAULTS["qsize"])
    if not options["axes"] or not (options["axes"] in "xyz"):
        raise getopt.GetoptError(f'\"{options["axes"]}\" is not a non empty subset of "xyz"')
    try:
        options["delta"] = int(options["delta"])
    except Exception as err:
        raise getopt.GetoptError(f"{err}")
    try:
        options["num"] = int(options["num"])
        assert(options["num"] > 0)
    except Exception as err:
        raise getopt.GetoptError(f"{err}")
    try:
        options["plot"] = int(options["plot"])
        assert(PLOTS[options["plot"]])
    except Exception as err:
        raise getopt.GetoptError(f"{err}")
    try:
        options["qsize"] = int(options["qsize"])
        assert(options["qsize"] >= 0)
    except Exception as err:
        raise getopt.GetoptError(f"{err}")
except getopt.GetoptError as err:
    print(err)
    usage()
    sys.exit(1)

def get_class(class_name):
    return globals()[class_name]

AXES = options["axes"]
DELTA = options["delta"]
NUM = options["num"]
PLOT = PLOTS[options["plot"]]
QSIZE = options["qsize"]

class MQTTFetcher:
    def __init__(self, host, port, channel, parse_message):
        self.client = mqtt.Client()
        self.client.message_callback_add(channel, self.on_message)
        self.client.connect(host, port)
        self.client.subscribe(channel)
        self.parse_message = parse_message
    def start(self):
        t = threading.Thread(target=self.client.loop_forever, daemon=True)
        t.start()
    def on_message(self, client, userdata, msg):
        try:
            payload = str(msg.payload.decode("utf-8"))
            data = json.loads(payload)
            self.parse_message(data)
        except Exception as e:
            print(f"ERROR: {e}")

def polar(i, q):
    c = np.array(i) + np.array(q) * 1j
    c = np.pad(c, (0, NUM), mode="constant", constant_values=0)[:NUM]
    return (np.abs(c), np.angle(c, deg=True))

counter = 0
discarded = 0
data_queue = queue.Queue(QSIZE)

def parse_measure(measure):
    global data_queue, counter, discarded
    counter += 1
    data = {}
    for i, axis in enumerate(AXES):
        data[axis] = []
        for item in measure.get(axis, []):
            if not "ch101" in item:
                continue
            rho, phi = polar(item["i"], item["q"])
            peak = rho.argmax()
            phi_sel = phi[49]#[peak + DELTA]
            data[axis].append([item["ch101"], item["mode"], rho, peak, phi, phi_sel])
        if not data[axis]:
            del data[axis]
    if data:
        if data_queue.full():
            try:
                tmp = data_queue.get_nowait()
                discarded += 1
                print(f'Queue full - discarding oldest measure {tmp["measure"]}')
            except queue.Empty:
                pass
        try:
            data["measure"] = counter
            data["timestamp"] = datetime.fromtimestamp(measure["timestamp_start"])
            data_queue.put_nowait(data)
        except queue.Full:
            print(f"Queue full - discarding current measure {counter}")
            discarded += 1

class BasePlot:
    def __init__(self):
        self.fig = plt.figure(figsize=(15,15))
        self.anim = FuncAnimation(self.fig, self.update, init_func=self.init, interval=50)
    def init(self):
        pass
    def update(self, n):
        pass
    def show(self):
        plt.show()
    def get_data(self):
        global data_queue
        try:
            data = data_queue.get_nowait()
        except queue.Empty:
            data = None
        return data

class AmpPhasePlot(BasePlot):
    def get_index(self, i, j):
        return (len(AXES) == 1) and [j] or [i,j]
    def init(self):
        self.lines = {}
        self.ax = self.fig.subplots(len(AXES),2)
        self.ax2 = self.ax.copy()
        zero = np.zeros(NUM)
        for i, axis in enumerate(AXES):
            for j in range(2):
                idx = self.get_index(i,j)
                self.ax.item(*idx).set_title(f'Sensor {j}')
                if j == 0:
                    self.ax.item(*idx).set_ylabel(f'Axis {axis}')
                self.ax.item(*idx).set_ylim(-180, 180)
                self.lines[f"{i},{j}.phase_line"], = self.ax.item(*idx).plot(zero, 'g', label='phase')
                self.lines[f"{i},{j}.phase_dots"], = self.ax.item(*idx).plot(zero, 'go', markersize=3, label='_')
                self.ax.item(*idx).legend(loc='upper left')
                self.ax2.itemset(*idx, self.ax.item(*idx).twinx())
                self.ax2.item(*idx).set_ylim(0, 10000)
                self.lines[f"{i},{j}.amp_line"], = self.ax2.item(*idx).plot(zero, 'b', label='amp')
                self.lines[f"{i},{j}.amp_dots"], = self.ax2.item(*idx).plot(zero, 'bo', markersize=3, label='_')
                self.lines[f"{i},{j}.amp_peak"]  = self.ax2.item(*idx).axvline(0, color='r', label='peak')
                self.ax2.item(*idx).legend(loc='upper right')
    def update(self, n):
        data = self.get_data()
        if not data:
            return
        self.fig.suptitle(f'Measure: {data["measure"]}')
        for i, axis in enumerate(AXES):
            for item in data[axis]:
                j, mode, rho, peak, phi, phi_sel = item
                self.lines[f"{i},{j}.phase_line"].set_ydata(phi)
                self.lines[f"{i},{j}.phase_dots"].set_ydata(phi)
                #idx = self.get_index(i,j)
                #self.ax2.item(*idx).set_ylim(0, rho.max()*1.05)
                self.lines[f"{i},{j}.amp_line"].set_ydata(rho)
                self.lines[f"{i},{j}.amp_dots"].set_ydata(rho)
                self.lines[f"{i},{j}.amp_peak"].set_xdata(peak)

class PeakPhasesPlot(BasePlot):
    COLORS = ['r', 'b']
    def init(self):
        self.lines = {}
        self.ax = self.fig.subplots(len(AXES))
        if len(AXES) == 1:
            self.ax = [self.ax]
        x, y = [datetime.now()], [np.nan]
        fmt = dates.DateFormatter('%H:%M:%S')
        for i, axis in enumerate(AXES):
            self.ax[i].set_ylabel(f'Axis {axis}')
            self.ax[i].set_ylim(-180, 180)
            self.ax[i].xaxis.set_major_formatter(fmt)
            for j in range(2):
                self.lines[f"{i},{j}.line"], = self.ax[i].plot(x, y, f'{self.COLORS[j]}', label=f'sensor {j}')
                self.lines[f"{i},{j}.dots"], = self.ax[i].plot(x, y, f'{self.COLORS[j]}o', markersize=3, label='_')
            self.ax[i].legend(loc='upper left')
    def update(self, n):
        data = []
        while True:
            item = self.get_data()
            if not item:
                break
            data.append(item)
        if not data:
            return
        self.fig.suptitle(f'Measure: {data[-1]["measure"]} - phase difference{DELTA:+}')
        t, y = {}, {}
        for i, axis in enumerate(AXES):
            for j in [0, 1]:
                k = f"{i},{j}"
                t[k], y[k] = self.lines[f"{k}.line"].get_data(True)
        for measure in data:
            for i, axis in enumerate(AXES):
                for item in measure[axis]:
                    j, mode, rho, peak, phi, phi_sel = item
                    k = f"{i},{j}"
                    t[k] = np.hstack((t[k], measure["timestamp"]))
                    y[k] = np.hstack((y[k], phi_sel))
        for i, axis in enumerate(AXES):
            for j in [0, 1]:
                k = f"{i},{j}"
                t_min = datetime.fromtimestamp(t[k][-1].timestamp() - NUM)
                t[k] = t[k][t[k] >= t_min]
                y[k] = y[k][-t[k].size:]
                self.lines[f"{k}.line"].set_data(t[k], y[k])
                self.lines[f"{k}.dots"].set_data(t[k], y[k])
                self.ax[i].set_xlim(t_min, t[k][-1])


class PhaseDifferencePlot(BasePlot):
    COLORS = ['r', 'b']
    
    def init(self):
        self.lines = {}
        self.ax = self.fig.subplots(len(AXES))
        if len(AXES) == 1:
            self.ax = [self.ax]
        x, y = [datetime.now()], [np.nan]
        fmt = dates.DateFormatter('%H:%M:%S')
        
        for i, axis in enumerate(AXES):
            self.ax[i].set_ylabel(f'Axis {axis}')
            self.ax[i].set_ylim(-180, 180)
            self.ax[i].xaxis.set_major_formatter(fmt)
            
            for j in [0, 1]:
                self.lines[f"{i},{j}.line"], = self.ax[i].plot(x, y, f'{self.COLORS[j]}', label=f'Sensor {j}')
                self.lines[f"{i},{j}.dots"], = self.ax[i].plot(x, y, f'{self.COLORS[j]}o', markersize=3, label='_')
                
            self.ax[i].legend(loc='upper left')
        
        # Memorizziamo i valori precedenti di phi_sel per calcolare la differenza di fase
        self.prev_phi_sel = {axis: [None, None] for axis in AXES}  # None iniziale per entrambi i sensori

    def update(self, n):
        data = []
        while True:
            item = self.get_data()
            if not item:
                break
            data.append(item)
        
        if not data:
            return
        
        # Aggiorna il titolo del grafico
        self.fig.suptitle(f'Measure: {data[-1]["measure"]} - Phase Difference')
        
        t, y = {}, {}
        for i, axis in enumerate(AXES):
            for j in [0, 1]:
                k = f"{i},{j}"
                t[k], y[k] = self.lines[f"{k}.line"].get_data(True)
        
        for measure in data:
            for i, axis in enumerate(AXES):
                for item in measure[axis]:
                    j, mode, rho, peak, phi, phi_sel = item
                    k = f"{i},{j}"
                    
                    # Calcola la differenza di fase solo se abbiamo una misura precedente
                    if self.prev_phi_sel[axis][j] is not None:
                        phase_diff = phi_sel - self.prev_phi_sel[axis][j]
                        
                        t[k] = np.hstack((t[k], measure["timestamp"]))
                        y[k] = np.hstack((y[k], phase_diff))
                    
                    # Memorizza il valore corrente di phi_sel per la prossima iterazione
                    self.prev_phi_sel[axis][j] = phi_sel
        
        for i, axis in enumerate(AXES):
            for j in [0, 1]:
                k = f"{i},{j}"
                t_min = datetime.fromtimestamp(t[k][-1].timestamp() - NUM)
                
                t[k] = t[k][t[k] >= t_min]
                y[k] = y[k][-t[k].size:]
                self.lines[f"{k}.line"].set_data(t[k], y[k])
                self.lines[f"{k}.dots"].set_data(t[k], y[k])
                self.ax[i].set_xlim(t_min, t[k][-1])


fetcher = MQTTFetcher(host, 1883, "anemometer", parse_measure)
fetcher.start()
graph = get_class(PLOT)()
graph.show()

if counter:
    print(f"Queue size: {data_queue.qsize()} - Discarded: {discarded}/{counter} ({discarded * 100 /counter:.2f}%)")
