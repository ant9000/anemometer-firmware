#!/usr/bin/python3

import os, sys, getopt, queue, time
from datetime import datetime
import logging, json, threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as dates
from matplotlib.animation import FuncAnimation
import paho.mqtt.client as mqtt

PLOTS = ["AmpPhasePlot", "PeakPhasesPlot"]
DEFAULTS = {"axes": "xyz", "delta": -1, "host": "foxd27.local", "plot": 0, "qsize": 1000, "samples": 150, "seconds": 60}

def usage():
    me = os.path.basename(sys.argv[0])
    usage = f"""
{me} [options]

-a AXES      only measure along AXES, a non empty substring of "xyz"; defaults to "{DEFAULTS["axes"]}"
-d DELTA     consider phase at peak + DELTA samples; defaults to {DEFAULTS["delta"]}
-h HOST      connect to HOST; defaults to {DEFAULTS["host"]}
-p PLOT_NUM  use plot PLOT_NUM to graph data; defaults to {DEFAULTS["plot"]}; values are {", ".join([f"{i}={n}" for i,n in enumerate(PLOTS)])}
-q QSIZE     use an incoming data queue size of QSIZE; defaults to {DEFAULTS["qsize"]}; use 0 for unbounded
-s SAMPLES   graph data points up to SAMPLES samples; defaults to {DEFAULTS["samples"]}
-t SECONDS   graph timeseries up to SECONDS seconds (for PeakPhasesPlot); defaults to {DEFAULTS["seconds"]}
"""
    print(usage)

options = DEFAULTS.copy()
try:
    optlists, args = getopt.getopt(sys.argv[1:], 'a:d:h:p:q:s:t:')
    optdict = dict(optlists)
    options["axes"] = "".join(sorted(optdict.get("-a", DEFAULTS["axes"])))
    options["delta"] = optdict.get("-d", DEFAULTS["delta"])
    options["host"] = optdict.get("-h", DEFAULTS["host"])
    options["plot"] = optdict.get("-p", DEFAULTS["plot"])
    options["qsize"] = optdict.get("-q", DEFAULTS["qsize"])
    options["samples"] = optdict.get("-s", DEFAULTS["samples"])
    options["seconds"] = optdict.get("-t", DEFAULTS["seconds"])
    if not options["axes"] or not (options["axes"] in "xyz"):
        raise getopt.GetoptError(f'\"{options["axes"]}\" is not a non empty subset of "xyz"')
    try:
        options["delta"] = int(options["delta"])
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
    try:
        options["samples"] = int(options["samples"])
        assert(options["samples"] > 0)
    except Exception as err:
        raise getopt.GetoptError(f"{err}")
    try:
        options["seconds"] = int(options["seconds"])
        assert(options["seconds"] > 0)
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
HOST = options["host"]
PLOT = PLOTS[options["plot"]]
QSIZE = options["qsize"]
SAMPLES = options["samples"]
SECONDS = options["seconds"]

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
    c = np.pad(c, (0, SAMPLES), mode="constant", constant_values=0)[:SAMPLES]
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
            # skip first samples to ignore ringdown peak
            skip = 25
            peak = skip + rho[skip:].argmax()
            if peak > skip:
                phi_sel = phi[peak + DELTA]
            else:
                peak = np.nan
                phi_sel = np.nan
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
        zero = np.zeros(SAMPLES)
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
                self.ax2.item(*idx).set_ylim(0, 40000)
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
    COLORS = ['#dd0000', '#0000dd']
    COLORS2 = ['#ff5f00', '#00a0ff']
    def init(self):
        self.lines = {}
        self.ax = self.fig.subplots(len(AXES))
        if len(AXES) == 1:
            self.ax = [self.ax]
        self.ax2 = self.ax.copy()
        x, y = [datetime.now()], [np.nan]
        fmt = dates.DateFormatter('%H:%M:%S')
        for i, axis in enumerate(AXES):
            self.ax[i].set_ylabel(f'Phase {axis}')
            self.ax[i].set_ylim(-180, 180)
            self.ax[i].xaxis.set_major_formatter(fmt)
            self.ax2[i] = self.ax[i].twinx()
            self.ax2[i].set_ylabel(f'Peak {axis}')
            self.ax2[i].set_ylim(0, 100)
            for j in range(2):
                self.lines[f"{i},{j}.phase_line"], = self.ax[i].plot(x, y, color=f'{self.COLORS[j]}', label=f'phase {j}')
                self.lines[f"{i},{j}.phase_dots"], = self.ax[i].plot(x, y, 'o', color=f'{self.COLORS[j]}', markersize=3, label='_')
                self.lines[f"{i},{j}.peak_line"], = self.ax2[i].plot(x, y, color=f'{self.COLORS2[j]}', label=f'peak {j}')
                self.lines[f"{i},{j}.peak_dots"], = self.ax2[i].plot(x, y, 'o', color=f'{self.COLORS2[j]}', markersize=3, label='_')
            self.ax[i].legend(loc='upper left')
            self.ax2[i].legend(loc='upper right')
    def update(self, n):
        data = []
        while True:
            item = self.get_data()
            if not item:
                break
            data.append(item)
        if not data:
            return
        self.fig.suptitle(f'Measure: {data[-1]["measure"]} - phase at peak{DELTA:+}')
        t, x, y = {}, {}, {}
        for i, axis in enumerate(AXES):
            for j in [0, 1]:
                k = f"{i},{j}"
                t[k], x[k] = self.lines[f"{k}.phase_line"].get_data(True)
                _, y[k] = self.lines[f"{k}.peak_line"].get_data(True)
        for measure in data:
            for i, axis in enumerate(AXES):
                for item in measure[axis]:
                    j, mode, rho, peak, phi, phi_sel = item
                    k = f"{i},{j}"
                    t[k] = np.hstack((t[k], measure["timestamp"]))
                    x[k] = np.hstack((x[k], phi_sel))
                    y[k] = np.hstack((y[k], peak))
        for i, axis in enumerate(AXES):
            for j in [0, 1]:
                k = f"{i},{j}"
                t_min = datetime.fromtimestamp(t[k][-1].timestamp() - SECONDS)
                t[k] = t[k][t[k] >= t_min]
                x[k] = x[k][-t[k].size:]
                y[k] = y[k][-t[k].size:]
                self.lines[f"{k}.phase_line"].set_data(t[k], x[k])
                self.lines[f"{k}.phase_dots"].set_data(t[k], x[k])
                self.lines[f"{i},{j}.peak_line"].set_data(t[k], y[k])
                self.lines[f"{i},{j}.peak_dots"].set_data(t[k], y[k])
                self.ax[i].set_xlim(t_min, t[k][-1])

fetcher = MQTTFetcher(HOST, 1883, "anemometer", parse_measure)
fetcher.start()
graph = get_class(PLOT)()
graph.show()

if counter:
    print(f"Queue size: {data_queue.qsize()} - Discarded: {discarded}/{counter} ({discarded * 100 /counter:.2f}%)")
