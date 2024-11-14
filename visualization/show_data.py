#!/usr/bin/env python3

from dash import Dash, html, dcc, callback, Output, Input
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import os, sys, serial, json, logging, threading, time
import numpy as np
logger = logging.getLogger()
logger.setLevel(logging.INFO)

measure = {}
last_timestamp = time.time()

def polar(i, q):
    c = np.array(i) + np.array(q) * 1j
    return (np.abs(c), np.angle(c, deg=True))

app = Dash()

app.layout = html.Div(children=[
    html.H1(children='H10 Anemometer test data', style={'textAlign':'center'}),
    dcc.Graph(id='graph-content'),
    dcc.Interval(
        id='interval-component',
        interval=100, # in milliseconds
        n_intervals=0
    )
])

@callback(
    Output('graph-content', 'figure'),
    Input('interval-component', 'n_intervals')
)
def update_graph(n):
    global measure, last_timestamp
    fig = make_subplots(rows=2, cols=2)
    timestamp = measure.get("timestamp", 0)
    if timestamp > last_timestamp:
        try:
            for i, item in enumerate(measure["data"]):
                print("sensor {sensor}:\n\trange {range_mm} mm, amp {amp}, samples {num_samples}".format(**item))
                x = np.arange(item["num_samples"])
                rho, phi = polar(item["i"], item["q"])
                peak = rho.argmax()
                fig.add_trace(go.Scatter(y=rho, mode="lines"), row=i+1, col=1)
                fig.add_trace(go.Scatter(y=phi, mode="lines"), row=i+1, col=2)
                fig.add_vline(peak, line_color="red", row=i+1)
        except Exception as e:
            logger.error(e)
    return fig

def read_serial(port, baud):
    global measure
    s = serial.Serial(port=port, baudrate=baud)
    while True:
        line = s.readline().decode("utf-8").rstrip()
        try:
            data = json.loads(line)
            measure = { 'timestamp': time.time(), 'data': data }
        except Exception:
            logger.info(line)

if __name__ == '__main__':
    port = "/dev/ttyUSB0"
    baud = 921600
    if "-h" in sys.argv:
        me = os.path.basename(sys.argv[0])
        print(f"Usage: {me} [serial port] [baud rate]\n")
        print(f"Default values are {port}, {baud}\n")
        sys.exit(0)
    try:
        port = sys.argv[1]
    except:
        pass
    try:
        baud = int(sys.argv[2])
    except:
        pass
    feed = threading.Thread(target=read_serial, args=[port, baud], daemon=True)
    feed.start()
    app.run()
