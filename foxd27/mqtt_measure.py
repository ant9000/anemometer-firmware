#!/usr/bin/python3 -u

import sys, math, json, re
import paho.mqtt.client as mqtt
from io import StringIO
from measure import Measure

NAME = "measure"
HOSTNAME = "localhost"
TOPICS = ["anemometer/raw","+/anemometer/raw"]

class Capturing(list):
    def __enter__(self):
        self._stdout = sys.stdout
        sys.stdout = self._stringio = StringIO()
        return self
    def __exit__(self, *args):
        self.extend(self._stringio.getvalue().splitlines())
        del self._stringio    # free up some memory
        sys.stdout = self._stdout

class FloatEncoder(json.JSONEncoder):
    def __init__(self, decimals=2, *args, **kwargs):
        self.decimals = decimals
        super().__init__(*args, **kwargs)
    def _process_floats(self, obj):
        if isinstance(obj, float) and not math.isnan(obj):
            D = 10**self.decimals
            return int(obj*D)/D
        elif isinstance(obj, dict):
            return {k: self._process_floats(obj[k]) for k in obj}
        elif isinstance(obj, list):
            return [self._process_floats(v) for v in obj]
        else:
            return obj
    def encode(self, obj):
        obj = self._process_floats(obj)
        return super().encode(obj)

measures = {}
def on_message(c, u, m):
    global measures
    try:
        topic = re.sub("/raw$","", m.topic)

        payload = str(m.payload.decode("utf-8"))
        data = json.loads(payload)

        if topic not in measures:
            axes = "".join([axis for axis in "xyz" if axis in data])
            measures[topic] = Measure(axes=axes, n_campioni=30, q_kalman=0.005)

        with Capturing() as output:
            v_air = measures[topic].compute(data)
        for line in output:
            print(f"[{topic}] {line}")

        if v_air:
            msg = json.dumps(v_air, cls=FloatEncoder, decimals=4)
            mqttClient.publish(topic=topic, payload=msg.encode("utf-8"), qos=0)
            print(f"[{topic}] sent message on {topic}")
    except Exception as e:
        print("ERROR: {e} - discarding")

print("Connecting to  MQTT server...", flush=True, end="")
mqttClient = mqtt.Client(NAME)
mqttClient.connect(HOSTNAME, 1883)
print(" done.")
mqttClient.on_message = on_message
mqttClient.subscribe([(t,0) for t in TOPICS])
try:
    mqttClient.loop_forever()
except KeyboardInterrupt:
    pass
