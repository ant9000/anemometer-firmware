#!/usr/bin/python3

import os, sys, getopt, gpiod, serial, time, json, signal
import paho.mqtt.client as mqtt
from collections import OrderedDict
from led import LED
from measure import Measure

NAME  = "anemometer"
DEFAULTS = {"axes": "xyz", "baudrate": 576000, "delay": 0.0, "tdelay": 0.012}
PORTS = {"x": "/dev/ttyS1", "y": "/dev/ttyS2", "z": "/dev/ttyS3"}

def usage():
    me = os.path.basename(sys.argv[0])
    usage = f"""
{me} [options]

-a AXES      only measure along AXES, a non empty substring of "xyz"; defaults to "{DEFAULTS["axes"]}"
-b BAUDRATE  open serial ports at BAUDRATE; defaults to {DEFAULTS["baudrate"]}
-d DELAY     DELAY between successive measures, in seconds; defaults to {DEFAULTS["delay"]}
-t TDELAY    TDELAY among axes measure triggers, in seconds; defaults to {DEFAULTS["tdelay"]}
"""
    print(usage)

options = DEFAULTS.copy()
try:
    optlists, args = getopt.getopt(sys.argv[1:], 'a:b:d:t:')
    optdict = dict(optlists)
    options["axes"] = "".join(OrderedDict.fromkeys(optdict.get("-a", DEFAULTS["axes"])))
    options["baudrate"] = optdict.get("-b", DEFAULTS["baudrate"])
    options["delay"] = optdict.get("-d", DEFAULTS["delay"])
    options["tdelay"] = optdict.get("-t", DEFAULTS["tdelay"])

    if not options["axes"] or not (set(options["axes"]) <= set("xyz")):
        raise getopt.GetoptError(f'\"{options["axes"]}\" is not a non empty subset of "xyz"')
    try:
        options["baudrate"] = int(options["baudrate"])
        assert(options["baudrate"] in [115200*i for i in range(1,9)])
    except Exception as err:
        raise getopt.GetoptError(f"{err}")
    try:
        options["delay"] = float(options["delay"])
        assert(options["delay"] >= 0)
    except Exception as err:
        raise getopt.GetoptError(f"{err}")
    try:
        options["tdelay"] = float(options["tdelay"])
        assert(options["tdelay"] >= 0)
    except Exception as err:
        raise getopt.GetoptError(f"{err}")

except getopt.GetoptError as err:
    print(err)
    usage()
    sys.exit(1)

AXES = options["axes"]
BAUD = options["baudrate"]
DELAY = options["delay"]
TDELAY = options["tdelay"]
print("Initializing LEDs...", flush=True, end="")
leds = {}
for color in ["red", "green", "blue"]:
    leds[color] = LED(f"x_{color}")
print(" done.")

def signal_handler(sig, frame):
    for led in leds.values():
        led.stop()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGHUP, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
leds["red"].on()

print("Initializing gpio lines...", flush=True, end="")
gpio = gpiod.Chip("gpiochip0")
enable = gpio.find_line("enable_1v8")
reset = gpio.find_lines([f"reset_{axis}" for axis in AXES])
triggers = gpio.find_lines([f"trigger_{axis}" for axis in AXES])
enable.request(consumer=NAME, type=gpiod.LINE_REQ_DIR_OUT)
reset.request(consumer=NAME, type=gpiod.LINE_REQ_DIR_OUT)
triggers.request(consumer=NAME, type=gpiod.LINE_REQ_DIR_OUT)
print(" done.")

print("Opening serial ports...", flush=True, end="")
ports = {axis: serial.Serial(PORTS[axis], baudrate=BAUD, timeout=.05) for axis in AXES}
print(" done.")

print("Connecting to local MQTT server...", flush=True, end="")
mqttClient = mqtt.Client(NAME)
mqttClient.connect("localhost", 1883)
mqttClient.loop_start()
print(" done.")

print(f"Initializing axes {AXES}", flush=True, end="")
triggers.set_values([1]*len(AXES))
enable.set_value(0)
time.sleep(.2)
reset.set_values([1]*len(AXES))
enable.set_value(1)
time.sleep(.2)
reset.set_values([0]*len(AXES))
for i in range(3):
    print(".", flush=True, end="")
    time.sleep(1)
print(" done.")

# consume initial banners
for axis, port in ports.items():
    while True:
        line = port.readline()
        if not line:
            break
        line = line.decode("utf-8").rstrip()
        print(f"[{axis}] {line}")

def trigger_measure():
    triggers.set_values([1]*len(AXES))
    if TDELAY == 0:
        triggers.set_values([0]*len(AXES))
    else:
        trig = [1]*len(AXES)
        for i in range(len(AXES)):
            trig[i] = 0
            triggers.set_values(trig)
            if len(AXES) > 1:
                time.sleep(TDELAY)

leds["red"].off()
leds["blue"].blink()
counter = 0

air_speed = Measure(axes=AXES, n_campioni=30, q_kalman=0.005)
while True:
    try:
        #print(f"Measure {counter}...")
        counter += 1
        measures = {"timestamp_start": time.time(), "counter": counter}
        trigger_measure()
        data = {axis: b"" for axis in AXES}
        while not set(AXES) < set(measures):
            for axis, port in ports.items():
                if port.in_waiting > 0:
                    data[axis] += port.read(port.in_waiting)
                    if data[axis].endswith(b"\n"):
                        for line in data[axis].decode("utf-8").split("\n"):
                            line = line.strip()
                            if line:
                                try:
                                    measures[axis] = json.loads(line)
                                except json.decoder.JSONDecodeError:
                                    print(f"[{axis}] {line}")
                        data[axis] = b""
        measures["timestamp_end"] = time.time()
        #msg = json.dumps(measures)
        #info = mqttClient.publish(topic=NAME, payload=msg.encode("utf-8"), qos=0)
        #info.wait_for_publish()
        #print(f" done.")
        v_air = air_speed.compute(measures)
        if v_air:
            msg = json.dumps(v_air)
            info = mqttClient.publish(topic=NAME, payload=msg.encode("utf-8"), qos=0)
            info.wait_for_publish()
    except Exception as e:
        print(f"ERROR: {e}")
    time.sleep(DELAY)
