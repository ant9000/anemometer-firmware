#!/usr/bin/python3

import os, sys, csv
import matplotlib.pyplot as plt
import numpy as np

if len(sys.argv) < 2:
    print(f"Usage: {os.path.basename(sys.argv[0])} fname")
    sys.exit(1)
fname = sys.argv[1]

try:
    csvfile = open(fname, newline='')
except Exception as e:
    print("ERROR: can't open '{fname}' ({e})")
    sys.exit(1)

# consume irrelevant lines
while True:
    line = csvfile.readline()
    if not line:
        print(f"ERROR: Cannot find start line in '{fname}'")
        sys.exit(1)
    if line.startswith("TIME;"):
        break
csvfile.seek(csvfile.tell()-len(line), 0)

reader = csv.reader(csvfile, delimiter=";")
fieldnames = [s.replace(" done", "") for s in next(reader)[2:]]
N = len(fieldnames)
data = []
for row in reader:
    if row and row[0] == "PROC inc":
        data.append(row[2:])
data = np.array(data, dtype=np.int32).T
data = [data[i] for i in range(N)]

fig, ax = plt.subplots()
plt.title(fname)
plt.xticks(rotation=90)
ax.set_ylabel('time spent (µs)')

bplot = ax.boxplot(data, sym='', patch_artist=True, labels=fieldnames)
fig.tight_layout()
plt.show()
