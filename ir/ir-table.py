import sys
import numpy as np

inFileName = sys.argv[1]
outFileName = sys.argv[2]

# Load data from CSV
data = np.genfromtxt(inFileName, delimiter=',', skip_header=2, skip_footer=2, names=['t', 'V'])

# Trim
startIndex = np.argmax(data['t'] >=0)
t = data['t'][startIndex:]
v = data['V'][startIndex:]

# Quantize
threshold = max(v) / 2
logic = map(lambda y: 1 if y > threshold else 0, v)

# Find crossings
crossings = reduce(lambda table, val: table if val[1] == table[-1][1] else table + [(val[0], val[1], val[0] - table[-1][0])], zip(t, logic), [(t[0], logic[0], 0)])

# Write CSV
np.savetxt(outFileName, crossings, delimiter=', ', fmt=['%1.5f', '%1.0f', '%1.5f'], comments='', header="t, Logic, dt")

