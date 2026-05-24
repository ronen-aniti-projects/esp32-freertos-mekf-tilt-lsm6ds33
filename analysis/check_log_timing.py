import sys
import numpy as np

path = "log/higher_baud.csv"

EXPECTED_DT_US = 48000
THRESHOLD_US = 4800

t_us = []

with open(path) as f:
    for line in f:
        try:
            t_us.append(int(line.split(",", 1)[0]))
        except ValueError:
            pass

t_us = np.array(t_us)
dt_us = np.diff(t_us)

bad = np.abs(dt_us - EXPECTED_DT_US) > THRESHOLD_US

print(f"samples: {len(t_us)}")
print(f"intervals checked: {len(dt_us)}")
print(f"bad intervals: {np.sum(bad)}")

#for i in np.where(bad)[0]:
#    print(f"row {i}->{i+1}: dt_us={dt_us[i]}")