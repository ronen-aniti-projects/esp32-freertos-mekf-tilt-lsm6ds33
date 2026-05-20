import numpy as np
import matplotlib.pyplot as plt

filename = "log/pulseview_predict_correct_timing.csv"
sample_rate_hz = 1_000_000

data = np.loadtxt(filename, delimiter=",", skiprows=1, dtype=int)

# D0
d0 = data[:, 0]

pulse_widths = []
count = 0

for value in d0:
    if value == 1:
        count += 1
    elif count > 0:
        pulse_widths.append(count)
        count = 0

# Handle case where file ends while still high
if count > 0:
    pulse_widths.append(count)

pulse_widths_us = np.array(pulse_widths)  # 1 sample = 1 us

print(f"num pulses: {len(pulse_widths_us)}")
print(f"mean high time: {np.mean(pulse_widths_us):.2f} us")
print(f"min high time:  {np.min(pulse_widths_us):.2f} us")
print(f"max high time:  {np.max(pulse_widths_us):.2f} us")


# Manual plotting
plt.style.use("bmh")

labels = ["Prediction", "Correction", "Full update"]
mean_us = np.array([384.63, 321.69, 702.02])
max_us = np.array([709.00, 417.00, 1151.00])

sample_period_us = 1e6 / 208.0

x = np.arange(len(labels))

plt.figure(figsize=(7, 4))

plt.bar(x, mean_us, label="Mean runtime")


plt.xticks(x, labels)
plt.ylabel("Runtime [us]")
plt.title("MEKF Runtime from GPIO Timing Trace")
plt.legend()
plt.tight_layout()
plt.show()