import numpy as np
import matplotlib.pyplot as plt 

plt.style.use("bmh")
data = np.loadtxt("log/imu_250_2.csv", delimiter=",")
t_us = data[:,0]
dt_us = np.diff(t_us)

print(f"num samples: {len(t_us)}")
print(f"mean dt us: {np.mean(dt_us): .2f}")
print(f"std dt_us: {np.std(dt_us):.2f}")
print(f"min dt_us: {np.min(dt_us):.2f}")
print(f"max dt_us: {np.max(dt_us):.2f}")

bin_width = 10 #us
bins = np.arange(dt_us.min(), dt_us.max() + bin_width, bin_width)
plt.hist(dt_us, bins=bins)
plt.title("IMU Sample Interval Distribution")
plt.xlabel(r"sample interval $\Delta t$ [$\mu$s]")
plt.ylabel("sample count")
plt.savefig("docs/media/imu_sample_interval_distribution.png", dpi=150, bbox_inches="tight")

plt.show()