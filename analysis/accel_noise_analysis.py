import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lombscargle

plt.style.use("bmh")

rows = []
expected_cols = 4

with open("log/accel_noise.csv") as f:
    for line in f:
        parts = [x.strip() for x in line.split(",")]
        if len(parts) != expected_cols:
            continue
        try:
            rows.append([float(x) for x in parts])
        except ValueError:
            continue

data = np.array(rows)

t = (data[:, 0] - data[0, 0]) * 1e-6

means = data.mean(axis=0)

ax = data[:, 1] - means[1]
ay = data[:, 2] - means[2]
az = data[:, 3] - means[3]

# Verify the Gaussian assumption (~0 skewness, ~3 kurtosis)
ax_var = np.var(ax)
ax_std = np.sqrt(ax_var)
ax_skew = np.mean((ax / ax_std) ** 3)
ax_kurt = np.mean((ax / ax_std) ** 4)


ay_var = np.var(ay)
ay_std = np.sqrt(ay_var)
ay_skew = np.mean((ay / ay_std) ** 3)
ay_kurt = np.mean((ay / ay_std) ** 4)

az_var = np.var(az)
az_std = np.sqrt(az_var)
az_skew = np.mean((az / az_std) ** 3)
az_kurt = np.mean((az / az_std) ** 4)

print("Gaussian check: ")
print(" ax var: ", ax_var)
print(" ay var: ", ay_var)
print(" az var: ", az_var)
print(" ax skew: ", ax_skew)
print(" ay skew: ", ay_skew)
print(" az skew: ", az_skew)
print(" ax kurtosis: ", ax_kurt)
print(" ay kurtosis: ", ay_kurt)
print(" az kurtosis: ", az_kurt)
print(" elapsed time [s]: ", t[-1])

# 2g fsr -> .061 mg/lsb -> m/s^2/lsb
lsb = .061/1000.0*9.81 

bins = np.arange(-100*lsb, 100*lsb, lsb)
#bins=50
fig, axes = plt.subplots(3, 1, figsize=(10, 8), constrained_layout=True)
plt.suptitle("Stationary Accelerometer Noise Distribution")

plt.subplot(3, 1, 1)
plt.hist(ax, bins=bins)
plt.title(r"X-Axis Acceleration Residual ($a_x - \bar{a}_x$)")
plt.xlabel(r"$a_x - \bar{a}_x$ [$\mathrm{m/s^2}$]")
plt.ylabel("count")

plt.subplot(3, 1, 2)
plt.hist(ay, bins=bins)
plt.title(r"Y-Axis Acceleration Residual ($a_y - \bar{a}_y$)")
plt.xlabel(r"$a_y - \bar{a}_y$ [$\mathrm{m/s^2}$]")
plt.ylabel("count")

plt.subplot(3, 1, 3)
plt.hist(az, bins=bins)
plt.title(r"Z-Axis Acceleration Residual ($a_z - \bar{a}_z$)")
plt.xlabel(r"$a_z - \bar{a}_z$ [$\mathrm{m/s^2}$]")
plt.ylabel("count")

plt.show()

freqs = np.linspace(0.1, 120, 500)
ang_freqs = 2 * np.pi * freqs

pgram_ax = lombscargle(t, ax, ang_freqs, normalize=True)
pgram_ay = lombscargle(t, ay, ang_freqs, normalize=True)
pgram_az = lombscargle(t, az, ang_freqs, normalize=True)

fig, axes = plt.subplots(3, 1, figsize=(7, 8), sharex=True)

axes[0].plot(freqs, pgram_ax)
axes[0].axhline(pgram_ax.mean(), color="red", ls="--", label="mean")
axes[0].set_title(r"X-Axis Acceleration Residual ($a_x - \bar{a}_x$)")
axes[0].set_ylabel("normalized power")
axes[0].legend()

axes[1].plot(freqs, pgram_ay)
axes[1].axhline(pgram_ay.mean(), color="red", ls="--", label="mean")
axes[1].set_title(r"Y-Axis Acceleration Residual ($a_y - \bar{a}_y$)")
axes[1].set_ylabel("normalized power")
axes[1].legend()

axes[2].plot(freqs, pgram_az)
axes[2].axhline(pgram_az.mean(), color="red", ls="--", label="mean")
axes[2].set_title(r"Z-Axis Acceleration Residual ($a_z - \bar{a}_z$)")
axes[2].set_xlabel("frequency [Hz]")
axes[2].set_ylabel("normalized power")
axes[2].legend()

fig.suptitle("Accelerometer Noise Spectrum After Mean Removal")

plt.tight_layout()
plt.show()