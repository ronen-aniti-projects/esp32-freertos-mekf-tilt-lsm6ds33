import numpy as np
import pandas as pd
from scipy.signal import lombscargle
import matplotlib.pyplot as plt
 
df = pd.read_csv('log/imu_250_3.csv', header=None,
                 names=['timestamp_us','ax','ay','az','gx','gy','gz'])
 
t = df['timestamp_us'].values.astype(float) / 1e6
 
freqs = np.linspace(0.1, 120, 500)
ang_freqs = 2 * np.pi * freqs
 
gx = df['gx'].values - df['gx'].mean()
gy = df['gy'].values - df['gy'].mean()
gz = df['gz'].values - df['gz'].mean()
 
pgram_gx = lombscargle(t, gx, ang_freqs, normalize=True)
pgram_gy = lombscargle(t, gy, ang_freqs, normalize=True)
pgram_gz = lombscargle(t, gz, ang_freqs, normalize=True)
 
 
plt.style.use("bmh")

fig, axes = plt.subplots(3, 1, figsize=(7, 8), sharex=True)

axes[0].plot(freqs, pgram_gx)
axes[0].axhline(pgram_gx.mean(), color="red", ls="--", label="mean")
axes[0].set_title("gx")
axes[0].set_ylabel("normalized power")
axes[0].legend()

axes[1].plot(freqs, pgram_gy)
axes[1].axhline(pgram_gy.mean(), color="red", ls="--", label="mean")
axes[1].set_title("gy")
axes[1].set_ylabel("normalized power")
axes[1].legend()

axes[2].plot(freqs, pgram_gz)
axes[2].axhline(pgram_gz.mean(), color="red", ls="--", label="mean")
axes[2].set_title("gz")
axes[2].set_xlabel("frequency [Hz]")
axes[2].set_ylabel("normalized power")
axes[2].legend()

fig.suptitle("Gyroscope Noise Spectrum After Bias Removal")

plt.tight_layout()
plt.savefig("docs/media/gyro_whiteness.png", dpi=200, bbox_inches="tight")
plt.show()
