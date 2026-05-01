import sys
import numpy as np
import matplotlib.pyplot as plt
import statsmodels.api as sm

plt.style.use("bmh")

filename = "log/imu_250_2.csv"

#filename = "log/calib_apr8/p1.csv"
data = np.loadtxt(filename, delimiter=",", usecols=(4, 5, 6))

means = data.mean(axis=0)
# Best means:
#  [ 0.05883807 -0.14938366 -0.04615213] 

print("Means from stationary gyro (rad/s): ", means, "\n")

# Subtract means from gyro measurements
gx = data[:, 0] - means[0]
gy = data[:, 1] - means[1]
gz = data[:, 2] - means[2]

# Verify the Gaussian assumption (~0 skewness, ~3 kurtosis)
gx_var = np.var(gx)
gx_std = np.sqrt(gx_var)
gx_skew = np.mean((gx / gx_std) ** 3)
gx_kurt = np.mean((gx / gx_std) ** 4)

gy_var = np.var(gy)
gy_std = np.sqrt(gy_var)
gy_skew = np.mean((gy / gy_std) ** 3)
gy_kurt = np.mean((gy / gy_std) ** 4)

gz_var = np.var(gz)
gz_std = np.sqrt(gz_var)
gz_skew = np.mean((gz / gz_std) ** 3)
gz_kurt = np.mean((gz / gz_std) ** 4)

print("Gaussian check: ")
print(" gx var: ", gx_var)
print(" gy var: ", gy_var)
print(" gz var: ", gz_var)
print(" gx skew: ", gx_skew)
print(" gy skew: ", gy_skew)
print(" gz skew: ", gz_skew)
print(" gx kurtosis: ", gx_kurt)
print(" gy kurtosis: ", gy_kurt)
print(" gz kurtosis: ", gz_kurt)


# Verify the white noise assumption. 
# Note: basically gx with all non-zero lag gx should tend to be zero, same for other axes.
#       gx with gy, with gz for all lags including zero must tend to be zero.
corr_gxgx = np.correlate(gx, gx, mode="full")
corr_gygy = np.correlate(gy, gy, mode="full")
corr_gzgz = np.correlate(gz, gz, mode="full")
corr_gxgy = np.correlate(gx, gy, mode="full")
corr_gxgz = np.correlate(gx, gz, mode="full")
corr_gygx = np.correlate(gy, gx, mode="full")
corr_gygz = np.correlate(gy, gz, mode="full")
corr_gzgx = np.correlate(gz, gx, mode="full")
corr_gzgy = np.correlate(gz, gy, mode="full")

# 245 dps -> 8.75e-3 dps/lsb -> rps/lsb
lsb = 8.75e-3 * (np.pi / 180.0)

bins = np.arange(-100*lsb, 100*lsb, lsb)
#bins=50
fig, axes = plt.subplots(3, 1, figsize=(10, 8), constrained_layout=True)
plt.suptitle("Stationary Gyroscope Noise Distribution")

plt.subplot(3, 1, 1)
plt.hist(gx, bins=bins)
plt.title("gx")
plt.xlabel("angular rate residual [rad/s]")
plt.ylabel("count")

plt.subplot(3, 1, 2)
plt.hist(gy, bins=bins)
plt.title("gy")
plt.xlabel("angular rate residual [rad/s]")
plt.ylabel("count")

plt.subplot(3, 1, 3)
plt.hist(gz, bins=bins)
plt.title("gz")
plt.xlabel("angular rate residual [rad/s]")
plt.ylabel("count")

plt.savefig("docs/media/gyro_residual_histograms.png", dpi=150, bbox_inches="tight")
plt.show()


plt.figure(figsize=(10, 8))
plt.subplot(3, 3, 1)
plt.plot(corr_gxgx)
plt.title("gx corr")

plt.subplot(3, 3, 2)
plt.plot(corr_gygy)
plt.title("gy corr")

plt.subplot(3, 3, 3)
plt.plot(corr_gzgz)
plt.title("gz corr")

# Plot cross correlations
plt.subplot(3, 3, 4)
plt.plot(corr_gxgy)
plt.title("gxgy corr")

plt.subplot(3, 3, 5)
plt.plot(corr_gxgz)
plt.title("gxgz corr")

plt.subplot(3, 3, 6)
plt.plot(corr_gygx)
plt.title("gygx corr")

plt.subplot(3, 3, 7)
plt.plot(corr_gygz)
plt.title("gygz corr")

plt.subplot(3, 3, 8)
plt.plot(corr_gzgx)
plt.title("gzgx corr")

plt.subplot(3, 3, 9)
plt.plot(corr_gzgy)
plt.title("gzgy corr")

plt.tight_layout()


fig, axes = plt.subplots(1, 3, figsize=(10, 3.5))
sm.qqplot(gx, line="s", ax=axes[0])
sm.qqplot(gy, line="s", ax=axes[1])
sm.qqplot(gz, line="s", ax=axes[2])
axes[0].set_title("gx")
axes[1].set_title("gy")
axes[2].set_title("gz")


fig.tight_layout()


plt.show()

data_t = np.loadtxt(filename, delimiter=",")
t_us = data_t[:, 0]
dt_us = np.diff(t_us)
plt.hist(dt_us, bins=np.arange(dt_us.min(), dt_us.max() + 50, 50))
plt.show()


