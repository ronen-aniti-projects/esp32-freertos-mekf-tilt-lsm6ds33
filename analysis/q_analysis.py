import numpy as np 
import matplotlib.pyplot as plt 
    
plt.style.use("bmh")

rows = []
expected_cols = 92 #44

with open("n18deg.csv") as f:
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
q = data[:, 1:5]
b = data[:, 5:8]
r = data[:, 8:11]

# actual P
P = data[:, 11:47].reshape(-1, 6, 6) 
# Qd
#P = data[:, 47:83].reshape(-1, 6, 6) 
# S
#P = data[:, 83:93].reshape(-1, 3, 3) 

qx = q[:, 0]
qy = q[:, 1]
qz = q[:, 2]
qw = q[:, 3]

q_v = q[:, :3]
q_skew = np.zeros((q.shape[0], 3, 3))
q_skew[:, 0, 1] = -qz 
q_skew[:, 0, 2] = qy 
q_skew[:, 1, 0] = qz 
q_skew[:, 1, 2] = -qx 
q_skew[:, 2, 0] = -qy 
q_skew[:, 2, 1] = qx 


# Convert quaternion into rotation matrix
R = (2.0 * qw[:, None, None]**2- 1.0) * np.eye(3) - 2.0 * qw[:, None, None] * q_skew + 2.0 * q_v[:, :, None] * q_v[:, None, :]

# Compute the tilt angle
tilt = np.arccos(np.clip(np.array([0.0, 0.0, 1.0]) @ np.transpose(R, (0, 2, 1)) @ np.array([0.0, 0.0, 1.0]), -1.0, 1.0))





# Plot the tilt angle vs time
plt.figure()
plt.plot(t, np.rad2deg(tilt))
plt.xlabel("Time [s]")
plt.ylabel("Tilt [deg]")
#plt.title("Prediction-Only Tilt Estimate")
plt.title("Corrected Tilt Estimate")
plt.grid(True)

# Plot the gyro bias estimate vs time
plt.figure()
plt.plot(t, b[:, 0], label="b_x")
plt.plot(t, b[:, 1], label="b_y")
plt.plot(t, b[:, 2], label="b_z")
plt.xlabel("Time [s]")
plt.ylabel("Bias estimate [rad/s]")
plt.title("Gyro Bias Estimate vs Time")
plt.legend()
plt.grid(True)

# Extarct the diagonal elements of P
P_diag = np.diagonal(P, axis1=1, axis2=2)

# Plot the time evolution of each diagonal element of P
fig, axs = plt.subplots(6, 1, sharex=True, figsize=(10, 8))

for i, ax in enumerate(axs):
    ax.plot(t, P_diag[:, i])
    ax.set_ylabel(rf"$\mathbf{{P}}_{{{i+1}{i+1}}}$")
    ax.grid(True)

axs[-1].set_xlabel("time [s]")
fig.suptitle(r"Diagonal Elements of State Error Covariance $\mathbf{P}$")
fig.tight_layout(rect=[0, 0, 1, 0.97])

# Compute the norm of the residual 
r_norm = np.linalg.norm(r, axis=1)

# Plot the 3D residual as a scatter plot 
fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection="3d")
sc = ax.scatter(r[:, 0], r[:, 1], r[:, 2], c=t, s=8, cmap="viridis")
ax.set_xlabel(r"$r_x$ [m/s$^2$]")
ax.set_ylabel(r"$r_y$ [m/s$^2$]")
ax.set_zlabel(r"$r_z$ [m/s$^2$]")
ax.set_title("Accelerometer Residual Trajectory")
fig.colorbar(sc, ax=ax, label="Time [s]", pad=0.1)
fig.tight_layout()

# Plot the time evolution of the residual norm
plt.figure()
plt.plot(t, r_norm)
plt.xlabel("Time [s]")
plt.ylabel(r"$\|\mathbf{r}\|$ [m/s$^2$]")
plt.title("Accelerometer Residual Norm")
plt.grid(True)

# Plot the evolution of the estimation frame
x_axis = R[:, :, 0]
y_axis = R[:, :, 1]
z_axis = R[:, :, 2]

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.quiver(0, 0, 0, 1, 0, 0, color="r")
ax.quiver(0, 0, 0, 0, 1, 0, color="g")
ax.quiver(0, 0, 0, 0, 0, 1, color="b")

ax.plot(x_axis[:, 0], x_axis[:, 1], x_axis[:, 2], color="r", label=r"$\mathbf{R}_{0:3,0}$", alpha=0.6)
ax.plot(y_axis[:, 0], y_axis[:, 1], y_axis[:, 2], color="g", label=r"$\mathbf{R}_{0:3,1}$", alpha=0.6)
ax.plot(z_axis[:, 0], z_axis[:, 1], z_axis[:, 2], color="b", label=r"$\mathbf{R}_{0:3,2}$")

#ax.quiver(0, 0, 0, x_axis[-1, 0], x_axis[-1, 1], x_axis[-1, 2], color="r")
#ax.quiver(0, 0, 0, y_axis[-1, 0], y_axis[-1, 1], y_axis[-1, 2], color="g")
#ax.quiver(0, 0, 0, z_axis[-1, 0], z_axis[-1, 1], z_axis[-1, 2], color="b")

ax.scatter(x_axis[-1, 0], x_axis[-1, 1], x_axis[-1, 2], color="r", marker="o")
ax.scatter(y_axis[-1, 0], y_axis[-1, 1], y_axis[-1, 2], color="g", marker="o")
ax.scatter(z_axis[-1, 0], z_axis[-1, 1], z_axis[-1, 2], color="b", marker="o")

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
########################################################
# CHANGE THIS DEPENDING ON WHETHER THE DATA IS PROP ONLY OR CORRECTION
#ax.set_title(r"Prediction-Only Drift of $\mathbf{R}$")
ax.set_title(r"Corrected Drift of $\mathbf{R}$")
########################################################
ax.set_box_aspect([1, 1, 1])
ax.legend()
plt.savefig("docs/media/q_drift_prop_only.png", dpi=150, bbox_inches="tight")
plt.show()
