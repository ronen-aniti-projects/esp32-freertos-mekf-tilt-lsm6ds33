import numpy as np
import matplotlib.pyplot as plt

plt.style.use("bmh")

rows = []
expected_cols = 92 #44

with open("log/translational.csv") as f:
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

# actual P
#P = data[:, 11:47].reshape(-1, 6, 6) 
# Qd
#P = data[:, 47:83].reshape(-1, 6, 6) 
# S
P = data[:, 83:93].reshape(-1, 3, 3) 

# Compute the eigenvaluse of P
eigs = np.linalg.eigvalsh(P)

# Compute the condition number of P
cond = eigs[:, -1] / eigs[:, 0]
plt.semilogy(t, cond)
plt.title("Condition Number of P")
plt.grid(True)
plt.show()
print(eigs.min())

# Take the minimum eigenvalue
min_eigs = eigs[:, 0]

# Compute the symmetry error 
P_transpose = np.transpose(P, (0, 2, 1))
sym_errs = np.sum(np.abs(P - P_transpose), axis=(1, 2)) # total symmetry error

# Plot the min eigenvalue vs time
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t, min_eigs)
plt.xlabel("Time [s]")
plt.ylabel("Min. eigenvalue of P")
plt.title("PSD Check")

# Plot the symmetry error vs time
plt.subplot(2, 1, 2)
plt.plot(t, sym_errs)
plt.xlabel("Time [s]")
plt.ylabel("Symmetry error")
plt.show()

# Compute the symmetry ratio and plot vs time
err = P - P_transpose 
sym_ratio = np.linalg.norm(err, axis=(1,2)) / np.linalg.norm(P, axis=(1,2))
plt.figure()
plt.plot(t, sym_ratio)
plt.xlabel("time [s]")
#plt.ylabel(r"$\|\mathbf{P} - \mathbf{P}^{T}\|_F / \|\mathbf{P}\|_F$")
#plt.title(r"Symmetry Error of State Covariance $\mathbf{P}$")

#plt.ylabel(r"$\|\mathbf{Q}_d - \mathbf{Q}_d^{T}\|_F / \|\mathbf{Q}_d\|_F$")
#plt.title(r"Symmetry Error of Discrete Gyro Covariance $\mathbf{Q}_d$")

plt.ylabel(r"$\|\mathbf{S} - \mathbf{S}^{T}\|_F / \|\mathbf{S}\|_F$")
plt.title(r"Symmetry Error of Innovation Covariance $\mathbf{S}$")


#plt.savefig("docs/media/p_symmetry_error.png", dpi=150, bbox_inches="tight")
plt.show()

# plot the trace of P
P_trace = np.trace(P, axis1=1, axis2=2)
plt.figure()
plt.plot(t, P_trace)
plt.xlabel("time [s]")
plt.ylabel(r"$\mathrm{tr}(\mathbf{P})$")
plt.title(r"Evolution of State Error Covariance $\mathbf{P}$")
#plt.savefig("docs/media/p_trace_propagation.png", dpi=150, bbox_inches="tight")
plt.show()

# plot all eigs of P to guage the size of uncertainty
eigs = np.linalg.eigvalsh(P)   
#fig, axes = plt.subplots(6, 1, figsize=(8, 12), sharex=True)
fig, axes = plt.subplots(3, 1, figsize=(8, 12), sharex=True)

for i, ax in enumerate(axes):
    ax.plot(t, eigs[:, i])
    ax.set_ylabel(rf"$\lambda_{i+1}$")
    ax.grid(True)

axes[-1].set_xlabel("time [s]")
#fig.suptitle(r"Prediction-Only Eigenvalues of State Covariance Matrix $\mathbf{P}$")
#fig.suptitle(r"Prediction-Only Eigenvalues of Gyro Covariance $\mathbf{Q}_d$")
#fig.suptitle(r"Eigenvalues of State Covariance Matrix $\mathbf{P}$")
#fig.suptitle(r"Eigenvalues of Discrete Gyro Covariance $\mathbf{Q}_d$")
fig.suptitle(r"Eigenvalues of Innovation Covariance $\mathbf{S}$")
plt.tight_layout(rect=[0, 0, 1, 0.97])
#plt.savefig("docs/media/p_eigenvalues_over_time.png", dpi=150, bbox_inches="tight")

plt.show()

#plot the trace of P 1-3
P_trace_attitude = np.trace(P[:, :3, :3], axis1=1, axis2=2)
plt.figure()
plt.plot(t, P_trace_attitude)
plt.xlabel("time [s]")
plt.ylabel(r"$\mathrm{tr}(\mathbf{P}_{0:3,0:3})$ [rad$^2$]")
plt.title(r"Propagation-Only Growth of Attitude Error Covariance $\mathbf{P}_{0:3,0:3}$")
#plt.savefig("docs/media/p_trace_attitude_propagation.png", dpi=150, bbox_inches="tight")

plt.show()


#plot the trace of P 4-6
P_trace_bias = np.trace(P[:, 3:6, 3:6], axis1=1, axis2=2)
plt.figure()
plt.plot(t, P_trace_bias)
plt.xlabel("time [s]")
plt.ylabel(r"$\mathrm{tr}(\mathbf{P}_{3:6,3:6})$ [rad$^2 / s^2$]")
plt.title(r"Propagation-Only Growth of Bias Error Covariance $\mathbf{P}_{3:6,3:6}$")
#plt.savefig("docs/media/p_trace_bias_propagation.png", dpi=150, bbox_inches="tight")
plt.show()
