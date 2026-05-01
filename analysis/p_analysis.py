import numpy as np
import matplotlib.pyplot as plt

plt.style.use("bmh")

rows = []
expected_cols = 44

with open("log/proponlysmallerp4.csv") as f:
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
P = data[:, 8:].reshape(-1, 6, 6)


eigs = np.linalg.eigvalsh(P)

cond = eigs[:, -1] / eigs[:, 0]
plt.semilogy(t, cond)
plt.title("Condition Number of P")
plt.grid(True)
plt.show()
print(eigs.min())



min_eigs = eigs[:, 0]

P_transpose = np.transpose(P, (0, 2, 1))
sym_errs = np.sum(np.abs(P - P_transpose), axis=(1, 2)) # total symmetry error

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t, min_eigs)
plt.xlabel("Time [s]")
plt.ylabel("Min. eigenvalue of P")
plt.title("PSD Check")

plt.subplot(2, 1, 2)
plt.plot(t, sym_errs)
plt.xlabel("Time [s]")
plt.ylabel("Symmetry error")
plt.show()

err = P - P_transpose 
sym_ratio = np.linalg.norm(err, axis=(1,2)) / np.linalg.norm(P, axis=(1,2))
plt.figure()
plt.plot(t, sym_ratio)
plt.xlabel("time [s]")
plt.ylabel(r"$\|\mathbf{P} - \mathbf{P}^{T}\|_F / \|\mathbf{P}\|_F$")
plt.title(r"Normalized Symmetry Error of State Covariance $\mathbf{P}$")
plt.savefig("docs/media/p_symmetry_error.png", dpi=150, bbox_inches="tight")
plt.show()

# plot the trace of P
P_trace = np.trace(P, axis1=1, axis2=2)
plt.figure()
plt.plot(t, P_trace)
plt.xlabel("time [s]")
plt.ylabel(r"$\mathrm{tr}(\mathbf{P})$")
plt.title(r"Propagation-Only Growth of State Error Covariance $\mathbf{P}$")
plt.savefig("docs/media/p_trace_propagation.png", dpi=150, bbox_inches="tight")
plt.show()

# plot all eigs of P to guage the size of uncertainty
eigs = np.linalg.eigvalsh(P)   
fig, axes = plt.subplots(6, 1, figsize=(8, 12), sharex=True)

for i, ax in enumerate(axes):
    ax.plot(t, eigs[:, i])
    ax.set_ylabel(rf"$\lambda_{i+1}$")
    ax.grid(True)

axes[-1].set_xlabel("time [s]")
fig.suptitle(r"Propagation-Only Eigenvalues of State Covariance Matrix $\mathbf{P}$")

plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig("docs/media/p_eigenvalues_over_time.png", dpi=150, bbox_inches="tight")

plt.show()

#plot the trace of P 1-3
P_trace_attitude = np.trace(P[:, :3, :3], axis1=1, axis2=2)
plt.figure()
plt.plot(t, P_trace_attitude)
plt.xlabel("time [s]")
plt.ylabel(r"$\mathrm{tr}(\mathbf{P}_{0:3,0:3})$ [rad$^2$]")
plt.title(r"Propagation-Only Growth of Attitude Error Covariance $\mathbf{P}_{0:3,0:3}$")
plt.savefig("docs/media/p_trace_attitude_propagation.png", dpi=150, bbox_inches="tight")

plt.show()

#plot the trace of P 4-6
P_trace_bias = np.trace(P[:, 3:6, 3:6], axis1=1, axis2=2)
plt.figure()
plt.plot(t, P_trace_bias)
plt.xlabel("time [s]")
plt.ylabel(r"$\mathrm{tr}(\mathbf{P}_{3:6,3:6})$ [rad$^2 / s^2$]")
plt.title(r"Propagation-Only Growth of Bias Error Covariance $\mathbf{P}_{3:6,3:6}$")
plt.savefig("docs/media/p_trace_bias_propagation.png", dpi=150, bbox_inches="tight")
plt.show()
