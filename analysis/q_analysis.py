import numpy as np 
import matplotlib.pyplot as plt 


    
plt.style.use("bmh")

rows = []
expected_cols = 44

with open("log/test15.csv") as f:
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


R = (2.0 * qw[:, None, None]**2- 1.0) * np.eye(3) - 2.0 * qw[:, None, None] * q_skew + 2.0 * q_v[:, :, None] * q_v[:, None, :]



x_axis = R[:, :, 0]
y_axis = R[:, :, 1]
z_axis = R[:, :, 2]

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.quiver(0, 0, 0, 1, 0, 0, color="r")
ax.quiver(0, 0, 0, 0, 1, 0, color="g")
ax.quiver(0, 0, 0, 0, 0, 1, color="b")

ax.plot(x_axis[:, 0], x_axis[:, 1], x_axis[:, 2], color="r", label=r"$\mathbf{R}_{0:3,0}$")
ax.plot(y_axis[:, 0], y_axis[:, 1], y_axis[:, 2], color="g", label=r"$\mathbf{R}_{0:3,1}$")
ax.plot(z_axis[:, 0], z_axis[:, 1], z_axis[:, 2], color="b", label=r"$\mathbf{R}_{0:3,2}$")

ax.quiver(0, 0, 0, x_axis[0, 0], x_axis[0, 1], x_axis[0, 2], color="r")
ax.quiver(0, 0, 0, y_axis[0, 0], y_axis[0, 1], y_axis[0, 2], color="g")
ax.quiver(0, 0, 0, z_axis[0, 0], z_axis[0, 1], z_axis[0, 2], color="b")

ax.scatter(x_axis[-1, 0], x_axis[-1, 1], x_axis[-1, 2], color="r", marker="o")
ax.scatter(y_axis[-1, 0], y_axis[-1, 1], y_axis[-1, 2], color="g", marker="o")
ax.scatter(z_axis[-1, 0], z_axis[-1, 1], z_axis[-1, 2], color="b", marker="o")

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_title(r"Propagation-Only Drift of $\mathbf{R}$")
ax.set_box_aspect([1, 1, 1])
ax.legend()
plt.savefig("docs/media/q_drift_prop_only.png", dpi=150, bbox_inches="tight")
plt.show()
