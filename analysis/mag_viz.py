import sys

import matplotlib.pyplot as plt


path = sys.argv[1]
mx, my, mz = [], [], []
in_mag = False

with open(path) as f:
    for line in f:
        line = line.strip()
        if line == "MAG CAPTURE:":
            in_mag = True
            continue
        if line == "IMU CAPTURE:":
            in_mag = False
            continue
        if not in_mag or not line:
            continue

        parts = line.split(",")
        if len(parts) == 4:
            mx.append(float(parts[1]))
            my.append(float(parts[2]))
            mz.append(float(parts[3]))

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.scatter(mx, my, mz, s=4)
ax.set_xlabel("mx [G]")
ax.set_ylabel("my [G]")
ax.set_zlabel("mz [G]")
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_title(f"{len(mx)} magnetometer samples")
plt.show()
