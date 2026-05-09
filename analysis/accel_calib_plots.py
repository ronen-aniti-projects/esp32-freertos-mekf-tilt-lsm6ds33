import matplotlib.pyplot as plt
import numpy as np
import glob

files = sorted(glob.glob("log/calib_apr8/p*.csv"))

# Extract means for each key pose
for filename in files:
    data = np.genfromtxt(filename, delimiter=",", usecols=(1, 2, 3), invalid_raise=False)

    data = data[~np.isnan(data).any(axis=1)]

    ax_mean = np.mean(data[:, 0])
    ay_mean = np.mean(data[:, 1])
    az_mean = np.mean(data[:, 2])

    print(f"{filename}: ax={ax_mean:.6f}, ay={ay_mean:.6f}, az={az_mean:.6f}")

# Each key pose contributes 3 equations


M_P1 = np.array([0.159386, -9.756954, 0.447697])
M_P2 = np.array([-0.100679, 9.869653, 0.568927])
M_P3 = np.array([9.746036, -0.080454, 0.091311])
M_P4 = np.array([-9.868618, 0.136421, 0.038947])
M_P5 = np.array([-0.044527, 0.153064, 10.041362])
M_P6 = np.array([-0.125500, 0.069435, -9.549665])

M = np.hstack((M_P1, M_P2, M_P3, M_P4, M_P5, M_P6))

gx_P1 = 0.0
gy_P1 = -9.81
gz_P1 = 0.0 

gx_P2 = 0.0
gy_P2 = 9.81
gz_P2 = 0.0 

gx_P3 = 9.81
gy_P3 = 0.0
gz_P3 = 0.0 

gx_P4 = -9.81
gy_P4 = 0.0
gz_P4 = 0.0 

gx_P5 = 0.0
gy_P5 = 0.0
gz_P5 = 9.81

gx_P6 = 0.0
gy_P6 = 0.0
gz_P6 = -9.81

H_P1 = np.array([[gx_P1, gy_P1, gz_P1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0, gx_P1, gy_P1, gz_P1,0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gx_P1, gy_P1, gz_P1,0.0, 0.0, 1.0]])

H_P2 = np.array([[gx_P2, gy_P2, gz_P2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0, gx_P2, gy_P2, gz_P2,0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gx_P2, gy_P2, gz_P2,0.0, 0.0, 1.0]])

H_P3 = np.array([[gx_P3, gy_P3, gz_P3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0, gx_P3, gy_P3, gz_P3,0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gx_P3, gy_P3, gz_P3,0.0, 0.0, 1.0]])

H_P4 = np.array([[gx_P4, gy_P4, gz_P4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0, gx_P4, gy_P4, gz_P4,0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gx_P4, gy_P4, gz_P4,0.0, 0.0, 1.0]])

H_P5 = np.array([[gx_P5, gy_P5, gz_P5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0, gx_P5, gy_P5, gz_P5,0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gx_P5, gy_P5, gz_P5,0.0, 0.0, 1.0]])

H_P6 = np.array([[gx_P6, gy_P6, gz_P6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0, gx_P6, gy_P6, gz_P6,0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gx_P6, gy_P6, gz_P6,0.0, 0.0, 1.0]])

H = np.vstack((H_P1, H_P2, H_P3, H_P4, H_P5, H_P6))

params = np.linalg.inv(H.T @ H) @ H.T @ M

A = params[:9].reshape(3, 3)
b = params[9:].reshape(3,)
print(params)
print(A)
print(b)

calib_P1 = np.linalg.inv(A) @ (M_P1 - b)
calib_P2 = np.linalg.inv(A) @ (M_P2 - b)
calib_P3 = np.linalg.inv(A) @ (M_P3 - b)
calib_P4 = np.linalg.inv(A) @ (M_P4 - b)
calib_P5 = np.linalg.inv(A) @ (M_P5 - b)
calib_P6 = np.linalg.inv(A) @ (M_P6 - b)

print(M_P1, calib_P1)
print(M_P2, calib_P2)
print(M_P3, calib_P3)
print(M_P4, calib_P4)
print(M_P5, calib_P5)
print(M_P6, calib_P6)

print("A inverse: ")
print(np.linalg.inv(A))

calib_points = np.array([calib_P1, calib_P2, calib_P3, calib_P4, calib_P5, calib_P6])
original_points = np.array([M_P1, M_P2, M_P3, M_P4, M_P5, M_P6])

plt.style.use("bmh")


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.scatter(calib_points[:, 0], calib_points[:, 1], calib_points[:, 2], label="calibrated")
ax.scatter(original_points[:, 0], original_points[:, 1], original_points[:, 2], label="raw")
ax.legend()
ax.set_xlabel(r"$a_x$ [m/s$^2$]")
ax.set_ylabel(r"$a_y$ [m/s$^2$]")
ax.set_zlabel(r"$a_z$ [m/s$^2$]")
plt.title("Raw vs. Calibrated Gravity Poses")
plt.savefig("docs/media/Calibration.png", dpi=150, bbox_inches="tight")

plt.show()