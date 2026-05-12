# First-Principles Implementation/Validation of Embedded Inertial MEKF for Tilt Estimation on ESP32 under FreeRTOS 

## Learning Objective
My learning objective is to demonstrate (i) that I’m able to implement, from first-principles, in firmware, under FreeRTOS, on an ESP32, a multiplicative extended Kalman filter (MEKF) that correctly estimates the angular tilt of an LSM6DS33 6-axis MEMS gyroscope-accelerometer IMU, embedded on a MinIMU-9 v5 IMU breakout board, (ii) that I’m able to explain, in a coherent way, the implementation process–from deciphering the underpinning mathematics and logic of the MEKF, to understanding how the I2C sensor register reads work, to organizing the project for an RTOS, to finally carrying out simple manual tests that confirm the filter achieved its intended purpose. 

## Acknowledgement
I’d like to thank Steven Mitchell of the Maryland Robotics Center at University of Maryland, College Park, for his guidance and mentorship during the development of this project. 
Technical Problem Statement
Implement, under FreeRTOS, on an ESP32, an MEKF (multiplicative extended Kalman filter) that correctly determines, from LSM6DS33 gyroscope and accelerometer measurements, the angular tilt of the LSM6DS33 embedded on the MinIMU-9 v5 IMU breakout board. Here, the term “angular tilt” refers to a description of angular orientation that includes only the roll and pitch angular degrees of freedom (DoFs), not the yaw DoF, which indicates rotation about the z-axis “down” direction in the body-fixed reference frame.

## Contextual Note
Some might think that numerical integration of gyroscope angular rate measurements is sufficient to reconstruct a full 3-DoF angular orientation. However, due to the fact that all sensors exhibit some degree of noise corruption, the numerical integration of gyro-measured rates will keep drifting unless corrected. The MEKF provides a framework for implementing such a correction; for this project that corrective action is accelerometer-derived. When not accelerating along any of its translational DoFs, the accelerometer provides an indication of the 2-DoF tilt of the sensor board relative to the plane perpendicular to the gravity direction (the level plane). Even though these accelerometer measurements  themselves are also noisy, the MEKF is able to combine both sensors’ measurements such that the resulting angular orientation estimate, along the observable DoFs (roll and pitch), becomes less drift-prone. 

## Implementation, Verification, and Validation
I have implemented an MEKF (multiplicative extended Kalman filter) that correctly estimates, from LSM6DS33 gyroscope and accelerometer measurements, the angular tilt of the LSM6DS33 embedded on the MinIMU-9 v5 IMU breakout board. I’m also able to explain, in a coherent way, the implementation process–from deciphering the underpinning mathematics and logic of the MEKF, to understanding how the I2C sensor register reads work, to organizing the project for an RTOS, to finally carrying out simple manual tests that confirm the filter achieved its intended purpose. 

I’ve rooted my implementation in the conceptual framework established by Nikolas Trawny and Stergios I. Roumeliotis in their 2005 University of Minnesota technical report titled Indirect Kalman Filter for 3D Attitude Estimation. After studying the paper, I’ve learned that the MEKF really has two state vectors: one for tracking the “nominal” state, and one for tracking an “error” state local to the current estimate. The nominal state tracks the big-picture quantities we care about: (i) the unit-quaternion mapping the world frame into the sensor frame, and (ii) the gyroscope rate biases. However, due to the fact that the set of all unit-quaternions is not a vector space, many of the incremental filter computations occur within a localized “tangent space”--a sort of derivative space–taken at the nominal estimate. The error state is a vector within this tangent space and essentially provides an estimate of the displacement between the current nominal estimate’s quaternion and bias and the true quaternion and biases. To resemble that quaternion delta, since unit-quaternions are off-limits, the filter math employs rotation vector representation, a sort of axis-angle representation having magnitude equal to the rotation angle and components pointing in the direction of the rotation axis.  



I’ve learned that the Trawny and Roumeliotis MEKF framework comprises two primary algorithmic steps: prediction and correction. These steps work together to update not only the nominal estimate but also the uncertainty regarding the estimate of the error state. In MEKF-predict, the estimate is propagated forward based only on the latest gyroscope measurement. In MEKF-correct, the estimate is corrected based only on the latest accelerometer measurement. 

This is the core math underpinning the MEKF-predict step. Key components include integrating the nominal quaternion forward in time based on the latest gyroscope reading and propagating the state covariance forward based on the assumed gyroscope noise density and the time since previous gyroscope measurement.




Likewise, this is the core math underpinning the MEKF-correct. The key components include computation of the measurement Jacobian, of the measurement residual, of the Kalman gain, of the correction vector, of the quaternion multiplicative update, or the gyroscope bias additive update, of the bias-compensated gyroscope estimate, and of the covariance update. 



Notably, I’ve implemented all of this math in C code (in math_lib.c/.h and fusion.h/.c), leveraging floating point arrays to represent matrices and vectors. 

With regards to how I’ve organized this project, the MEKF implementation and sensor driver codes are structured as an ESP-IDF project that is compiled on my PC and flashed onto the ESP32, while the analysis scripts, which I used for debugging, verification, and validation, are Python scripts that live on my PC. On the firmware side, I have modules that address IMU interfacing, MEKF logic, mathematics helpers, and main driver code. On the analysis, verification, and validation side, I have Python scripts for timing jitter analysis, accelerometer calibration, gyroscope noise analysis, and MEKF intermediate results analysis. The following diagram illustrates the project’s architecture. 



In essence, main.c imports the functionality of lsm_driver.c, MEKF.c, and math_library.c to drive three tasks: one handles IMU polling, one handles MEKF computation, and one handles UART logging. This code lives on the ESP32. The ESP32 is wired to the LSM6DS33 via I2C enabled pins. The ESP32 is also connected to the PC via a UART-USB bridge. When the ESP32 boots, it configures the LSM6DS33 to a 208 Hz accel and gyro ODR, a ±245 dps gyro FSR, and a ± 2g FSR, by writing to control registers 16, 17, and 18 respectively.  Following configuring the IMU, main.c starts the three RTOS tasks. IMU Polling Task has a period of 1 ms. Every period, it will check for a new gyro and accel measurement, then, if one is ready, it will package the measurement as a custom imu_scaled_sample_t and pass it to MEKF task via IMU Sample Queue. When MEKF task receives data on IMU Sample Queue, it immediately processes it according to the MEKF algorithm. Following the computation, MEKF Task will package the processed data as a custom fusion_log_t and pass that to UART Logging Task via Logging Queue. UART Logging Task allows for CSV-type logging of intermediate MEKF calculations, which played a key role in the implementation, verification, and validation of this project. 

Because I opted to implement a sensor driver from scratch, I had to learn how to interpret the raw LSM6DS33 data register bits into physical units. The sensor datasheet indicates that registers 34 (0x22) to 45 (0x2D) contain the gyroscope and accelerometer output measurements. For this register interval, two registers together encode the measurement along one axis of either the gyro or the accel. For any pair of registers encoding the measurement along one sensor axis, the eight bits of the first register indicate the least significant byte of the measurement, while the eight bits of the next register indicate the most significant byte of the same measurement (an idea called “little-endian”). The datasheet indicates that each of these 16-bit words encodes a signed value (“two’s complement”), which needs to further be scaled to physical units by a scale factor provided in the datasheet specific to the configured sensor FSR (“sensitivity scaling”). In my driver code, I consider all of these specifications, leveraging  uint8_t for a 12-byte all-data buffer, int16_t for interpreting bytes into two’s complement words, and preprocessor macro constants for scale factors. 

Because I chose to implement, for capturing IMU measurements, a polling scheme rather than a FIFO-based interrupt scheme, I placed special emphasis on ensuring my sensor driver never retrieve the same measurement twice. First, IMU Polling Task invokes the driver’s lsm6ds_read_scaled_sample(). The driver invokes lsm6ds33_data_ready(), which reads IMU Register 30, bits 0 and 1, which indicate whether the gyro and accel measurements contained in data registers are fresh. If both bits are set, the driver proceeds to extract the measurements with a burst read of the accelerometer and gyroscope data registers, Register 34 (0x22) to Register 45 (0x2D). The IMU driver invokes esp_timer_get_time() to assign a timestamp to the measurement at the time of retrieval. 

Part of my IMU-polling verification process involved an analysis of the IMU sample interval timing distribution. Having set the sensor ODR to 208 Hz (4.8 ms period) and the IMU Polling Task to 250 Hz (4 ms period), and having implemented polling logic that waits a full period if IMU data is not ready, I expected successive IMU measurements to be spaced either 4 ms or 8 ms apart. To test my hypothesis, I implemented code in firmware to log several thousand successive IMU samples, then I plotted the time deltas as a histogram. The results confirm my hypothesis. When I repeated a similar logging experiment but changed the IMU Polling Task period to 1 ms, IMU samples were spaced either 4 ms or 5 ms apart, a result conforming to that same idea. 



In developing this project, I dedicated care to calibrating the LSM6DS33’s accelerometer, for the reason that an accurate accelerometer measurement is directly tied to the MEKF-correct step being logically consistent. The actual calibration process involved designing and 3D printing a custom calibration fixture, selecting a calibration math model, collecting accelerometer data in various poses, formulating a least-squares regression problem, solving for the unknown parameters of the least-squares problem, and finally implementing the results in firmware. 

The math model I decided on was a mapping of true acceleration to measured acceleration. If we think of acceleration as a vector, then geometrically, the measured acceleration is assumed to be equal to the true acceleration, but only after the true acceleration is scaled/sheared/rotated then translated. The A matrix and the b vector are assumed to be the unknown constants of the model. 



The calibration fixture I designed featured a ball and socket joint and a cube-shaped top that allowed mounting of the sensor board in mutually orthogonal directions. The bubble level helped ensure that each pose was perpendicular to the gravity direction. 














To solve for the unknown calibration constants, I formulated an over-determined system of 18 linear equations in 12 unknowns. The 18 equations came from the fact that I considered six poses and from the fact that the accelerometer measures acceleration along three axes. For each pose, a different face of the sensor board is positioned perpendicular to the gravity direction. For example, for one pose, the accelerometer’s +Z face is positioned so that it’s aligned as closely as possible with the gravity down direction. For each pose, I collected approximately 90 seconds of accelerometer measurements, then averaged the measurement along each axis. With those averaged measurements, I constructed the LHS of the system. To construct the RHS, I assembled a matrix-vector product representing how the calibration parameters would transform the expected gravity measurement along each axis in each of the six poses. After I assembled the system, I solved it with least-squares regression. The results seemed believable: an A matrix close to I and a small but not negligible b vector.  So I then implemented the calibration math in lsm_driver.c’s calibrate_accel(). 















An extremely important notion in Kalman filtering is that both process and measurement noise are assumed to be zero-mean Gaussian white. Regarding the definition of these terms, zero-mean Gaussian means that a distribution is centered at zero, looks bell-shaped and is symmetric. White noise refers to the idea that the expected value of any element of the outer product formed from the two samples is only nonzero when the samples are taken at the same moment in time. Process noise refers to the noise term appended to the state equations, while measurement noise refers to the noise term appended to the measurement equations. The following set of equations summarizes the process noise model explained in Trawny and Roumeliotis’s paper. It’s a model that I studied extensively while developing my MEKF.  





In implementing my project, I decided to characterize the noise generated by the gyroscope against this zero-mean Gaussian assumption. Initially, I was thinking that MEKF process noise is gyroscope noise; however, as I collected data, I changed my thinking once I considered that physical vibrations and other electromagnetic interference patterns were likely being picked up by the gyroscope. 

In one test, I collected several minutes of gyroscope measurements with the sensor board strapped motionless, then plotted the mean-subtracted distributions, which suggest that the process noise does reasonably conform to the zero-mean Gaussian assumption. 



With that same dataset, I plotted the power spectra of the mean-subtracted time-series since white noise has a flat power spectrum. To do this, I constructed, in Python/Numpy, a Lomb-Scargle periodogram from the same mean-subtracted time series. I used the Lomb-Scargle periodogram (LSP)  instead of the fast Fourier transform (FFT) because the LSP extracts frequency content from time-series data having variable inter-sample timing, whereas the FFT assumes uniform inter-sample timing. The presence of several peaks on the resulting diagrams, even if caused by external vibrations and not generated by the gyroscope itself, is suggestive that the process noise does not exactly conform to a white noise model. 





During the course of this project, I designed both a calibration fixture and a test fixture in Autodesk Fusion and printed both on a Bambu Labs A1 mini 3D printer. The design of both fixtures was similar, with both featuring a ball-and-socket joint for achieving arbitrary tilt angles and featuring M4 fasteners to achieve gravity-perpendicular leveling of their bottom parts. The calibration fixture’s top part features a cube-shaped structure that allows for mounting of the sensor board into the six mutually-orthogonal directions. The test fixture’s top part has a rectangular mounting bed for the sensor board. The test fixture’s socket component features shallow 2 mm circular holes spaced 18 degrees apart in a spherical pattern. This allows for mounting the test fixture’s top part to known tilt angles, an idea I leveraged when I validated that the MEKF tilt estimate settled at ground-truth values. I designed the interface between the ball-and-socket to be what I learned is called a “contact-fit” interface, meaning that the joint doesn’t simply collapse under its own weight–it requires manual adjustment to change angular position. The following pictures show screenshots from the design and printing process of the two fixtures. 





The most telling portion of the validation process of my implementation involved a series of tests I designed whereby I positioned the sensor board on the test fixture at 0 deg, 18 deg, and 36 deg tilt, then assessed logged data against expectations. 

The first test involved positioning the sensor board at 0 tilt, then inspecting the drift pattern, over approximately five minutes, of the estimated sensor-board reference frame and of the estimated tilt angle. To visualize the drifting reference frame, I converted the logged quaternion time evolution into a rotation matrix time evolution to form a sort of 3D reference frame trajectory plot. To visualize the tilt angle evolution, I simply plotted the tilt-angle time evolution, where the tilt angle is the computed angle between the body-fixed z-axis of the sensor frame (third column of the sensor board rotation matrix) and the gravity down direction in the inertial frame. 

And I conducted two versions of this 0 degree tilt test. For the first version, I disabled the MEKF-correct step and assessed the drift. For the second version, I ran the full MEKF algorithm. The results I determined supported my expectations. For the case of MEKF-correct-disabled, the 3D sensor frame trajectory supports that sensor frame drift occurs in all three angular DoFs. The MEKF-predict step integrates noise-affected gyro angular rates to form an updated nominal quaternion. So this full-DoF drift is expected. For the case of MEKF-correct, the results support that sensor frame drift occurs in only the yaw DoF–defined as the rotational DoF about the sensor-frame body-z axis. This is exactly what the 3D plot shows. 



The tilt angle trajectories I collect support that while the MEKF-correct curbs drift in the tilt estimate, MEKF-correct-disabled does not. This is also an expected result. We also see that MEKF-correct estimates the correct tilt angle, 0 deg, to within a small margin of error– attributable to imperfect mounting of the board to the test fixture. 



A second test I designed involved examining state covariance drift matters for the case MEKF-correct-disabled and MEKF-correct. To gauge how large state uncertainty was growing over time, I specifically looked at the time evolution of the state covariance matrix, i.e. the time evolution of the sum of the variance of all error state elements. For the case of MEKF-correct-disabled, the resulting plot supports that MEKF-correct-disabled causes the trace of the state covariance to grow large. For the case of MEKF-correct, the resulting plot supports that the MEKF-correct step does well to reduce the uncertainty in the state estimate. This is the expected result. 


A third test I designed involved examining whether the MEKF maintained the correct mathematical structure of the three covariance matrices that it recursively updates. All covariance matrices must be symmetric. In addition to this symmetry requirement, the error-state covariance matrix and the discretized process noise covariance matrix must be positive semi-definite, while the innovation covariance matrix must be positive definite. If all of the eigenvalues of a matrix are nonnegative, then that matrix is positive semi-definite; if all of the matrix’s eigenvalues are greater than zero, the matrix is positive definite. A matrix is symmetric when its off-diagonal elements correspond, such that the ijth element equals its jith element. In computational linear algebra, as a consequence of floating-point roundoff error, a matrix can be approximately symmetric. An algorithm that, in theory, preserves symmetry may, in computational implementation, only approximately preserve symmetry. 

So, for this test, I logged the evolution of all covariance matrices for over 100 seconds while the sensor board was motionless and positioned at a level tilt til. I conducted data collection for MEKF-correct-disabled and for MEKF-correct, then I examined eigenvalues and symmetry. The results support that MEKF-correct-disabled and MEKF-correct preserve the correct definiteness for all three covariance matrices. The results also support that both preserve symmetry. These are the expected results. 












With the sensor board held in the same level orientation, stationary, I proceeded to also confirm that the MEKF-correct step maintains a small measurement residual. The resulting plot supports that it does. The measurement residual is basically the difference between what the MEKF-correct step expects the accelerometer measures, given the nominal quaternion estimate, and what the accelerometer actually measures. 


 
As a next test, I assessed the MEKF-correct’s drift behavior and ability to settle on the correct tilt angle at both 36 deg tilt and at 18 deg tilt. As expected, the resulting plots support that drift occurs only about the yaw DoF and that the tilt angle estimate settles quickly to within a small margin of error, attributable to mounting imperfections, of the correct value.





