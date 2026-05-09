# First-Principles Implementation/Validation of Embedded Inertial MEKF for Tilt Estimation on ESP32 under FreeRTOS 

## Learning Objective
My learning objective is to demonstrate (i) that I’m able to implement, from first-principles, in firmware, under FreeRTOS, on an ESP32, a multiplicative extended Kalman filter (MEKF) that correctly estimates the angular tilt of an LSM6DS33 6-axis MEMS gyroscope-accelerometer IMU, embedded on a MinIMU-9 v5 IMU breakout board, (ii) that I’m able to explain, in a coherent way, the implementation process–from deciphering the underpinning mathematics and logic of the MEKF, to understanding how the I2C sensor register reads work, to organizing the project for an RTOS, to finally carrying out simple manual tests that confirm the filter achieved its intended purpose. 

## Technical Problem Statement
Implement, under FreeRTOS, on an ESP32, an MEKF (multiplicative extended Kalman filter) that correctly determines, from LSM6DS33 gyroscope and accelerometer measurements, the angular tilt of the LSM6DS33 embedded on the MinIMU-9 v5 IMU breakout board. Here, the term “angular tilt” refers to a description of angular orientation that includes only the roll and pitch angular degrees of freedom (DoFs), not the yaw DoF, which indicates rotation about the z-axis “down” direction in the body-fixed reference frame.

## Background Note
Some might think that numerical integration of gyroscope angular rate measurements is sufficient to reconstruct a full 3-DoF angular orientation. But the reality is that, due to the fact that all sensors exhibit some degree of noise corruption, the numerical integration of gyro-measured rates will keep drifting unless corrected. The MEKF provides a framework for implementing such a correction; for this project that corrective action is accelerometer-derived. When not accelerating along any of its translational DoFs, the accelerometer provides an indication of the 2-DoF tilt of the sensor board relative to the plane perpendicular to the gravity direction (the level plane). Even though these accelerometer measurements  themselves are also noisy, the MEKF is able to combine both sensors’ measurements such that the resulting angular orientation estimate, along the observable DoFs (roll and pitch), becomes less drift-prone. 
Implementation, Verification, and Validation
I have implemented an MEKF (multiplicative extended Kalman filter) that correctly determines, from LSM6DS33 gyroscope and accelerometer measurements, the angular tilt of the LSM6DS33 embedded on the MinIMU-9 v5 IMU breakout board. 

This is the evidence: 
Under stationary conditions, the MEKF exhibits the expected estimation behavior. 
Show the quaternion estimate with no correction
Show the quaternion estimate with correction 
Under stationary conditions, all of the MEKF covariance matrices are propagated in a believable and mathematically valid way. 
Show plots to support the PSD of several matrices under no correction
Show plots to support the PSD of several matrices under correction
Under stationary conditions, the filter converges to a reasonable estimate of ground truth tilt. 
Show that cold-start from -36 degrees converges to -36 degrees
Show that cold-start from -18 degrees converges to -18 degrees
Show that cold-start from 0 degrees converges to 0 degrees
Show that cold-start from 18 degrees converges to 18 degrees
Show that cold-start from 36 degrees converges to 36 degrees
Under translational acceleration, the estimate becomes corrupted, which is expected. 
Show a plot of the estimated attitude during vigorous up/down shaking. 



I’m also able to explain, in a coherent way, the implementation process–from deciphering the underpinning mathematics and logic of the MEKF, to understanding how the I2C sensor register reads work, to organizing the project for an RTOS, to finally carrying out simple manual tests that confirm the filter achieved its intended purpose.  

Here’s the evidence: 
Summarize the MEKF mathematical framework I employ and indicate how I employ that mathematics inside of an RTOS program. 


Summarize why accelerometer calibration is important and how I calibrated the accelerometer.
 
Summarize my characterization of the gyroscope noise against the MEKF assumption and the reason for doing this.


