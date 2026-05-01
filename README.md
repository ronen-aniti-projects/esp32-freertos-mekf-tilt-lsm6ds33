# First-Principles Implementation/Validation of Embedded Inertial MEKF for Tilt Estimation on ESP32 under FreeRTOS 

## Learning Objective
My learning objective is to demonstrate (i) that I’m able to implement, from first-principles, in firmware, under FreeRTOS, on an ESP32, a multiplicative extended Kalman filter (MEKF) that correctly estimates the angular tilt of an LSM6DS33 6-axis MEMS gyroscope-accelerometer IMU, embedded on a MinIMU-9 v5 IMU breakout board, (ii) that I’m able to explain, in a coherent way, the implementation process–from deciphering the underpinning mathematics and logic of the MEKF, to understanding how the I2C sensor register reads work, to organizing the project for an RTOS, to finally carrying out simple manual tests that confirm the filter achieved its intended purpose. 

## Technical Problem Statement
Implement, under FreeRTOS, on an ESP32, an MEKF (multiplicative extended Kalman filter) that correctly determines, from LSM6DS33 gyroscope and accelerometer measurements, the angular tilt of the LSM6DS33 embedded on the MinIMU-9 v5 IMU breakout board. Here, the term “angular tilt” refers to a description of angular orientation that includes only the roll and pitch angular degrees of freedom (DoFs), not the yaw DoF, which indicates rotation about the gravity “down” direction in the inertial reference frame. 

asdfasdf 
