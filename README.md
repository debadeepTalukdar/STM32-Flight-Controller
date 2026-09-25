# STM32-Flight-Controller
# STM32 IMU Sensor Fusion & Flight Control

A bare-metal closed-loop flight controller implemented in C on an ARM Cortex-M3 (STM32F103C8T6). This project reads 6-DOF telemetry from an MPU6050 IMU via I2C, cleans the high-frequency analog noise using a real-time Kalman Filter, and stabilizes the pitch/roll axes using a custom PID controller.

## Hardware Stack
*   **MCU:** STM32F103C8T6 (ARM Cortex-M3)
*   **Sensor:** MPU6050 (3-axis Accelerometer + 3-axis Gyroscope)
*   **Protocol:** I2C (400 kHz Fast Mode)

## Core Architecture
1.  **Data Acquisition:** Raw analog-to-digital sensor data is fetched via low-level I2C register reads.
2.  **Sensor Fusion (Kalman Filter):** Accelerometer data (noisy but stable over time) and Gyroscope data (smooth but suffers from kinematic drift) are fused to produce an accurate, drift-free state estimation of the drone's pitch and roll.
3.  **PID Stabilization:** The filtered angles are fed into a Proportional-Integral-Derivative (PID) loop to calculate the necessary motor output corrections to maintain a level setpoint (0 degrees).

## File Structure
*   `Core/Src/main.c`: The primary RTOS/Super-loop managing the I2C polling, timing (`dt`), and the PID calculations.
*   `Core/Src/kalman.c`: The mathematical matrix implementation of the 1D Kalman filter state predictor and updater.
