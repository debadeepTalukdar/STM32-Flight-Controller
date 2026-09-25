#include "stm32f1xx_hal.h"
#include "kalman.h"
#include "mpu6050.h"

// PID Constants
#define KP 1.5f
#define KI 0.05f
#define KD 0.8f

KalmanFilter kalman_pitch;
KalmanFilter kalman_roll;

float pitch_setpoint = 0.0f; // Target level flight
float pid_i_term = 0.0f;
float last_error = 0.0f;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();
    MX_TIM2_Init(); // Hardware timer for dt
    
    MPU6050_Init();
    Kalman_Init(&kalman_pitch);
    Kalman_Init(&kalman_roll);
    
    uint32_t last_time = HAL_GetTick();
    
    while (1) {
        // 1. Calculate time delta (dt)
        uint32_t current_time = HAL_GetTick();
        float dt = (current_time - last_time) / 1000.0f;
        last_time = current_time;
        
        // 2. Read raw registers via I2C
        MPU6050_Data raw_data;
        MPU6050_Read(&raw_data);
        
        // 3. Sensor Fusion: Pass noisy accel angle and drifting gyro rate through Kalman
        float clean_pitch = Kalman_GetAngle(&kalman_pitch, raw_data.accel_pitch, raw_data.gyro_x_rate, dt);
        
        // 4. Closed-Loop PID Calculation
        float error = pitch_setpoint - clean_pitch;
        pid_i_term += (error * dt);
        float d_term = (error - last_error) / dt;
        last_error = error;
        
        float motor_output = (KP * error) + (KI * pid_i_term) + (KD * d_term);
        
        // 5. Apply to hardware PWM (Signal to ESCs)
        Update_Motor_PWM(motor_output);
        
        HAL_Delay(4); // Run loop at ~250Hz
    }
}
