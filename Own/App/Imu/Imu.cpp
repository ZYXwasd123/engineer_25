//
// Created by liaohy on 24-11-17.
//

#ifdef __cplusplus
extern "C" {
#endif
#include "tim.h"
#include "adc.h"

#include "BMI088/inc/BMI088driver.h"
#include "Imu/Algorithm/QuaternionEKF.h"
#include "Mahony/MahonyAHRS.h"
#ifdef __cplusplus
}
#endif

#include <MyMath/MyMath.hpp>
#include "Imu.hpp"

extern float j4_offset_val;

float H723_Temperature;
void Imu::init() {
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f, 0); //ekf初始化
    // PID_init(&Temperature_PID, PID_POSITION,Temperature_PID_Para,2000,200); //加热pidlimit
    temprature_pid.set(200 , 0.05 , 35 , 200 , 2000);
    if (offset.restart_measure == IMU_MEASURE::MEASURE_ENABLE)
        temp_ticks = 5000;
    else
        temp_ticks = 300;
    temp_work = 40;
    offset.target_times = 10000;
    pwm = &htim3;
    Channel = TIM_CHANNEL_4;
    Mahony_Init(1000); //mahony姿态解算初始化
    // imu heat init
    HAL_TIM_PWM_Start(pwm, Channel);
    while (BMI088_init()) {
    }; //陀螺仪初始化
}

void Imu::mahony_start() {
    MahonyAHRSinit(accel[0], accel[1], accel[2], 0, 0, 0); //mahony上电快速初始化
}

void Imu::update() {
    gyro[0] -= offset.correct[0]; //减去陀螺仪0飘
    gyro[1] -= offset.correct[1];
    gyro[2] -= offset.correct[2];

#if cheat              //作弊 可以让yaw很稳定 去掉比较小的值
    if(fabsf(gyro[2])<0.003f)
        gyro[2]=0;
#endif
    //===========================================================================
    //ekf姿态解算部分
    imu.ekf_update();
    // IMU_QuaternionEKF_Update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2]);
    //===============================================================================

    //=================================================================================
    //mahony姿态解算部分
    imu.mahony_update();
    // Mahony_update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],0,0,0);
    // Mahony_computeAngles(); //角度计算
    //=============================================================================
    //ekf获取姿态角度函数
    pitch = Get_Pitch(); //获得pitch
    roll = Get_Roll(); //获得roll
    yaw = Get_Yaw(); //获得yaw
    //==============================================================================
}

void Imu::update_raw_data() {
    BMI088_read(gyro, accel, &temprature);
}

uint8_t Imu::is_temp_const(uint32_t ticks) {
    if (ticks > 10)
        return 1;
    return 0;
}

uint8_t Imu::is_temp_dead_condition(float err) {
    if (my_abs<float>(temprature - temp_work) < err)
        return 1;
    return 0;
}

void Imu::ekf_update() {
    IMU_QuaternionEKF_Update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void Imu::mahony_update() {
    Mahony_update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0, 0, 0);
    Mahony_computeAngles(); //角度计算
}

void Imu::update_offset(uint32_t cnt, uint8_t &flag) {
    offset.correct[0] += gyro[0];
    offset.correct[1] += gyro[1];
    offset.correct[2] += gyro[2];
    if (cnt >= offset.target_times) {
        offset.correct[0] /= offset.target_times;
        offset.correct[1] /= offset.target_times;
        offset.correct[2] /= offset.target_times;
        offset.correct[3] = j4_offset_val;
        flag = 2;
    }
}

void Imu::update_temp_ctrl() {
    temprature_pid.update(temp_work, temprature);
    HAL_ADC_Start(&hadc3);

    H723_Temperature = ((110.0 - 30.0) / (*(unsigned short *) (0x1FF1E840) - *(unsigned short *) (0x1FF1E820))) * (HAL_ADC_GetValue(&hadc3) - *(unsigned short *) (0x1FF1E820)) + 30;
    if (temprature_pid.output < 0)
        temprature_pid.output = 0;
    pwm->Instance->CCR4 = static_cast<uint32_t>(temprature_pid.output);
}
