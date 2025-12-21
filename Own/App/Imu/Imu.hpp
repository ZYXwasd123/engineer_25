//
// Created by liaohy on 24-11-17.
//
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

#ifdef __cplusplus
}
#endif

#include "Pid/Pid.hpp"
enum class IMU_MEASURE {
    MEASURE_ENABLE,
    MEASURE_DISABLE,
};
class Imu {

public:
    Imu(IMU_MEASURE is_able = IMU_MEASURE::MEASURE_DISABLE): temprature_pid(0, 0, 0, 0, 0) {
        offset.restart_measure = is_able;
    };

    float roll;
    float pitch;
    float yaw;
    float temprature;

    /* 初始化 */
    void init();

    void mahony_start();

    void update();

    void update_raw_data();

    void update_offset(uint32_t cnt, uint8_t &flag);

    void update_temp_ctrl();

    uint8_t is_temp_const(uint32_t ticks);

    uint8_t is_temp_dead_condition(float err);

    struct {
        bool on;
        float correct[4];
        uint32_t target_times;
        IMU_MEASURE restart_measure;
    } offset;

private:
    void ekf_update();

    void mahony_update();

    float gyro[3];
    float accel[3];

    Pid temprature_pid;
    float temp_work;
    uint32_t temp_ticks;
    TIM_HandleTypeDef *pwm;
    uint32_t Channel;
};

extern Imu imu;
