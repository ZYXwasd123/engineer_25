//
// Created by liaohy on 24-11-16.
//
#include "CppTask.hpp"

#include "Imu/Imu.hpp"
#include "W25Q64/W25Q64.hpp"
#include "ThreadConfig.h"
#define TEMP_CTRL_ENABLE 0
static uint8_t imu_flag = 0;
static uint32_t count = 0;
static uint32_t correct_times = 0;
static uint32_t temp_ticks = 0;

void IMUTask() {
    imu.init();
    imu.update_raw_data();
    imu.mahony_start();
    imu.update_offset(++correct_times, imu_flag);
    while (1) {
        if (count % 10 == 0) {
#if TEMP_CTRL_ENABLE
            imu.update_temp_ctrl();
#elif TEMP_CTRL_ENABLE == 0
            if (imu_flag == 0)
                imu_flag = 1;
#endif
            if (imu_flag == 0 && imu.is_temp_dead_condition(0.5)) {
                imu_flag = imu.is_temp_const(++temp_ticks);
            } else if (!imu.is_temp_dead_condition(0.5)) {
                temp_ticks = 0;
            }
        }
        ++count;
        IMUHeapCnt = uxTaskGetStackHighWaterMark(NULL);
        osDelay(1);
        if (count % 1 == 0) {
            imu.update_raw_data();
            switch (imu_flag) {
                case 1:
                    switch (imu.offset.restart_measure) {
                    case IMU_MEASURE::MEASURE_DISABLE:
                            // osMutexWait(flash_mutex_id, osWaitForever);
                            w25q64.read_buffer(IMU_OFFSET_ADDRESS, (uint8_t *) &imu.offset.correct, 16);
                            // osMutexRelease(flash_mutex_id);
                            imu_flag = 2;
                            break;
                    case IMU_MEASURE::MEASURE_ENABLE:
                            imu.update_offset(++correct_times, imu_flag);
                            if (imu_flag == 2) {
                                // osMutexWait(flash_mutex_id, osWaitForever);
                                w25q64.erase_sector_4(IMU_OFFSET_ADDRESS);
                                w25q64.write_buffer(IMU_OFFSET_ADDRESS, (uint8_t *) &imu.offset.correct, 16);
                                // osMutexRelease(flash_mutex_id);
                            }
                            break;
                    }
                    break;
                case 2:
                    imu.update();
                    xEventGroupSetBits(osEventGroup, IMU_EVENT);
                    break;
                default:
                    break;
            }
        }
    }
}
