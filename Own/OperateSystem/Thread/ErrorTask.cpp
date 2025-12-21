//
// Created by liaohy on 24-11-15.
//
#include "CppTask.hpp"
#include "RGBLED/RGBLED.hpp"
#include "Buzzer/Buzzer.hpp"
#include "WDG/SuperIWDG.hpp"
#include "OneStepGet/OneStepGet.hpp"
#include "Interact/Interact.hpp"
#include "RoboArm/RoboArm.hpp"

void ErrorTask() {
    osThreadSuspend(ERROR_TASKHandle);  // 挂起自身，恢复后进入急停
    osThreadSuspendAll();
    if (ARM_INIT_TASKHandle != NULL) {
         vTaskDelete(ARM_INIT_TASKHandle);
     }
    osThreadResumeAll();
    uint32_t time = 0;
    uint8_t red = 0;
    while (1) {
        roboArm.close();
        buzzer.StartMusic<8, HAL_Delay>(Buzzer::error_music);
        canPlus1.transmit(M2006Diff::foc.TX_LOW_ID, 0, 0, 0, 0);
        canPlus2.transmit(M3508::foc.TX_LOW_ID, 0, 0, 0, 0);
        canPlus3.transmit(0x200, 0, 0, 0, 0);
        one_step_gets.rotate.unlock();
        if (time < 10) {
            interact.sub_board.set_reset_err(1);
        } else {
            interact.sub_board.set_reset_err(0);
        }
        interact.sub_board.set_pump(0);
        interact.sub_board.set_rf_valve(0);
        interact.sub_board.set_rb_valve(0);
        interact.sub_board.set_main_valve(0);
        interact.sub_board.set_lb_valve(0);
        interact.sub_board.set_lf_valve(0);
        interact.sub_board.set_chassis_valve(0);
        interact.sub_board.transmit();
        Led.SetColor(red * 255, 0, 0);
        ++time;
        if (time % 3 == 0) { red = 1 - red; }

        interact.remote_control.detect.JudgeLost();

        if (interact.remote_control.rcInfo.right != static_cast<uint8_t>(RemoteControl::lever::lower)
            || interact.remote_control.rcInfo.left != static_cast<uint8_t>(RemoteControl::lever::lower)) {
            __set_FAULTMASK(1);
            HAL_NVIC_SystemReset();
        }
        SuperIWDG::GotInstance().give();
        HAL_Delay(30);
        ErrorHeapCnt = uxTaskGetStackHighWaterMark(NULL);
    }
}
