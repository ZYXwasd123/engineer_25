//
// Created by liaohy on 24-11-15.
//

#include "Buzzer/Buzzer.hpp"
#include "CAN/SuperCan.hpp"
#include "Chassis/Chassis.hpp"
#include "CppTask.hpp"
#include "Detect/Detect.hpp"
#include "Interact/Interact.hpp"
#include "Judge/ui.hpp"
#include "Pump/Pump.hpp"
#include "RGBLED/RGBLED.hpp"
#include "RemoteControl/RemoteControl.hpp"
#include "RoboArm/RoboArm.hpp"
#include "ThreadConfig.h"
#include "W25Q64/W25Q64.hpp"
#include <DWT/SuperDWT.hpp>

void air_left_callback(KeyEventType event);
void air_right_callback(KeyEventType event);

void roboarm_shift_e_callback(KeyEventType event);
void robo_arm_shift_q_callback(KeyEventType event);
void robo_arm_ctrl_q_callback(KeyEventType event);
void action_shift_f_callback(KeyEventType event);
void robo_arm_r_callback(KeyEventType event);
void action_shift_r_callback(KeyEventType event);

void chassis_w_callback(KeyEventType event);
void chassis_a_callback(KeyEventType event);
void chassis_s_callback(KeyEventType event);
void chassis_d_callback(KeyEventType event);
void chassis_q_callback(KeyEventType event);

#ifdef __cplusplus
extern "C" {
#endif
extern osThreadId ERROR_TASKHandle;
#include "usb_device.h"
#ifdef __cplusplus
}
#endif
#include "WDG/SuperIWDG.hpp"
#include "OneStepGet/OneStepGet.hpp"

volatile uint32_t ada = 0;
void action_z_callback(KeyEventType event);
void action_shift_z_callback(KeyEventType event);
void action_x_callback(KeyEventType event);
void action_shift_v_callback(KeyEventType event);
void action_v_callback(KeyEventType event);
void action_shift_c_callback(KeyEventType event);
void action_c_callback(KeyEventType event);
void action_shift_x_callback(KeyEventType event);

void action_ctrl_z_callback(KeyEventType event);
void action_ctrl_x_callback(KeyEventType event);
void action_ctrl_c_callback(KeyEventType event);
void action_ctrl_v_callback(KeyEventType event);

void robo_arm_ctrl_e_callback(KeyEventType event);
void action_ctrl_r_callback(KeyEventType event);

extern interact_dep::Actions reset1;
extern interact_dep::Actions reset2;

extern interact_dep::Actions exchange_left;
extern interact_dep::Actions exchange_right;

extern interact_dep::Actions arm_get_gold;
extern interact_dep::Actions arm_get_gold_z;

extern interact_dep::ActionsGroup arm_get_silver_group;
extern interact_dep::ActionsGroup get_gold_group;
extern interact_dep::ActionsGroup get_left_gold_group;
extern interact_dep::ActionsGroup get_right_gold_group;

extern interact_dep::ActionsGroup put_down_silver_group;
extern interact_dep::ActionsGroup put_down_gold_group;

void chassis_shift_w_callback(KeyEventType event);
void chassis_shift_a_callback(KeyEventType event);
void chassis_shift_s_callback(KeyEventType event);
void chassis_shift_d_callback(KeyEventType event);
void action_b_callback(KeyEventType event);
void robo_arm_ctrl_g_callback(KeyEventType event);
void action_ctrl_f_callback(KeyEventType event);
void action_e_callback(KeyEventType event);
void action_f_callback(KeyEventType event);
void robo_arm_g_callback(KeyEventType event);
void robo_arm_shift_g_callback(KeyEventType event);

void chassis_motor_detect() {
    buzzer.PushMusic<8>(Buzzer::error_music);
}

void joint1_motor_detect() {
    buzzer.PushMusic<8>(Buzzer::error_music);

}

void joint2_internal_motor_detect() {
    buzzer.PushMusic<8>(Buzzer::error_music);

}
void joint2_external_motor_detect() {
    buzzer.PushMusic<8>(Buzzer::error_music);

}
void joint3_motor_detect() {
    buzzer.PushMusic<8>(Buzzer::error_music);

}
void joint4_motor_detect() {
    buzzer.PushMusic<8>(Buzzer::error_music);

}void joint5_motor_detect() {
    buzzer.PushMusic<8>(Buzzer::error_music);

}void joint6_motor_detect() {
    buzzer.PushMusic<8>(Buzzer::error_music);

}
void osg_detect() {
    buzzer.PushMusic<8>(Buzzer::error_music);
}
std::atomic<bool> rc_ready(false);

void reset_err_ctrl_b_callback(KeyEventType event);
void chassis_ctrl_w_callback(KeyEventType event);
void chassis_ctrl_a_callback(KeyEventType event);
void chassis_ctrl_s_callback(KeyEventType event);
void chassis_ctrl_d_callback(KeyEventType event);
void StartTask() {
    ada = SuperDWT::get_tick();
    /* USB初始化 */
    MX_USB_DEVICE_Init();

    /* 关闭两个24V  打开5V*/
    power_24v_right.WriteDown();
    power_24v_left.WriteDown();
    power_5v.WriteUp();

    chassis.base.left_front.detect_lost(chassis_motor_detect);
    chassis.base.right_front.detect_lost(chassis_motor_detect);
    chassis.base.left_rear.detect_lost(chassis_motor_detect);
    chassis.base.right_rear.detect_lost(chassis_motor_detect);
    chassis.extend.left.detect_lost(chassis_motor_detect);
    chassis.extend.right.detect_lost(chassis_motor_detect);

    roboArm.joint1.detect_lost(joint1_motor_detect);
    roboArm.joint2.internal.detect_lost(joint2_internal_motor_detect);
    roboArm.joint2.external.detect_lost(joint2_external_motor_detect);
    roboArm.joint3.detect_lost(joint3_motor_detect);
    roboArm.joint4.detect_lost(joint4_motor_detect);
    roboArm.joint5.detect_lost(joint5_motor_detect);
    roboArm.joint6.detect_lost(joint6_motor_detect);
    one_step_gets.rotate_move.Motor.detect_lost(osg_detect);

    SuperIWDG::GotInstance().give();

    /* W25Q64初始化 */
    w25q64.init();
    /* CAN初始化 */
    canPlus1.filter_config(1);
    canPlus2.filter_config(1);
    canPlus3.filter_config(2);
    canPlus1.start();
    canPlus2.start();
    canPlus3.start();
    SuperIWDG::GotInstance().give();
    /* 遥控器和键鼠初始化 */
    // 键鼠默认配置uint32_t clickTime = 75, uint32_t longPressTime = 1000
    interact.remote_control.detect.lostFun = &remote_ctrl_recover;
    KeyBoardRegister(interact.keyList, Key_W, CombineKey_None, chassis_w_callback);
    KeyBoardRegister(interact.keyList, Key_A, CombineKey_None, chassis_a_callback);
    KeyBoardRegister(interact.keyList, Key_S, CombineKey_None, chassis_s_callback);
    KeyBoardRegister(interact.keyList, Key_D, CombineKey_None, chassis_d_callback);
    KeyBoardRegister(interact.keyList, Key_W, CombineKey_Shift, chassis_shift_w_callback);
    KeyBoardRegister(interact.keyList, Key_A, CombineKey_Shift, chassis_shift_a_callback);
    KeyBoardRegister(interact.keyList, Key_S, CombineKey_Shift, chassis_shift_s_callback);
    KeyBoardRegister(interact.keyList, Key_D, CombineKey_Shift, chassis_shift_d_callback);
    KeyBoardRegister(interact.keyList, Key_W, CombineKey_Ctrl, chassis_ctrl_w_callback);
    KeyBoardRegister(interact.keyList, Key_A, CombineKey_Ctrl, chassis_ctrl_a_callback);
    KeyBoardRegister(interact.keyList, Key_S, CombineKey_Ctrl, chassis_ctrl_s_callback);
    KeyBoardRegister(interact.keyList, Key_D, CombineKey_Ctrl, chassis_ctrl_d_callback);
    KeyBoardRegister(interact.keyList, Key_Q, CombineKey_None, chassis_q_callback);

    KeyBoardRegister(interact.keyList, Key_Q, CombineKey_Shift, robo_arm_shift_q_callback);
    KeyBoardRegister(interact.keyList, Key_Q, CombineKey_Ctrl, robo_arm_ctrl_q_callback);

    KeyBoardRegister(interact.keyList, Key_Left, CombineKey_None, air_left_callback);
    interact.keyList[16].longPressTime = 100;
    KeyBoardRegister(interact.keyList, Key_Right, CombineKey_None, air_right_callback);
    interact.keyList[16].longPressTime = 5000;

    interact.keyList[7].longPressTime = 5000;
    KeyBoardRegister(interact.keyList, Key_R, CombineKey_None, robo_arm_r_callback);
    KeyBoardRegister(interact.keyList, Key_R, CombineKey_Shift, action_shift_r_callback);
    KeyBoardRegister(interact.keyList, Key_R, CombineKey_Ctrl, action_ctrl_r_callback);
    KeyBoardRegister(interact.keyList, Key_F, CombineKey_Shift, action_shift_f_callback);
    KeyBoardRegister(interact.keyList, Key_F, CombineKey_Ctrl, action_ctrl_f_callback);
    KeyBoardRegister(interact.keyList, Key_E, CombineKey_None, action_e_callback);
    KeyBoardRegister(interact.keyList, Key_F, CombineKey_None, action_f_callback);
    KeyBoardRegister(interact.keyList, Key_E, CombineKey_Ctrl, robo_arm_ctrl_e_callback);
    KeyBoardRegister(interact.keyList, Key_E, CombineKey_Shift, roboarm_shift_e_callback);

    KeyBoardRegister(interact.keyList, Key_Z, CombineKey_None, action_z_callback);
    KeyBoardRegister(interact.keyList, Key_Z, CombineKey_Shift, action_shift_z_callback);
    KeyBoardRegister(interact.keyList, Key_X, CombineKey_None, action_x_callback);
    KeyBoardRegister(interact.keyList, Key_X, CombineKey_Shift, action_shift_x_callback);
    KeyBoardRegister(interact.keyList, Key_C, CombineKey_None, action_c_callback);
    KeyBoardRegister(interact.keyList, Key_C, CombineKey_Shift, action_shift_c_callback);
    KeyBoardRegister(interact.keyList, Key_V, CombineKey_None, action_v_callback);
    KeyBoardRegister(interact.keyList, Key_V, CombineKey_Shift, action_shift_v_callback);

    KeyBoardRegister(interact.keyList, Key_Z, CombineKey_Ctrl, action_ctrl_z_callback);
    KeyBoardRegister(interact.keyList, Key_X, CombineKey_Ctrl, action_ctrl_x_callback);
    KeyBoardRegister(interact.keyList, Key_C, CombineKey_Ctrl, action_ctrl_c_callback);
    KeyBoardRegister(interact.keyList, Key_V, CombineKey_Ctrl, action_ctrl_v_callback);

    KeyBoardRegister(interact.keyList, Key_B, CombineKey_None, action_b_callback);
    KeyBoardRegister(interact.keyList, Key_B, CombineKey_Ctrl, reset_err_ctrl_b_callback);
    KeyBoardRegister(interact.keyList, Key_G, CombineKey_Ctrl, robo_arm_ctrl_g_callback);
    KeyBoardRegister(interact.keyList, Key_G, CombineKey_None, robo_arm_g_callback);
    KeyBoardRegister(interact.keyList, Key_G, CombineKey_Shift, robo_arm_shift_g_callback);


    interact.remote_control.start();
    interact.image_trans.start_receive();
    interact.sub_board.start_receive();
    roboArm.joint1.start();

    ui.start_receive();

    /* 蜂鸣器初始化 */
    buzzer.Start();
    ada = SuperDWT::get_tick() - ada;
    while (1) {
        xEventGroupWaitBits(osEventGroup, REMOTE_CONTROL_START_EVENT, pdTRUE, pdTRUE, 1) & REMOTE_CONTROL_START_EVENT;
        if (rc_ready.load()) {
            break;
        }
        interact.remote_control.detect.JudgeLost();
    }


    xEventGroupSetBits(osEventGroup, START_END_EVENT);
    StartHeapCnt = uxTaskGetStackHighWaterMark(NULL);
    vTaskDelete(NULL);
}
