//
// Created by Administrator on 25-5-19.
//

#include "RoboArm/RoboArm.hpp"
#include "Interact/Interact.hpp"
#include "W25Q64/W25Q64.hpp"


extern interact_dep::ActionsGroup reset_err_group;


void air_left_callback(KeyEventType event)
{
    switch (event)
    {
        case KeyEvent_OnClick:
            interact.sub_board.toggle_main_valve();
            break;
        default:
            break;
    }
}

void air_right_callback(KeyEventType event)
{
    switch (event)
    {
        case KeyEvent_OnClick:
            interact.sub_board.toggle_pump();
            break;
        default:
            break;
    }
}

void robo_arm_ctrl_g_callback(KeyEventType event)
{
    switch (event)
    {
        case KeyEvent_OnClick:
            // interact.robo_arm.last_mode = interact_dep::robo_mode::NONE;
            // interact.robo_arm.mode = interact_dep::robo_mode::NONE;
            // interact.joint[5] = 0;
            // interact.joint[4] = 0;
            roboArm.reset_offset();
            break;
        default:
            break;
    }
}

void robo_arm_shift_g_callback(KeyEventType event)
{
    switch (event)
    {
        case KeyEvent_OnClick:
            roboArm.reset_offset();
            //w25q64.erase_sector_4(MOTOR_OFFSET_ADDRESS);
            //w25q64.write_buffer(MOTOR_OFFSET_ADDRESS, reinterpret_cast<uint8_t*>(&roboArm.offset.joint4), 4);
            break;
        default:
            break;
    }
            
}

void reset_err_ctrl_b_callback(KeyEventType event)  // 末端电机重置
{
    switch (event)
    {
        case KeyEvent_OnClick:
            interact.set_action_group(reset_err_group);
            break;
        default:
            break;
    }
}

void robo_arm_g_callback(KeyEventType event)  // 进出自定义控制器模式
{
    switch (event)
    {
        case KeyEvent_OnClick:
            if (interact.robo_arm.mode != interact_dep::robo_mode::CUSTOM)
            {
                interact.robo_arm.mode = interact_dep::robo_mode::CUSTOM;
            }
            else
            {
                interact.robo_arm.mode = interact_dep::robo_mode::NONE;
            }
            break;
        default:
            break;
    }
}

void robo_arm_shift_q_callback(KeyEventType event)  // 关闭全部气泵和电磁阀
{
    switch (event) {
        case KeyEvent_OnClick:
            interact.sub_board.set_pump(0);
            interact.sub_board.set_main_valve(0);
            interact.sub_board.set_lf_valve(0);
            interact.sub_board.set_rf_valve(0);
            break;
        default:
            break;
    }
}

void robo_arm_ctrl_q_callback(KeyEventType event)  // 进入视觉模式
{
    switch (event)
    {
        case KeyEvent_OnDown:
        case KeyEvent_OnLongPress:
        case KeyEvent_OnPressing:
        case KeyEvent_OnClick:
            if (interact.robo_arm.mode != interact_dep::robo_mode::VISION)
            {
                interact.robo_arm.mode = interact_dep::robo_mode::VISION;
            }
            break;
        default:
            break;
    }
}

extern std::atomic<bool> ui_reset;

void robo_arm_r_callback(KeyEventType event)  // 刷新UI
{
    switch (event)
    {
        case KeyEvent_OnDown:
        case KeyEvent_OnLongPress:
        case KeyEvent_OnPressing:
            ui_reset.store(true);
            break;
        default:
            ui_reset.store(false);
            break;
    }
}
