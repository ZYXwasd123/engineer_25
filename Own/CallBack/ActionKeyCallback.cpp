//
// Created by Administrator on 25-5-19.
//

#include "Interact/Interact.hpp"
#include "Chassis/Chassis.hpp"
#include "OneStepGet/OneStepGet.hpp"

extern interact_dep::ActionsGroup arm_get_silver_group;
extern interact_dep::ActionsGroup put_mine_group;
extern interact_dep::ActionsGroup get_second_silver_group;
extern interact_dep::ActionsGroup get_gold_group;
extern interact_dep::ActionsGroup get_left_gold_group;
extern interact_dep::ActionsGroup get_right_gold_group;
extern interact_dep::ActionsGroup put_down_gold_group;
extern interact_dep::ActionsGroup put_down_silver_group;
extern interact_dep::ActionsGroup get_silver_group;
extern interact_dep::ActionsGroup put_silver_group;
extern interact_dep::Actions arm_error;

extern interact_dep::Actions arm_get_gold;
extern interact_dep::Actions arm_get_gold_z;
extern interact_dep::Actions exchange_left;
extern interact_dep::Actions exchange_right;
extern interact_dep::Actions reset2;
extern interact_dep::Actions reset1;
extern interact_dep::Actions get_silver_mine;


void action_z_callback(KeyEventType event)
{
    if (interact.robo_arm.mode != interact_dep::robo_mode::CUSTOM)
    {
        if (OSG::mode == OneStepGetControl::MANUAL)  // 左侧缩回
        {
            switch (event)
            {
                case KeyEvent_OnDown:
                case KeyEvent_OnLongPress:
                case KeyEvent_OnPressing:
                    one_step_gets.Xleft.axis.Tdecrease();
                    break;
                case KeyEvent_None:
                case KeyEvent_OnUp:
                    break;
                default:
                    break;
            }
        }
        else if (OSG::mode == OneStepGetControl::AUTO)  // 取金矿
        {
            switch (event)
            {
                case KeyEvent_OnClick:
                    get_gold_group.reset();
                    interact.actions_group = &get_gold_group;
                    interact.robo_arm.mode = interact_dep::robo_mode::ACTIONS_GROUP;
                    break;
                default:
                    break;
            }
        }
        else if (OSG::mode == OneStepGetControl::ROBO_ARM)  // 啥都不干
        {
            //switch (event)
            //{
                //case KeyEvent_OnClick:
                    // arm_get_gold_group.reset();
                    // interact.actions_group = &arm_get_gold_group;
                    // interact.robo_arm.mode = interact_dep::robo_mode::ACTIONS_GROUP;
                    //break;
                //default:
                    //break;
            //}
        }
    }
    else  // 自定义控制器下右转
    {
        switch (event)
        {
            case KeyEvent_OnDown:
            case KeyEvent_OnLongPress:
            case KeyEvent_OnPressing:
                chassis.w2Speed = -1;
                break;
            case KeyEvent_None:
            case KeyEvent_OnUp:
                chassis.w2Speed = 0;
                break;
            default:
                break;
        }
    }
}


void action_shift_z_callback(KeyEventType event) {
    if (OSG::mode == OneStepGetControl::MANUAL) {
        switch (event) {
            case KeyEvent_OnDown:
            case KeyEvent_OnLongPress:
            case KeyEvent_OnPressing:
                one_step_gets.Xleft.axis.Tincrease();
                break;
            case KeyEvent_None:
            case KeyEvent_OnUp:
                break;
            default:
                break;
        }
    }
    else if (OSG::mode == OneStepGetControl::AUTO)  // 取左侧金矿
    {
        switch (event)
        {
            case KeyEvent_OnClick:
                interact.set_action_group(get_left_gold_group);
                break;
            default:
                break;
        }
    }
    else if (OSG::mode == OneStepGetControl::ROBO_ARM)
    {
        switch (event)
        {
            case KeyEvent_OnClick:
                interact.set_action(arm_get_gold);
                break;
            default:
                break;
        }
    }
}

void action_ctrl_z_callback(KeyEventType event)
{
    if (OSG::mode == OneStepGetControl::MANUAL)  // 左侧开闭
    {
        switch (event)
        {
            case KeyEvent_OnClick:
                interact.sub_board.toggle_lf_valve();
                break;
            default:
                break;
        }
    }
    else if (OSG::mode == OneStepGetControl::AUTO)  // 取右侧金矿
    {
        switch (event)
        {
            case KeyEvent_OnClick:
                interact.set_action_group(get_right_gold_group);
                break;
            default:
                break;
        }
    }
    else if (OSG::mode == OneStepGetControl::ROBO_ARM)
    {
        switch (event) {
            case KeyEvent_OnClick:
                interact.set_action(arm_get_gold_z);
                break;
            default:
                break;
        }
    }
}


void action_x_callback(KeyEventType event) {
    if (interact.robo_arm.mode != interact_dep::robo_mode::CUSTOM) {
        if (OSG::mode == OneStepGetControl::MANUAL) {
            switch (event) {
                case KeyEvent_OnDown:
                case KeyEvent_OnLongPress:
                case KeyEvent_OnPressing:
                    one_step_gets.Yleft.axis.Tincrease();
                    break;
                case KeyEvent_None:
                case KeyEvent_OnUp:
                    break;
                default:
                    break;
            }
        } else if (OSG::mode == OneStepGetControl::AUTO) {
            switch (event) {
                case KeyEvent_OnClick:
                    interact.set_action_group(put_down_gold_group);
                    break;
                default:
                    break;
            }
        } else if (OSG::mode == OneStepGetControl::ROBO_ARM) {
            // switch (event) {
            //     case KeyEvent_OnClick:
            //         get_silver_group.reset();
            //         interact.actions_group = &get_silver_group;
            //         interact.robo_arm.mode = interact_dep::robo_mode::ACTIONS_GROUP;
            //
            //         break;
            //     default: break;
            // }
        }
    } else {
        switch (event) {
            case KeyEvent_OnDown:
            case KeyEvent_OnLongPress:
            case KeyEvent_OnPressing:
                chassis.w1Speed = 1;
                break;
            case KeyEvent_None:
            case KeyEvent_OnUp:
                chassis.w1Speed = 0;
                break;
            default:
                break;
        }
    }
}

void action_shift_x_callback(KeyEventType event) {
    if (OSG::mode == OneStepGetControl::MANUAL) {
        switch (event) {
            case KeyEvent_OnDown:
            case KeyEvent_OnLongPress:
            case KeyEvent_OnPressing:
                one_step_gets.Yleft.axis.Tdecrease();
                break;
            case KeyEvent_None:
            case KeyEvent_OnUp:
                break;
            default:
                break;
        }
    } else if (OSG::mode == OneStepGetControl::AUTO) {

    } else if (OSG::mode == OneStepGetControl::ROBO_ARM) {
        switch (event) {
            case KeyEvent_OnClick:
                // get_silver_from_left_group.reset();
                // interact.actions_group = &get_silver_from_left_group;
                // interact.robo_arm.mode = interact_dep::robo_mode::ACTIONS_GROUP;

                break;
            default:
                break;
        }
    }
}

void action_ctrl_x_callback(KeyEventType event) {
    if (OSG::mode == OneStepGetControl::MANUAL) {
        switch (event) {
            case KeyEvent_OnClick:
                interact.sub_board.toggle_lb_valve();
                break;
            default:
                break;
        }
    } else if (OSG::mode == OneStepGetControl::AUTO) {
    } else if (OSG::mode == OneStepGetControl::ROBO_ARM) {
    }
}


void action_c_callback(KeyEventType event) {
    if (interact.robo_arm.mode != interact_dep::robo_mode::CUSTOM) {
        if (OSG::mode == OneStepGetControl::MANUAL) {
            switch (event) {
                case KeyEvent_OnDown:
                case KeyEvent_OnLongPress:
                case KeyEvent_OnPressing:
                    one_step_gets.Yright.axis.Tincrease();
                    break;
                case KeyEvent_None:
                case KeyEvent_OnUp:
                    break;
                default:
                    break;
            }
        }
        else if (OSG::mode == OneStepGetControl::AUTO)  // 左侧取银（没用）
        {
            switch (event) {
                case KeyEvent_OnClick:
                    interact.set_action_group(get_silver_group);
                    break;
                default:
                    break;
            }
        } else if (OSG::mode == OneStepGetControl::ROBO_ARM) {
            switch (event) {
                case KeyEvent_OnClick:
                    interact.set_action_group(arm_get_silver_group);
                    break;
                default:
                    break;
            }
        }
    } else {
        switch (event) {
            case KeyEvent_OnClick:
                interact.sub_board.toggle_lf_valve();
                break;
            default:
                break;
        }
    }
}

void action_shift_c_callback(KeyEventType event) {
    if (OSG::mode == OneStepGetControl::MANUAL) {
        switch (event) {
            case KeyEvent_OnDown:
            case KeyEvent_OnLongPress:
            case KeyEvent_OnPressing:
                one_step_gets.Yright.axis.Tdecrease();
                break;
            case KeyEvent_None:
            case KeyEvent_OnUp:
                break;
            default:
                break;
        }
    } else if (OSG::mode == OneStepGetControl::AUTO) {
        switch (event) {
            case KeyEvent_OnClick:
                interact.set_action_group(get_second_silver_group);
                break;
            default:
                break;
        }
    } else if (OSG::mode == OneStepGetControl::ROBO_ARM) {

    }
}

void action_ctrl_c_callback(KeyEventType event) {
    if (OSG::mode == OneStepGetControl::MANUAL) {
        switch (event) {
            case KeyEvent_OnClick:
                interact.sub_board.toggle_rb_valve();
                break;
            default:
                break;
        }
    } else if (OSG::mode == OneStepGetControl::AUTO) {
        switch (event) {
            case KeyEvent_OnClick:
                interact.set_action_group(get_second_silver_group);
                break;
            default:
                break;
        }

    } else if (OSG::mode == OneStepGetControl::ROBO_ARM) {
    }
}

void action_v_callback(KeyEventType event) {
    if (interact.robo_arm.mode != interact_dep::robo_mode::CUSTOM) {

        if (OSG::mode == OneStepGetControl::MANUAL) {
            switch (event) {
                case KeyEvent_OnDown:
                case KeyEvent_OnLongPress:
                case KeyEvent_OnPressing:
                    one_step_gets.Xright.axis.Tdecrease();
                    break;
                case KeyEvent_None:
                case KeyEvent_OnUp:
                    break;
                default:
                    break;
            }
        }
        else if (OSG::mode == OneStepGetControl::AUTO)  // 右侧存矿
        {
            switch (event) {
                case KeyEvent_OnClick:
                    interact.set_action_group(put_mine_group);
                    break;
                default:
                    break;
            }
        } else if (OSG::mode == OneStepGetControl::ROBO_ARM) {
            switch (event) {
                case KeyEvent_OnDown:
                case KeyEvent_OnLongPress:
                case KeyEvent_OnPressing:
                    interact.set_action_group(put_mine_group);
                    break;
                default:
                    break;
            }
        }
    } else {
        switch (event) {
            case KeyEvent_OnClick:
                interact.sub_board.toggle_rf_valve();
                break;
            default:
                break;
        }
    }
}

void action_shift_v_callback(KeyEventType event)
{
    if (OSG::mode == OneStepGetControl::MANUAL) {
        switch (event) {
            case KeyEvent_OnDown:
            case KeyEvent_OnLongPress:
            case KeyEvent_OnPressing:
                one_step_gets.Xright.axis.Tincrease();
                break;
            case KeyEvent_None:
            case KeyEvent_OnUp:
                break;
            default:
                break;
        }
    } else if (OSG::mode == OneStepGetControl::AUTO) {

    } else if (OSG::mode == OneStepGetControl::ROBO_ARM) {

    }
}

void action_ctrl_v_callback(KeyEventType event)
{
    if (OSG::mode == OneStepGetControl::MANUAL) {
        switch (event) {
            case KeyEvent_OnClick:
                interact.sub_board.toggle_rf_valve();
                break;
            default:
                break;
        }
    } else if (OSG::mode == OneStepGetControl::AUTO) {

    } else if (OSG::mode == OneStepGetControl::ROBO_ARM) {

    }
}

void action_b_callback(KeyEventType event)  // 切换模式
{
    switch (event)
        {
        case KeyEvent_OnClick:
            if (OSG::mode == OneStepGetControl::AUTO)
            {
                OSG::mode = OneStepGetControl::MANUAL;
            }
            else if (OSG::mode == OneStepGetControl::MANUAL)
            {
                OSG::mode = OneStepGetControl::ROBO_ARM;
            }
            else if (OSG::mode == OneStepGetControl::ROBO_ARM)
            {
                OSG::mode = OneStepGetControl::AUTO;
            }
            break;
        default:
            break;
    }
}

void action_ctrl_r_callback(KeyEventType event)  // 复位
{
    switch (event) {
        case KeyEvent_OnClick:
            one_step_gets.Xleft.set_state(translation::state::RESET, 0);
            one_step_gets.Xright.set_state(translation::state::RESET, 0);
            one_step_gets.Yleft.set_state(translation::state::RESET, 0);
            one_step_gets.Yright.set_state(translation::state::RESET, 0);
            one_step_gets.rotate_move.set_state(translation::state::RESET, 0);
            one_step_gets.rotate.set_target(osg::rota_init);
            one_step_gets.rotate.is_unlock = false;
            break;
        default:
            break;
    }
}


void action_shift_f_callback(KeyEventType event)  // 左侧兑矿准备
{
    switch (event)
    {
        case KeyEvent_OnDown:
        case KeyEvent_OnLongPress:
        case KeyEvent_OnPressing:
            interact.set_action(exchange_left);
            break;
        default:
            break;
    }
}

void action_ctrl_f_callback(KeyEventType event)  // 右侧兑矿准备
{
    switch (event)
    {
        case KeyEvent_OnDown:
        case KeyEvent_OnLongPress:
        case KeyEvent_OnPressing:
            interact.set_action(exchange_right);
            break;
        default:
            break;
    }

}

void action_e_callback(KeyEventType event)  // 向下看
{
    switch (event)
    {
        case KeyEvent_OnDown:
        case KeyEvent_OnLongPress:
        case KeyEvent_OnPressing:
            interact.set_action(reset2);
            break;
        default:
            break;
    }
}

void action_f_callback(KeyEventType event)  // 取银矿
{
    switch (event)
    {
        case KeyEvent_OnClick:
            interact.set_action_group(arm_get_silver_group);
            break;
        default:
            break;
    }
}

void action_shift_r_callback(KeyEventType event)  // 大臂回正
{
    switch (event)
    {
        case KeyEvent_OnDown:
        case KeyEvent_OnLongPress:
        case KeyEvent_OnPressing:
            interact.set_action(reset1);
        default:
            break;
    }
}

void roboarm_shift_e_callback(KeyEventType event) {
    switch (event) {
        case KeyEvent_OnClick:
//            interact.set_action(arm_error);
            break;
        default: break;
    }
}


