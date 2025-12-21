//
// Created by Administrator on 25-1-21.
//
#include "CppTask.hpp"
#include "Interact/Interact.hpp"
#include "OneStepGet/OneStepGet.hpp"
#include "RoboArm/RoboArm.hpp"
#include "Judge/ui.hpp"

int32_t fps = 0;
std::atomic<bool> ui_reset(false);
UI::ui_item UI::figure_list[30];
uint8_t UI::figure_index = 0;

UI::ui_item UI::str_list[30];
uint8_t UI::str_index = 0;
// uint32_t cnt_change;
// uint32_t cnt_trans;
void JudgeTask()
	{
    while (!ui.is_get_id.load()) {
        osDelay(1);
    }
    auto int_valve_tx = []() -> int32_t {
        auto valve_tx_s = interact.sub_board.read_tx_status();
        int32_t result  = 0;
        result          = result * 10 + (0x01);
        for (int i = 3; i >= 0; --i) { result = result * 10 + ((valve_tx_s >> i) & 0x01); }
        return result;
    };
    auto int_robo_arm = []() -> int32_t {
        if (interact.robo_arm.mode == interact_dep::robo_mode::CUSTOM) { return 1; }
        return 0;
    };
    auto int_chassis = []() -> int32_t {
        if (interact.chassis.mode == interact_dep::chassis_mode::NONE) { return 0; }
        return 1;
    };
    auto int_auto = []() -> int32_t {
        if (OneStepGetControl::AUTO == OSG::mode) { return 1; }
        if (OneStepGetControl::MANUAL == OSG::mode) { return 0; }
        if (OneStepGetControl::ROBO_ARM == OSG::mode) { return 2; }
        return 0;
    };
    while (1) {
        uint32_t time = 0;
        ui.reset();
        osDelay(35);

        UI::create_str("101", UI::operation::ADD, UI::layer::LAYER_5, UI::color::PINK, 2, 24, 838, 24, 5, "air:", 40);
        UI::create_str("102", UI::operation::ADD, UI::layer::LAYER_5, UI::color::CYAN, 2, 24, 686, 24, 9,
                       "chassis:", 40);
        UI::create_str("103", UI::operation::ADD, UI::layer::LAYER_5, UI::color::YELLOW, 2, 24, 595, 24, 9,
                       "roboarm:", 40);
        UI::create_str("104", UI::operation::ADD, UI::layer::LAYER_5, UI::color::PINK, 2, 24, 900, 24, 6, "auto:", 40);

        UI::create_str("105", UI::operation::ADD, UI::layer::LAYER_5, UI::color::GREEN, 1, 858, 845, 10, 4, "lf:", 40);
        UI::create_str("106", UI::operation::ADD, UI::layer::LAYER_5, UI::color::GREEN, 1, 858, 790, 10, 4, "lb:", 40);
        UI::create_str("107", UI::operation::ADD, UI::layer::LAYER_5, UI::color::GREEN, 1, 1022, 845, 10, 4, "rf:", 40);
        UI::create_str("108", UI::operation::ADD, UI::layer::LAYER_5, UI::color::GREEN, 1, 1022, 790, 10, 4, "rb:", 40);
        UI::create_str("109", UI::operation::ADD, UI::layer::LAYER_5, UI::color::GREEN, 1, 930, 880, 10, 6,
                       "main:", 40);

        UI::create_str("110", UI::operation::ADD, UI::layer::LAYER_5, UI::color::PURPLE, 2, 1570, 870, 24, 8,
                       "joint1:", 40);
        UI::create_str("111", UI::operation::ADD, UI::layer::LAYER_5, UI::color::PURPLE, 2, 1570, 820, 24, 8,
                       "joint2:", 40);
        UI::create_str("112", UI::operation::ADD, UI::layer::LAYER_5, UI::color::PURPLE, 2, 1570, 770, 24, 8 ,
                       "joint3:", 40);
        UI::create_str("113", UI::operation::ADD, UI::layer::LAYER_5, UI::color::PURPLE, 2, 1570, 720, 24, 8 ,
                       "joint4:", 40);
        UI::create_str("114", UI::operation::ADD, UI::layer::LAYER_5, UI::color::PURPLE, 2, 1570, 670, 24, 8,
                       "joint5:", 40);
        UI::create_str("115", UI::operation::ADD, UI::layer::LAYER_5, UI::color::PURPLE, 2, 1570, 620, 24, 8,
                       "joint6:", 40);
//        UI::create_str("116", UI::operation::ADD, UI::layer::LAYER_5, UI::color::PINK, 2, 24, 750, 24, 4, "err:", 40);

        // for (int i = 0; i < UI::str_index; ++i) {
        //     ui.update();
        //     osDelay(34);
        // }

        const auto air_num =
            UI::create_int("201", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PINK, 2, 221, 838, 24, int_valve_tx(), 1);
        const auto chassis_num =
            UI::create_int("202", UI::operation::ADD, UI::layer::LAYER_6, UI::color::CYAN, 2, 221, 686, 24, int_chassis(), 1);
        const auto robo_num =
            UI::create_int("203", UI::operation::ADD, UI::layer::LAYER_6, UI::color::YELLOW, 2, 221, 595, 24, int_robo_arm(), 1);
        const auto auto_num =
            UI::create_int("204", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PINK, 2, 221, 900, 24, int_auto(), 1);

        const auto lf_num   = UI::create_float("205", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 1,
                                               858 + 50, 845, 10, 5, 2);
        const auto lb_num   = UI::create_float("206", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 1,
                                               858 + 50, 790, 10, 5, 2);
        const auto rf_num   = UI::create_float("207", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 1,
                                               1022 + 50, 845, 10, 5, 2);
        const auto rb_num   = UI::create_float("208", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 1,
                                               1022 + 50, 790, 10, 5, 2);
        const auto main_num = UI::create_float("209", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 1,
                                               930 + 70, 880, 10, 5, 2);
        auto reset_arc = UI::create_arc("210", UI::operation::ADD, UI::layer::LAYER_6, UI::color::GREEN, 10, 955, 518,
                                        270, 90, 80, 60, 40);

        const auto joint1_num = UI::create_float("211", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 2,
                                               1750, 870, 24, 5, 3);
        const auto joint2_num = UI::create_float("212", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 2,
                                               1750, 820, 24, 5, 3);
        const auto joint3_num = UI::create_float("213", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 2,
                                               1750, 770, 24, 5, 3);
        const auto joint4_num = UI::create_float("214", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 2,
                                               1750, 720, 24, 5, 3);
        const auto joint5_num = UI::create_float("215", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 2,
                                               1750, 670, 24, 5, 3);
        const auto joint6_num = UI::create_float("216", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 2,
                                               1750, 620, 24, 5, 3);

        // const auto gold_left = UI::create_ellipse("217", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 6,
        //                                        493, 667, 70, 60, 40);
        // const auto gold_right = UI::create_ellipse("218", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 6,
        //                                        1283, 755, 60, 50, 40);


        const auto gold_left_line1 = UI::create_line("219", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 10,
                                               431, 293, 707, 319, 40);

        const auto gold_left_line2 = UI::create_line("220", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 10,
                                               691, 564, 372, 562, 40);

        const auto gold_right_line2 = UI::create_line("221", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 10,
                                               1148, 570, 1450, 569, 40);

        const auto gold_right_line1 = UI::create_line("222", UI::operation::ADD, UI::layer::LAYER_6, UI::color::PURPLE, 10,
                                               1295, 382, 1130, 378, 40);

        const auto gold_left_line3 = UI::create_line("223", UI::operation::ADD, UI::layer::LAYER_6, UI::color::ORANGE, 10,
                                               296, 369, 675, 363, 40);

        const auto gold_left_line4 = UI::create_line("224", UI::operation::ADD, UI::layer::LAYER_6, UI::color::ORANGE, 10,
                                               357, 513, 669, 572, 40);

        const auto gold_right_line3 = UI::create_line("225", UI::operation::ADD, UI::layer::LAYER_6, UI::color::ORANGE, 10,
                                               1142, 673, 1370, 669, 40);

        const auto gold_right_line4 = UI::create_line("226", UI::operation::ADD, UI::layer::LAYER_6, UI::color::ORANGE, 10,
                                               1210, 370, 1594, 388, 40);



        while (ui.update()) {
            osDelay(50);
        }

        auto joint_lost = [](UI::float_data* self, bool is_lost) {
            if (is_lost) {
                self->set_color(UI::color::BLACK);
            } else {
                self->set_color(UI::color::PURPLE);
            }
        };

        for (int i = 0; i < UI::figure_index; ++i) { UI::figure_list[i].set_operate(UI::operation::MODIFY); }

        while (1) {
            ++time;
            if (time % 35 == 0) {
                ++fps;
                for (int i = 0; i < UI::figure_index; ++i) {
                    if (fps % UI::figure_list[i].control.display_interval == 0) {
                        ++UI::figure_list[i].control.display_num;
                    }
                }
                lf_num->set_float(interact.sub_board.custom_frame_rx.s.valve1);
//                lb_num->set_float(interact.sub_board.custom_frame_rx.s.valve4);
                rf_num->set_float(interact.sub_board.custom_frame_rx.s.valve3);
//                rb_num->set_float(interact.sub_board.custom_frame_rx.s.valve2);
                main_num->set_float(interact.sub_board.custom_frame_rx.s.valve2);

                chassis_num->set_int(int_chassis());
                robo_num->set_int(int_robo_arm());
                auto_num->set_int(int_auto());
                air_num->set_int(int_valve_tx());
                if (interact.sub_board.custom_frame_tx.s.pump == 1) {
                    air_num->set_color(UI::color::PINK);
                } else {
                    air_num->set_color(UI::color::YELLOW);
                }
                // if (air_num->control.visible) {
                    // cnt_change++;
                // }
                air_num->control.visible = true;
                joint_lost(joint1_num, roboArm.joint1.detect.isLost);
                if (roboArm.joint2.internal.detect.isLost == false && roboArm.joint2.external.detect.isLost == false) {
                    joint2_num->set_color(UI::color::PURPLE);
                } else if (roboArm.joint2.internal.detect.isLost == false) {
                    joint2_num->set_color(UI::color::GREEN);
                } else if (roboArm.joint2.external.detect.isLost == false) {
                    joint2_num->set_color(UI::color::YELLOW);
                } else {
                    joint2_num->set_color(UI::color::BLACK);
                }
                joint_lost(joint3_num, roboArm.joint3.detect.isLost);
                joint_lost(joint4_num, roboArm.joint4.detect.isLost);
                joint_lost(joint5_num, roboArm.joint5.detect.isLost);
                joint_lost(joint6_num, roboArm.joint6.detect.isLost);
                joint1_num->set_float(roboArm.relative_pos[0]);
                joint2_num->set_float(roboArm.relative_pos[1]);
                joint3_num->set_float(roboArm.relative_pos[2]);
                joint4_num->set_float(roboArm.relative_pos[3]);
                joint5_num->set_float(roboArm.relative_pos[4]);
                joint6_num->set_float(roboArm.relative_pos[5]);

                ui.update();
                // if (air_num->control.visible == false) {
                //     ++cnt_trans;
                // }
            }

            if (time % 1000 == 0) { fps = 0; }
            osDelay(1);
            if (ui_reset.load()) { break; }
        }
    }
}
