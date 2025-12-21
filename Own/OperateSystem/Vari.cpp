//
// Created by Administrator on 24-10-3.
//
extern "C" {
#include "cmsis_os.h"
}
#include "Buzzer/Buzzer.hpp"
#include "CAN/SuperCan.hpp"
#include "CDC/SuperCDC.hpp"
#include "Chassis/Chassis.hpp"
#include "GPIO/SuperGPIO.hpp"
#include "Heap/CustomHeap.hpp"
#include "Interact/Interact.hpp"
#include "Judge/ui.hpp"
#include "OneStepGet/OneStepGet.hpp"
#include "RGBLED/RGBLED.hpp"
#include "RoboArm/RoboArm.hpp"
#include "W25Q64/W25Q64.hpp"
#include <Imu/Imu.hpp>

__attribute__((section(".DTCMRAM"))) uint64_t DTCMUsed[8 * 1024 / 8];
__attribute__((section(".RAM_D1"))) uint64_t D1Used[8 * 1024 / 8];
__attribute__((section(".RAM_D2"))) uint64_t D2Used[8 * 1024 / 8];
__attribute__((section(".RAM_D3"))) uint64_t D3Used[8 * 1024 / 8];

__attribute__((section(".DTCMRAM"))) CustomHeap DTCMHeap(DTCMUsed, sizeof(DTCMUsed));
__attribute__((section(".RAM_D1"))) CustomHeap D1Heap(D1Used, sizeof(D1Used));
__attribute__((section(".RAM_D2"))) CustomHeap D2Heap(D2Used, sizeof(D2Used));
__attribute__((section(".RAM_D3"))) CustomHeap D3Heap(D3Used, sizeof(D3Used));

osMutexId flash_mutex_id;

W25Q64 w25q64(&hospi2);

__weak SuperCan canPlus1(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
__weak SuperCan canPlus2(&hfdcan2, FDCAN_RX_FIFO0, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
__weak SuperCan canPlus3(&hfdcan3, FDCAN_RX_FIFO0, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

#if USING_LKMOTOR == 1
#endif

#if USING_DMMOTOR == 1
#endif

#if USING_GM6020 == 1
#endif

#if USING_M3508 == 1
#endif

#if USING_M2006 == 1
#endif

#if USING_CHASSIS == 1
#endif

Buzzer buzzer(&htim12, TIM_CHANNEL_2);

Chassis chassis(&canPlus2, {Slope(15, 1), Slope(15, 1), Slope(0.1, 0), Slope(15, 1)}, chassis_dep::base_motor_default,
                chassis_dep::extend_motor_default);

float j4_offset_val = 65.0f;  // 翎控丢零点，重设后存进flash，每次上电时读取
uint16_t uartlk_param[9] = {};

// joint3的offset是不会变的，因为joint3是没有经过180°的，joint1也是一样
RoboArm roboArm(&canPlus1,&huart1, 5, 65536, 10, 1, 65536, 6, 2, 65536, 6, 3, 65536, 6, 4, 65536, 10,
                1, Pid(400, 0.002, 0.8, 500, 1200, 0), Pid(0.25, 0.010, 0.00, 300, 500, 1.0),
                7, 65536, 10,
                6, 65536, 10,
                {88.961792, -45.0833359 + 360 - 102.278336 + 5, -45.0833359 + 37.5383339 + 5, 135 + 27.9533329,
                 206.0f, /*(310.715 - 360)*/61, 0});

__attribute__((section(".RAM_D3"))) RGBLED Led(&hspi6);

Imu imu(IMU_MEASURE::MEASURE_DISABLE);

Interact interact(0xFF, 0xFE, &huart5, &huart10, &huart3);

SuperGPIO power_5v(GPIOC, GPIO_PIN_15);
SuperGPIO power_24v_right(GPIOC, GPIO_PIN_14);
SuperGPIO power_24v_left(GPIOC, GPIO_PIN_13);

UI ui(102, 0x0166, &huart7);
uint16_t power_buffer = 0;

OneStepGetControl OSG::mode   = OneStepGetControl::AUTO;
OSG one_step_gets(Pid(100, 0.0000, 20, 500, 9000, 0.0), Pid(1.5, 0, 2.3, 4000, 7000, 1),
                  4, Slope(1.3, 0), false,4000,-4000,
                  Pid(20, 0, 4, 8000, 16000, 1.0), Pid(20, 0, 4, 8000, 16000, 1.0),
                  2, Slope {0.6, 0}, true,4000,-4000,
                  Pid(100, 0.0000, 20, 500, 9000, 0.0), Pid(1.5, 0.00, 2.3, 4000, 7000, 1),
                  3, Slope(1.3, 0), true,4000,-4000,
                  Pid(20, 0, 4, 8000, 16000, 1.0), Pid(20, 0, 4, 8000, 16000, 1.0),
                  1, Slope(0.6, 0), false,4000,-4000,
                  Pid(100, 0.0000, 20, 500, 9000, 0.0), Pid(1.5, 0.00, 2.3, 4000, 7000, 1),
                  6, Slope(1.3, 0), false,4000,-4000,
                  &canPlus3,3
);
// 349.146 0 -131.812
//interact_dep::Actions get_silver_mine({0,33.6004066,114.50135,0,31.870,0}, {480, 720, 360, 360});
interact_dep::Actions get_silver_mine({0,25.666613159627129519036695734921, 114.03037145218561331967584563847,0,180 - 114.03037145218561331967584563847 - 25.666613159627129519036695734921,0}, {180, 360, 480, 360},1000);
interact_dep::Actions get_silver_mine_lz(std::array<float,3>{349.146f,0.f,-131.812},std::array<float,3>{0.f,180.f,0.f},std::array<float,3>{0.06f,0.06f,0.2f});
interact_dep::Actions get_silver_mine_z(std::array<float,3>{349.146f,0.f,-131.812+250},std::array<float,3>{0.f,180.f,0.f},std::array<float,3>{0.06f,0.06f,0.16f});
interact_dep::Actions put_silver_mine(std::array<float,3>{295.f,-300.5f,126.5},std::array<float,3>{0.f,180.f,0.f},std::array<float,3>{0.15f,0.2f,0.1f});
interact_dep::Actions put_silver_mine_back(std::array<float,3>{230.f, -300.5f, 126.5}, std::array<float,3>{0.f, 190.f, 0.f}, std::array<float,3>{0.1f, 0.2f, 0.1f});
interact_dep::Actions put_silver_mine_up(std::array<float,3>{235.f,-300.5f,200.0},std::array<float,3>{0.f,190.f,0.f},std::array<float,3>{0.1f,0.2f,0.1f});

interact_dep::Actions put_mine({0, 0, 103, 0, 76, 0},{360, 360, 360, 360},1000);

interact_dep::Actions exchange_left({-17.9960938, 36.7366142, 35.3361511, -89.4694138, -89.9465207, 52.6248474});
interact_dep::Actions exchange_right({17.9960938,36.7366142,35.3361511,89.4694138,-89.9465207,-52.6248474});

interact_dep::Actions reset1({0, -55, 145, 0, 0, 0}, {720,720,360,720}, 1000);
interact_dep::Actions reset2({0, -8.31188679, 135, 0, -90, 0}, {720, 720, 360, 720}, 1000);

// 661.090, 0, 23.632
interact_dep::Actions arm_get_gold({0, 59.222788179042829536805234328593, 52.172861259904178421522373683569, 0, -21.395649438947007958327608012162, 0}, {480, 720, 900, 720},2000);
interact_dep::Actions arm_get_gold_z(std::array<float,3>{700, 0, 45+80},std::array<float,3>{0.f,90.f,0.f},1000);

interact_dep::Actions arm_error({-25.3774509, 4.23235941, 98.7619476, 0, 77.0056915, -25.3774529}, {480, 720, 900, 720});


// **************************************************************************************************** //
// OK

std::array<interact_dep::Actions, 7> get_second_silver_action        = {
        get_silver_mine,
        get_silver_mine_lz,
        get_silver_mine_z,
        put_silver_mine,
        put_silver_mine_back,
        put_silver_mine_up,
        reset1
};
// std::array<interact_dep::Actions, 2> get_second_silver_action        = {
//     interact_dep::Actions({0, 37.604, 115.54184, 0, 27.570, 0},{480,720,360,360}),
//     interact_dep::Actions(Slope(0.4, 0.15, 310), interact_dep::action_status::CartesianZ_z)
// };
std::array<uint32_t, 7> get_second_silver_time                       = {1500,1000,2000,1300,1500,500,500};
std::array<interact_dep::ActionsGroup::exe, 8> get_second_silver_exe = {
    []() {
        interact.sub_board.set_pump(1);
        interact.sub_board.set_main_valve(1);
    },
    nullptr,
    nullptr,
    []() {
        interact.sub_board.set_rf_valve(1);
    },
    []() {
        one_step_gets.Xright.set_state(translation::state::MOVE,210);
    },
    []() {
        interact.sub_board.set_main_valve(0);
    },
    []() {

    },
    []() { interact.robo_arm.mode = interact_dep::robo_mode::NONE; }
};
std::array<interact_dep::ActionsGroup::event, 7> get_second_silver_event = {
    []()->bool {
        return interact.sub_board.custom_frame_rx.s.valve3 < 250;
    },
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};
interact_dep::ActionsGroup get_second_silver_group = {.actions_list = get_second_silver_action.data(),
                                                      .time_list    = get_second_silver_time.data(),
//                                                      .event_list   = get_second_silver_event.data(),
                                                      .event_list   = nullptr,
                                                      .exe_list     = get_second_silver_exe.data(),
                                                      .len          = 7,
                                                      .index        = 0,
                                                      .time_cnt     = 0};// **************************************************************************************************** //
// OK

std::array<interact_dep::Actions, 2> reset_err_action        = {
        interact_dep::Actions{},interact_dep::Actions{}
};
std::array<uint32_t, 2> reset_err_time                       = {1000,50};
bool is_error = false;
std::array<interact_dep::ActionsGroup::exe, 3> reset_err_exe = {
    []() {
        interact.sub_board.set_reset_err(1);
        is_error = false;
    },
    []() {
        interact.sub_board.set_reset_err(1);
        is_error = false;
    },
    []() {
        interact.sub_board.set_reset_err(0);
        roboArm.joint4.close_flag = 0;
        roboArm.joint4.start_flag = 0;
        roboArm.joint4.offset_flag = 0;
        roboArm.joint5.close_flag = 0;
        roboArm.joint5.start_flag = 0;
        roboArm.joint5.offset_flag = 0;
        roboArm.joint6.close_flag = 0;
        roboArm.joint6.start_flag = 0;
        roboArm.joint6.offset_flag = 0;
        is_error = true;
        interact.robo_arm.mode = interact_dep::robo_mode::NONE; }
};

interact_dep::ActionsGroup reset_err_group = {.actions_list = reset_err_action.data(),
                                                      .time_list    = reset_err_time.data(),
                                                      .event_list   = nullptr,
                                                      .exe_list     = reset_err_exe.data(),
                                                      .len          = 2,
                                                      .index        = 0,
                                                      .time_cnt     = 0};
// **************************************************************************************************** //
// OK

std::array<interact_dep::Actions, 5> put_mine_action        = {
        put_mine,
        put_silver_mine,
        put_silver_mine_back,
        put_silver_mine_up,
        reset1
};
// std::array<interact_dep::Actions, 2> get_second_silver_action        = {
//     interact_dep::Actions({0, 37.604, 115.54184, 0, 27.570, 0},{480,720,360,360}),
//     interact_dep::Actions(Slope(0.4, 0.15, 310), interact_dep::action_status::CartesianZ_z)
// };
std::array<uint32_t, 5> put_mine_time                       = {2000,1300,1500,500,500};
std::array<interact_dep::ActionsGroup::exe, 6> put_mine_exe = {
    []() {
        interact.sub_board.set_pump(1);
        interact.sub_board.set_main_valve(1);
    },
    []() {
        interact.sub_board.set_rf_valve(1);
    },
    []() {
        one_step_gets.Xright.set_state(translation::state::MOVE,210);
    },
    []() {interact.sub_board.set_main_valve(0);},
    []() {},
    []() { interact.robo_arm.mode = interact_dep::robo_mode::NONE; }
};
std::array<interact_dep::ActionsGroup::event, 5> put_mine_event = {
    []()->bool {
        return interact.sub_board.custom_frame_rx.s.valve3 < 250;
    },
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};
interact_dep::ActionsGroup put_mine_group = {.actions_list = put_mine_action.data(),
                                                      .time_list    = put_mine_time.data(),
//                                                      .event_list   = get_second_silver_event.data(),
                                                      .event_list   = nullptr,
                                                      .exe_list     = put_mine_exe.data(),
                                                      .len          = 5,
                                                      .index        = 0,
                                                      .time_cnt     = 0};
// **************************************************************************************************** //
// OK

std::array<interact_dep::Actions, 3> arm_get_silver_action        = {
        get_silver_mine,
        get_silver_mine_lz,
        get_silver_mine_z
};

std::array<uint32_t, 3> arm_get_silver_time                       = {1500, 1000, 2000};
std::array<interact_dep::ActionsGroup::exe, 4> arm_get_silver_exe = {
        []() {
            interact.sub_board.set_pump(1);
            interact.sub_board.set_main_valve(1);
        },
        nullptr,
        nullptr,
        []() { interact.robo_arm.mode = interact_dep::robo_mode::NONE; }
};
std::array<interact_dep::ActionsGroup::event, 3> arm_get_silver_event = {
        []()->bool {
            return interact.sub_board.custom_frame_rx.s.valve3 < 250;
        },
        nullptr,
        nullptr
};
interact_dep::ActionsGroup arm_get_silver_group = {.actions_list = arm_get_silver_action.data(),
        .time_list    = arm_get_silver_time.data(),
//                                                      .event_list   = arm_get_silver_event.data(),
        .event_list   = nullptr,
        .exe_list     = arm_get_silver_exe.data(),
        .len          = 3,
        .index        = 0,
        .time_cnt     = 0};

// **************************************************************************************************** //
// !!!

std::array<uint32_t, 1> put_down_silver_time = {100};
std::array<interact_dep::Actions, 1> put_down_silver_action        = {reset2};
std::array<interact_dep::ActionsGroup::exe, 2> put_down_silver_exe = {
    []() {
        interact.sub_board.set_pump(0);
        interact.sub_board.set_lf_valve(0);
        interact.sub_board.set_rf_valve(0);
//        one_step_gets.Xleft.set_state(translation::state::MOVE,1000);
//        one_step_gets.Xright.set_state(translation::state::MOVE,-1000);
    },
    []() { interact.robo_arm.mode = interact_dep::robo_mode::NONE; }};

interact_dep::ActionsGroup put_down_silver_group = {.actions_list = put_down_silver_action.data(),
                                                      .time_list    = put_down_silver_time.data(),
                                                      .event_list   = nullptr,
                                                      .exe_list     = put_down_silver_exe.data(),
                                                      .len          = 1,
                                                      .index        = 0,
                                                      .time_cnt     = 0};
// **************************************************************************************************** //
// !!!

std::array<uint32_t, 3> get_silver_time = {1500,1500,2000};
std::array<interact_dep::Actions, 3> get_silver_action        = {reset2,reset2,reset2};
std::array<interact_dep::ActionsGroup::exe, 4> get_silver_exe = {
    []() {
        interact.sub_board.set_pump(1);
        interact.sub_board.set_lf_valve(1);
        one_step_gets.rotate.set_target(osg::rota_up);
        one_step_gets.Yleft.set_state(translation::state::MOVE,osg::M3508::meter2deg(0));
//        one_step_gets.Xleft.set_state(translation::state::MOVE,osg::xl_max + 200);
        one_step_gets.Xleft.set_state(translation::state::MOVE,-osg::M2006::meter2deg(290));
    },
    []() {
//        one_step_gets.rotate_move.set_state(translation::state::MOVE,-646);
        one_step_gets.rotate_move.set_state(translation::state::MOVE,-osg::M2006::meter2deg(160));
        },
    []() {
        one_step_gets.rotate_move.set_state(translation::state::MOVE,-osg::M2006::meter2deg(0));
        one_step_gets.Yleft.set_state(translation::state::MOVE,osg::M3508::meter2deg(osg::yl_meter));
        },
    []() {
        one_step_gets.rotate.is_unlock = true;
        one_step_gets.rotate_move.set_state(translation::state::MOVE,-osg::M2006::meter2deg(0));
        one_step_gets.Xleft.set_state(translation::state::MOVE,-osg::M2006::meter2deg(190));
        interact.robo_arm.mode = interact_dep::robo_mode::NONE; }
};

interact_dep::ActionsGroup get_silver_group = {.actions_list = get_silver_action.data(),
                                                      .time_list    = get_silver_time.data(),
                                                      .event_list   = nullptr,
                                                      .exe_list     = get_silver_exe.data(),
                                                      .len          = 3,
                                                      .index        = 0,
                                                      .time_cnt     = 0};
// **************************************************************************************************** //
// !!!

std::array<uint32_t, 2> put_silver_time = {1000,1500};
std::array<interact_dep::Actions, 2> put_silver_action        = {reset2,reset2};
std::array<interact_dep::ActionsGroup::exe, 3> put_silver_exe = {
    []() {
        interact.sub_board.set_pump(1);
        interact.sub_board.set_lf_valve(1);
        one_step_gets.rotate_move.set_state(translation::state::MOVE,-osg::M2006::meter2deg(45));
    },
    []() {
        one_step_gets.rotate.set_target(osg::rota_init);
        one_step_gets.rotate.is_unlock = false;
        },
    []() {
        one_step_gets.Xleft.set_state(translation::state::MOVE,-osg::M2006::meter2deg(0));
        one_step_gets.Yleft.set_state(translation::state::MOVE,osg::M2006::meter2deg(0));
        interact.robo_arm.mode = interact_dep::robo_mode::NONE; }
};

interact_dep::ActionsGroup put_silver_group = {.actions_list = put_silver_action.data(),
                                                      .time_list    = put_silver_time.data(),
                                                      .event_list   = nullptr,
                                                      .exe_list     = put_silver_exe.data(),
                                                      .len          = 2,
                                                      .index        = 0,
                                                      .time_cnt     = 0};

// **************************************************************************************************** //
// OK

std::array<uint32_t, 1> put_down_gold_time = {100};
std::array<interact_dep::Actions, 1> put_down_gold_action        = {reset2};
std::array<interact_dep::ActionsGroup::exe, 2> put_down_gold_exe = {
    []() {
        interact.sub_board.set_pump(0);
        interact.sub_board.set_lf_valve(0);
        interact.sub_board.set_rf_valve(0);
//        one_step_gets.Xleft.set_state(translation::state::MOVE, 1000);
//        one_step_gets.Xright.set_state(translation::state::MOVE, -1000);
//        one_step_gets.Yleft.set_state(translation::state::MOVE, 1450);
//        one_step_gets.Yright.set_state(translation::state::MOVE, -1450);
    },
    []() { interact.robo_arm.mode = interact_dep::robo_mode::NONE; }};

interact_dep::ActionsGroup put_down_gold_group = {.actions_list = put_down_gold_action.data(),
                                                      .time_list    = put_down_gold_time.data(),
                                                      .event_list   = nullptr,
                                                      .exe_list     = put_down_gold_exe.data(),
                                                      .len          = 1,
                                                      .index        = 0,
                                                      .time_cnt     = 0};

// **************************************************************************************************** //


std::array<uint32_t, 4> get_gold_time = {10, 3000, 1500, 2000};
std::array<interact_dep::Actions, 4> get_gold_action        = {reset1, reset1, reset1, reset1};
std::array<interact_dep::ActionsGroup::exe, 5> get_gold_exe = {
    []() {
        interact.sub_board.set_pump(1);
        interact.sub_board.set_rf_valve(1);
        interact.sub_board.set_lf_valve(1);
        one_step_gets.rotate.set_target(osg::rota_init);
    },
    []() {
        one_step_gets.Xleft.set_state(translation::state::MOVE, osg::xl_max);
        one_step_gets.rotate_move.set_state(translation::state::MOVE, -osg::M2006::meter2deg(45));
//        one_step_gets.Yleft.set_state(translation::state::MOVE, 200);
        one_step_gets.Xright.set_state(translation::state::MOVE, osg::xr_max+20);
//        one_step_gets.Yright.set_state(translation::state::MOVE, -200);
    },
    []() {
        one_step_gets.Yleft.set_state(translation::state::MOVE, osg::yl_max); //255
        one_step_gets.Yright.set_state(translation::state::MOVE, osg::yr_max); //-345
    },
    []() {
        one_step_gets.Xleft.set_state(translation::state::MOVE, 0);
        one_step_gets.Xright.set_state(translation::state::MOVE, 0);
    },
    []() { interact.robo_arm.mode = interact_dep::robo_mode::NONE; }};

std::array<interact_dep::ActionsGroup::event, 4> get_gold_event = {
    []()-> bool {
//        return interact.sub_board.custom_frame_rx.s.valve5 < 250  && interact.sub_board.custom_frame_rx.s.valve1 < 250;
        return false;
    },
    nullptr,
    nullptr,
    nullptr};

interact_dep::ActionsGroup get_gold_group = {.actions_list = get_gold_action.data(),
                                                      .time_list    = get_gold_time.data(),
                                                      .event_list   = get_gold_event.data(),
                                                      .exe_list     = get_gold_exe.data(),
                                                      .len          = 4,
                                                      .index        = 0,
                                                      .time_cnt     = 0};

// **************************************************************************************************** //


std::array<uint32_t, 4> get_right_gold_time = {10, 3000, 2000, 4000};
std::array<interact_dep::Actions, 4> get_right_gold_action        = {
    reset1,
    reset1,
    reset1,
    reset1};
std::array<interact_dep::ActionsGroup::exe, 5> get_right_gold_exe = {
    []() {
        interact.sub_board.set_pump(1);
        interact.sub_board.set_rf_valve(1);
    },
    []() {
        one_step_gets.Xright.set_state(translation::state::MOVE, osg::xr_max+20);
//        one_step_gets.Yright.set_state(translation::state::MOVE, -200);
    },
    []() {
        one_step_gets.Yright.set_state(translation::state::MOVE, osg::yr_max); //-345
    },
    []() {
        one_step_gets.Xright.set_state(translation::state::MOVE, 0);
    },
    []() { interact.robo_arm.mode = interact_dep::robo_mode::NONE; }};

interact_dep::ActionsGroup get_right_gold_group = {.actions_list = get_right_gold_action.data(),
                                                      .time_list    = get_right_gold_time.data(),
                                                      .event_list   = nullptr,
                                                      .exe_list     = get_right_gold_exe.data(),
                                                      .len          = 4,
                                                      .index        = 0,
                                                      .time_cnt     = 0};

// **************************************************************************************************** //


std::array<uint32_t, 4> get_left_gold_time = {10, 3000, 2000, 4000};
std::array<interact_dep::Actions, 4> get_left_gold_action        = {reset1, reset1, reset1, reset1};
std::array<interact_dep::ActionsGroup::exe, 5> get_left_gold_exe = {
    []() {
        interact.sub_board.set_pump(1);
        interact.sub_board.set_lf_valve(1);
    },
    []() {
        one_step_gets.Xleft.set_state(translation::state::MOVE, osg::xl_max-20);
        one_step_gets.rotate_move.set_state(translation::state::MOVE, -osg::M2006::meter2deg(45));

//        one_step_gets.Yleft.set_state(translation::state::MOVE, 200);
    },
    []() {
        one_step_gets.Yleft.set_state(translation::state::MOVE, osg::yl_max); //255
    },
    []() {
        one_step_gets.Xleft.set_state(translation::state::MOVE, 0);
    },
    []() { interact.robo_arm.mode = interact_dep::robo_mode::NONE; }};

interact_dep::ActionsGroup get_left_gold_group = {.actions_list = get_left_gold_action.data(),
                                                      .time_list    = get_left_gold_time.data(),
                                                      .event_list   = nullptr,
                                                      .exe_list     = get_left_gold_exe.data(),
                                                      .len          = 4,
                                                      .index        = 0,
                                                      .time_cnt     = 0};


// **************************************************************************************************** //


