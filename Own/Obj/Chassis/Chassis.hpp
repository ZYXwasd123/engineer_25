//
// Created by lhy on 2024/9/9.
//
#pragma once

#define USING_CHASSIS 1
#if USING_CHASSIS == 1

#include "Interact/Interact_dep.hpp"
#include "chassis_dep.hpp"

namespace CHASSIS_N
{
    constexpr float power_limit = 120.0f;
    constexpr float target_buffer = 30.0f;
    constexpr float k1 = 1.74e-7f;
    constexpr float k2 = 2.4324e-6f;  // toque_coefficient, ((20/16384)*(0.3)*(187/3591)/9.55)
    constexpr float k3 = 1.534e-7f;
    constexpr float c = 1.0f;
    constexpr float err_sum_min = 5.f;
    constexpr float err_sum_max = 10000.f;
    inline auto Clamp = [](auto value, auto min, auto max)
    {
        if (value > max) return max;
        if (value < min) return min;
        return value;
    };
}

class Chassis {
public:
    Chassis(SuperCan* can, const std::array<Slope, 4>& move_cfg, const std::array<chassis_dep::ChassisMotorCfg, 4>& base_cfg,
            const std::array<chassis_dep::ChassisMotorCfg, 2>& extend_cfg)
        : can(can)
        , base(base_cfg)
        , extend(extend_cfg)
        , move(move_cfg)
        , key() {}

    SuperCan* can;
    void send_foc(float _buffer_left, float _power_limit, float _target_buffer);
    void UpdateMotor();
    void UpdatePid();
    void update_slope(interact_dep::chassis_mode mode);
    float PowerCalc(float _power, float _vel, bool _positive);
    float wheelSpeed[6] {};
    chassis_dep::base_motor base;
    chassis_dep::extend_motor extend;
    float w1Speed = 0;
    float w2Speed = 0;

private:
    chassis_dep::move move;
    chassis_dep::key key;

    void load_speed(interact_dep::chassis_mode mode);

    friend class Interact;
    friend void chassis_w_callback(KeyEventType event);
    friend void chassis_a_callback(KeyEventType event);
    friend void chassis_s_callback(KeyEventType event);
    friend void chassis_d_callback(KeyEventType event);
    friend void chassis_q_callback(KeyEventType event);
    friend void chassis_shift_w_callback(KeyEventType event);
    friend void chassis_shift_a_callback(KeyEventType event);
    friend void chassis_shift_s_callback(KeyEventType event);
    friend void chassis_shift_d_callback(KeyEventType event);
    friend void chassis_shift_q_callback(KeyEventType event);

    friend void chassis_ctrl_w_callback(KeyEventType event);
    friend void chassis_ctrl_a_callback(KeyEventType event);
    friend void chassis_ctrl_s_callback(KeyEventType event);
    friend void chassis_ctrl_d_callback(KeyEventType event);
};

extern Chassis chassis;

#endif
