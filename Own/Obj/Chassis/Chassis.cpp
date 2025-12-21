//
// Created by lhy on 2024/9/9.
//



#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

#ifdef __cplusplus
}
#endif

#include "Chassis.hpp"
#include "MyMath/MyMath.hpp"
#include "MicroTime/MicroTime.hpp"

#if USING_CHASSIS == 1


using namespace CHASSIS_N;

float Chassis::PowerCalc(float _power, const float _vel, const bool _positive)
{
    if (const float delta = k2 * k2 * _vel * _vel - 4.f * k1 * (k3 * _vel * _vel + c - _power); delta < 1e-6f)
        _power = -k2 * _vel / (2.f * k1);
    else if (_positive)_power = (-k2 * _vel + sqrtf(delta)) / (2.f * k1);  // using VSQRT.F32 instruction
    else _power = (-k2 * _vel - sqrtf(delta)) / (2.f * k1);
		if (_power > 16000) return 16000;
		if (_power < -16000) return -16000;
		return _power;
}

void Chassis::send_foc(const float _buffer_left, const float _power_limit, const float _target_buffer)
{
    float power[6] {};
    float init_power_sum = 0.f;
    float error[6] {};
    float error_sum = 0.f;
    float power_max = _power_limit;
    float current[6] {base.left_front.output(), base.right_front.output(), base.left_rear.output(),
        base.right_rear.output(), extend.left.output(), extend.right.output()};  // mind the IDs
    float speed_cur[6] {base.left_front.GetRawSpeed(), base.right_front.GetRawSpeed(), base.left_rear.GetRawSpeed(),
        base.right_rear.GetRawSpeed(), extend.left.GetRawSpeed(), extend.right.GetRawSpeed()};
    for (uint8_t i = 0; i < 6; ++i) {
        power[i] = k1 * current[i] * current[i] + k2 * current[i] * speed_cur[i] + k3 * speed_cur[i] * speed_cur[i] + c;
        error[i] = wheelSpeed[i] > speed_cur[i] ? wheelSpeed[i] - speed_cur[i] : speed_cur[i] - wheelSpeed[i];
        init_power_sum += power[i] > 0.f? power[i] : 0.f;
        error_sum += error[i];
    }
    if (init_power_sum > power_max) {
        if (error_sum > err_sum_max) {
            for (uint8_t i = 0; i < 6; i++)
            {
                if (power[i] > 1e-6f)
                {
                    power[i] = power_max * error[i] / error_sum;
                    current[i] = PowerCalc(power[i], speed_cur[i], wheelSpeed[i] > speed_cur[i]);
                }
            }
        }
        else if (error_sum < err_sum_min) {
            for (uint8_t i = 0; i < 6; i++)
            {
                if (power[i] > 1e-6f)
                {
                    power[i] = power_max * power[i] / init_power_sum;
                    current[i] = PowerCalc(power[i], speed_cur[i], wheelSpeed[i] > speed_cur[i]);
                }
            }
        }
        else {
            const float confidence = (error_sum - err_sum_min) / (err_sum_max - err_sum_min);
            for (uint8_t i = 0; i < 6; i++)
            {
                if (power[i] > 1e-6f)
                {
                    power[i] = power_max * (confidence * error[i] / error_sum + (1.f - confidence) * power[i] / init_power_sum);
                    current[i] = PowerCalc(power[i], speed_cur[i], wheelSpeed[i] > speed_cur[i]);
                }
            }
        }
    }
    can->transmit(M3508::foc.TX_LOW_ID, current[0], current[1], current[3], current[2]);
    can->transmit(M3508::foc.TX_HIGH_ID, current[4], current[5], 0, 0);
/*
    can->transmit(M3508::foc.TX_LOW_ID, base.left_front.output(), base.right_front.output(), base.right_rear.output(),
                  base.left_rear.output());
    can->transmit(M3508::foc.TX_HIGH_ID, (int16_t) extend.left.output(), (int16_t) extend.right.output(), 0, 0);
*/
}

void Chassis::UpdatePid() {
    // base.left_front.motor.set_speed(base.left_front.feed_back.data.speed);
    base.left_front.set_speed(wheelSpeed[chassis_dep::LeftFront]);
    base.right_front.set_speed(wheelSpeed[chassis_dep::RightFront]);
    base.left_rear.set_speed(wheelSpeed[chassis_dep::LeftRear]);
    base.right_rear.set_speed(wheelSpeed[chassis_dep::RightRear]);
    extend.left.set_speed(wheelSpeed[chassis_dep::ExtendLeft]);
    extend.right.set_speed(wheelSpeed[chassis_dep::ExtendRight]);
}

void Chassis::UpdateMotor() {
    uint16_t id = can->read_header()->Identifier - M3508::foc.RX_ID;
    if (base.left_front.get_feedback(id, can->read())) { return; }
    if (base.right_front.get_feedback(id, can->read())) { return; }
    if (base.left_rear.get_feedback(id, can->read())) { return; }
    if (base.right_rear.get_feedback(id, can->read())) { return; }
    if (extend.left.get_feedback(id, can->read())) { return; }
    if (extend.right.get_feedback(id, can->read())) { return; }
}

void Chassis::update_slope(interact_dep::chassis_mode mode) {
    move.xSlope.update();
    move.ySlope.update();

    //    float gimbalAngleSin = arm_sin_f32(relative_angle * d2r);
    float gimbalAngleSin = 0;
    //    float gimbalAngleCos = arm_cos_f32(relative_angle * d2r);
    float gimbalAngleCos = 1;
    move.vx = move.xSlope.get() * gimbalAngleCos + move.ySlope.get() * gimbalAngleSin;
    move.vy = -move.xSlope.get() * gimbalAngleSin + move.ySlope.get() * gimbalAngleCos;
    move.w = move.wSlope.update() + w1Speed + w2Speed;
    move.extend = move.extendSlope.update();
    load_speed(mode);
}

void Chassis::load_speed(interact_dep::chassis_mode mode) {
    using namespace chassis_dep;
    float rotateRatio[4];

    rotateRatio[LeftFront] = (front_info.wheel_base + front_info.wheel_track) / 2.f
                             - info.offset_y * my_abs(move.w) / max.w + info.offset_x * my_abs(move.w) / max.w;
    rotateRatio[RightFront] = (front_info.wheel_base + front_info.wheel_track) / 2.f
                              - info.offset_y * my_abs(move.w) / max.w - info.offset_x * my_abs(move.w) / max.w;
    rotateRatio[LeftRear] = (rear_info.wheel_base + rear_info.wheel_track) / 2.f
                            + info.offset_y * my_abs(move.w) / max.w + info.offset_x * my_abs(move.w) / max.w;
    rotateRatio[RightRear] = (rear_info.wheel_base + rear_info.wheel_track) / 2.f
                             + info.offset_y * my_abs(move.w) / max.w - info.offset_x * my_abs(move.w) / max.w;

    wheelSpeed[LeftFront] = (move.vx + move.vy - rotateRatio[LeftFront] * move.w) * v2rpm;
    wheelSpeed[RightFront] = (move.vx - move.vy - rotateRatio[RightFront] * move.w) * v2rpm;
    wheelSpeed[LeftRear] = (-move.vx + (move.vy) - rotateRatio[LeftRear] * move.w) * v2rpm;
    wheelSpeed[RightRear] = (-move.vx - (move.vy) - rotateRatio[RightRear] * move.w) * v2rpm;

    switch (mode) {
        case interact_dep::chassis_mode::CLIMB:
            wheelSpeed[ExtendLeft] = (move.extend) * v2rpm;
            wheelSpeed[ExtendRight] = -(move.extend) * v2rpm;
            break;
        default:
            wheelSpeed[ExtendLeft] = (move.vy + move.extend) * v2rpm;
            wheelSpeed[ExtendRight] = -(move.vy + move.extend) * v2rpm;
            break;
    }
}

#endif
