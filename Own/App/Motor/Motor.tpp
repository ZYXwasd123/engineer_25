//
// Created by liaohy on 24-11-25.
//
#pragma once
#include "Detect/Detect.hpp"
#include "Count/Count.hpp"
#include <concepts>

namespace motor_const {
    constexpr float M2006Gain  = 36;
    constexpr float GM6020Gain = 1;
    constexpr float M3508Gain  = 3591.f / 187;
} // namespace motor_const

struct FOC {
    uint16_t RX_ID;
    uint16_t TX_LOW_ID;
    uint16_t TX_HIGH_ID;
};


template<typename T>
concept motor_param = requires(T t, uint8_t* data) {
    typename T::Feedback;

    t.feedback.raw_data.speed;
    t.feedback.raw_data.position;
    t.feedback.raw_data.current;

    t.feedback.data.speed;
    t.feedback.data.position;
    t.feedback.data.current;

    t.detect;
    t.rx_id;
    t.get_feedback(data);
};

template<typename T>
concept motor_control = requires(T t, uint8_t* data) {
    requires requires { t.set_position(0.f); } || requires { t.set_speed(0.f); } || requires { t.set_current(0.f); };
};

struct DefaultFeedback {
    struct {
        int16_t speed;
        int16_t position;
        int16_t current;
        int16_t temperature;
    } raw_data;

    struct {
        float speed;
        float position;
        float current;
        float temperature;
        float last_position;
    } data;

    float total_position;
};



template<uint32_t precision_range, uint32_t reduction_head, uint32_t reduction_src>
class default_motor {
public:
    using Feedback = DefaultFeedback;

public:
    Feedback feedback;
    uint16_t rx_id;
    Detect detect;

    explicit default_motor(const uint16_t rx_id)
        : feedback()
        , rx_id(rx_id)
        , detect(1000) {}
    constexpr static float reduction_ratio = static_cast<float>(reduction_head) / reduction_src;
    void get_feedback(const uint8_t* data);
private:
    Count rx_cnt{};
};

template<uint32_t precision_range, uint32_t reduction_head, uint32_t reduction_src>  // 声明三个非类型模板参数（编译期常量）
void default_motor<precision_range, reduction_head, reduction_src>::get_feedback(const uint8_t* data) {  // 类外定义重复声明
    feedback.raw_data.position    = static_cast<int16_t>(((data[0] << 8) | data[1]));
    feedback.raw_data.speed       = static_cast<int16_t>((data[2] << 8) | data[3]);
    feedback.raw_data.current     = static_cast<int16_t>((data[4] << 8) | data[5]);
    feedback.raw_data.temperature = (data[6]);

    feedback.data.position    = static_cast<float>(feedback.raw_data.position) * 360.0f / precision_range;
    feedback.data.speed       = feedback.raw_data.speed;
    feedback.data.current     = feedback.raw_data.current;
    feedback.data.temperature = feedback.raw_data.temperature;

    float dPos = feedback.data.position - feedback.data.last_position;
    if (dPos > 180) {
        dPos = dPos - 360;
    } else if (dPos < -180) {
        dPos = dPos + 360;
    }

    feedback.total_position     += (dPos / reduction_ratio);
    feedback.data.last_position  = feedback.data.position;
    ++rx_cnt;
    detect.update();
}
