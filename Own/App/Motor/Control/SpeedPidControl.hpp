//
// Created by Administrator on 2025/4/8.
//

#pragma once
#include "Pid/Pid.hpp"
#include "../Motor.tpp"

template<motor_param motor>
class SpeedPidControl : protected motor {
public:
    template<typename... Args>
    explicit SpeedPidControl(const Pid& cfg, Args&&... args)
        : motor(std::forward<Args>(args)...)
        , speed(cfg) {};


    float set_speed(float target) {
        return speed.update(target, this->feedback.data.speed);
    }

    [[nodiscard]] float output() const { return speed.output; }

protected:
    Pid speed;
};
