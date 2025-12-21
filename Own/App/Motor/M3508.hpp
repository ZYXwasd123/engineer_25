//
// Created by lhy on 2024/9/9.
//
#pragma once

#include "Control/SpeedPidControl.hpp"
#include "Control/PosPidControl.hpp"
#include "Param/M3508.hpp"

#define USING_M3508 1
#if USING_M3508 == 1

class M3508Speed : public SpeedPidControl<M3508> {
public:
    template<typename... Args>
    explicit M3508Speed(Args&&... args)
        : SpeedPidControl(std::forward<Args>(args)...) {};

    float GetRawSpeed() const {
        return this->feedback.raw_data.speed;
    }
    bool get_feedback(const uint16_t id, const uint8_t* data) {
        if (id == this->rx_id) {
            SpeedPidControl::get_feedback(data);
            return true;
        }
        return false;
    };

    bool is_block(int16_t max_current) {
        if (fabs(feedback.raw_data.current) > max_current ) {
            return true;
        }
        return false;
    }
};

class M3508Pos : public PosPidControl<M3508> {
public:
    template<typename... Args>
    explicit M3508Pos(Args&&... args)
        : PosPidControl(std::forward<Args>(args)...) {};

    float& total_position() {
        return feedback.total_position;
    }

    bool get_feedback(const uint16_t id, const uint8_t* data) {
        if (id == this->rx_id) {
            PosPidControl::get_feedback(data);
            return true;
        }
        return false;
    };

    bool is_block(int16_t min_current, int16_t max_current) {
        if (feedback.raw_data.current < min_current || feedback.raw_data.current > max_current) {
            return true;
        }
        return false;
    }
    void clear() {
        this->position.clear();
        this->speed.clear();
    }
};

#endif //USING_M3508
