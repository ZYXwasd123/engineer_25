//
// Created by Administrator on 25-2-15.
//
#pragma once

#include "stm32h7xx_hal.h"

class SuperDWT {
public:
    static SuperDWT& GetInstance() {
        static SuperDWT instance;
        return instance;
    }

    [[nodiscard]] static  uint32_t get_tick() {
        return DWT->CYCCNT;
    }

    static void delay_us(uint32_t u_delay_time) {
        uint32_t start     = DWT->CYCCNT;
        uint32_t delay_cnt = u_delay_time * (SystemCoreClock / 1000000);
        while (DWT->CYCCNT - start < delay_cnt);
    };

private:
    SuperDWT() {
        if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        }
        DWT->CYCCNT  = 0;
        DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
    }
};
