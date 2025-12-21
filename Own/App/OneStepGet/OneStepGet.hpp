//
// Created by Administrator on 25-5-1.
//

#pragma once

#include "Motor/Motor.hpp"
#include "Slope/Slope.hpp"
#include "CanServos/CanServos.hpp"

enum class OneStepGetControl {
    MANUAL,
    AUTO,
    ROBO_ARM,
};
namespace translation {
    enum class state {
        RESET,
        P_BLOCK,
        N_BLOCK,
        MOVE,
    };
}
template<typename T>
class Translation {
    using state = translation::state;
public:
    Translation(const Pid &x_pos_pid, const Pid &x_speed_pid, const uint8_t x_id, const Slope &x_slope_cfg,bool p,
                int16_t pThreshold, int16_t nThreshold)
            : s(state::RESET),polarity(p),axis(x_slope_cfg), pblock_count(0),nblock_count(0), pThresholdBlock(pThreshold), nThresholdBlock(nThreshold),
              Motor(x_pos_pid, x_speed_pid, x_id) {};


    void move_handle();
    void state_handle();

    void set_target(float target) { axis.target_set(target); };
    void set_step(float step) {axis.step_set(step);};

    void set_positive_threshold(int16_t threshold) { pThresholdBlock = threshold; }
    void set_negative_threshold(int16_t threshold) { nThresholdBlock = threshold; }

    void set_state(state s, float param)
    {
        if (s == state::RESET)
        {
            this->s = state::RESET;
            return;
        }
        if (this->s != state::P_BLOCK && this->s != state::N_BLOCK) this->s = s;
        axis.target_set(param);
    }
    Motor<T> Motor;
    Slope axis;

private:
    state s;
    bool polarity;

    int32_t pblock_count;
    int32_t nblock_count;
    int16_t pThresholdBlock;
    int16_t nThresholdBlock;
};

template<typename T>
void Translation<T>::move_handle()
{
    Motor.set_position(axis.get());  // 步进移动
};
template<typename T>
void Translation<T>::state_handle() {
    switch (s) {
        case state::P_BLOCK:
            if (pblock_count > -5)  // 未脱离堵转，目标后移
                axis.decrease();
            else  // 脱离堵转，回到移动状态
            {
                pblock_count = 0;
                s = state::MOVE;
            }
            if (Motor.feedback.raw_data.current < pThresholdBlock)
            {
                --pblock_count;
            }
            else
            {
                ++pblock_count;
            }
            break;
        case state::N_BLOCK:
            if (nblock_count > -5)
                axis.increase();
            else
            {
                nblock_count = 0;
                s = state::MOVE;
            }
            if (Motor.feedback.raw_data.current > nThresholdBlock)
            {
                --nblock_count;
            }
            else
            {
                ++nblock_count;
            }
            break;
        case state::MOVE:
            axis.update();
            if (Motor.feedback.raw_data.current > pThresholdBlock)  // 大于最大值，正向堵转
            {
                if(++pblock_count > 20)
                {
                    s = state::P_BLOCK;
                };
            }
            else if (Motor.feedback.raw_data.current < nThresholdBlock)  // 小于最小值，负向堵转
            {
                if(++pblock_count > 20)
                {
                    s = state::N_BLOCK;
                };
            }
            break;
        case state::RESET:
            if (polarity)  // 正极性负向撤，负极性正向撤，直到机械限位
            {
                if (Motor.feedback.raw_data.current < nThresholdBlock)
                {
                    if(++nblock_count > 10)  // 负向堵转，设为当前点
                    {
                        Motor.total_position() = 0;
                        axis.target_set(0);
                        axis.target_arrive();
                        nblock_count = 10;
                        s = state::N_BLOCK;
                    }
                }
                else  // 负向未堵转，继续后缩
                {
                    axis.target_set(-10000);
                }
            }
            else
            {
                if (Motor.feedback.raw_data.current > pThresholdBlock)
                {
                    if(++pblock_count > 10)
                    {
                        Motor.total_position() = 0;
                        Motor.clear();
                        axis.target_set(0);
                        axis.target_arrive();
                        pblock_count = 10;
                        s = state::P_BLOCK;
                    }
                }
                else
                {
                    axis.target_set(10000);
                }
            }
            axis.update();  // 更新步进值
            break;
    }
};
namespace osg {
    constexpr float xl_max = -1540.33521; //  -1636   -1502.33521
    constexpr float yl_max = 230;   //  224
    constexpr float xr_max = 1630.41211;  //  1666    1557.41211
    constexpr float yr_max = -218;  //  -244
    constexpr float rota_init = 185.449219;  //  -244
    constexpr float rota_up = 100.4707031;  //  -244

    constexpr float xl_meter = 340;
    constexpr float xr_meter = 350;

    constexpr float yl_meter = 70;
    constexpr float yr_meter = 70;


    namespace M2006 {
        consteval float meter2deg(float meter) {
            return meter / 6.28f  * 360 / 12 ;
        }
    }
    namespace M3508 {
        consteval float meter2deg(float meter) {
            return meter / 6.28f  * 360 / 17 ;
        }
    }
}

class OSG {
public:
    static OneStepGetControl mode;

    OSG(const Pid &xl_pos_pid, const Pid &xl_speed_pid, const uint8_t xl_id,const Slope &xl_slope,bool xl_p,int16_t xl_pt,int16_t xl_nt,
        const Pid &yl_pos_pid, const Pid &yl_speed_pid,const uint8_t yl_id, const Slope &yl_slope,bool yl_p,int16_t yl_pt,int16_t yl_nt,
        const Pid &xr_pos_pid,const Pid &xr_speed_pid, const uint8_t xr_id, const Slope &xr_slope,bool xr_p,int16_t xr_pt,int16_t xr_nt,
        const Pid &yr_pos_pid, const Pid &yr_speed_pid, const uint8_t yr_id,const Slope &yr_slope,bool yr_p,int16_t yr_pt,int16_t yr_nt,
        const Pid &rotate_pos_pid, const Pid &rotate_speed_pid, const uint8_t rotate_id,const Slope &rotate_slope,bool rotate_p,int16_t rotate_pt,int16_t rotate_nt,
        SuperCan*canPluse, uint32_t id)
            : Xleft(xl_pos_pid, xl_speed_pid, xl_id, xl_slope, xl_p,xl_pt,xl_nt),
              Yleft(yl_pos_pid, yl_speed_pid, yl_id, yl_slope, yl_p, yl_pt, yl_nt),
              Xright(xr_pos_pid, xr_speed_pid, xr_id, xr_slope,xr_p,xr_pt,xr_nt),
              Yright(yr_pos_pid, yr_speed_pid, yr_id, yr_slope,yr_p,yr_pt,yr_nt),
              rotate_move(rotate_pos_pid, rotate_speed_pid, rotate_id, rotate_slope,rotate_p,rotate_pt,rotate_nt),
              rotate(canPluse, id) {};
        Translation<M2006Pos> Xleft;  // 水平
        Translation<M2006Pos> Xright;  // 抬升
        Translation<M3508Pos> Yleft;
        Translation<M3508Pos> Yright;

        Translation<M2006Pos> rotate_move;
        CanServos rotate;

};

extern OSG one_step_gets;
