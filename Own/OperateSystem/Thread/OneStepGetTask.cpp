//
// Created by Administrator on 25-5-2.
//
#include "Interact/Interact.hpp"
#include "OneStepGet/OneStepGet.hpp"

#ifdef __cplusplus
extern "C" {
#endif
#include "cmsis_os.h"
#ifdef __cplusplus
}
#endif


void OneStepGetTask()
{
    // one_step_gets.rotate.set_target(osg::rota_init);
    uint32_t time = 0;
    while (1)
    {
        //if (time++ % 10 == 0)
        //{
            // one_step_gets.rotate.read();
            // one_step_gets.rotate.set_pos_speed(one_step_gets.rotate.target / 180.f * 2048,100);
        //}
        if (++time % 2 == 0) {
            one_step_gets.Yright.state_handle();
            one_step_gets.Yleft.state_handle();
            one_step_gets.Xright.state_handle();
            one_step_gets.Xleft.state_handle();
            one_step_gets.rotate_move.state_handle();

            one_step_gets.Yright.move_handle();
            one_step_gets.Yleft.move_handle();
            one_step_gets.Xright.move_handle();
            one_step_gets.Xleft.move_handle();
            one_step_gets.rotate_move.move_handle();


            canPlus3.transmit(0x200,
                              one_step_gets.Yright.Motor.speed_output(),
                              one_step_gets.Yleft.Motor.speed_output(),
                              one_step_gets.Xright.Motor.speed_output(),
                              one_step_gets.Xleft.Motor.speed_output());
            canPlus3.transmit(M2006::foc.TX_HIGH_ID, 0, one_step_gets.rotate_move.Motor.speed_output(), 0, 0);
        }
        osDelay(1);
    }
}