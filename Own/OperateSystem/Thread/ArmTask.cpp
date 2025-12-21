//
// Created by Administrator on 2025/4/4.
//
#include "CppTask.hpp"
#include "Interact/Interact.hpp"
#include "RoboArm/RoboArm.hpp"
#include "Imu/Imu.hpp"
struct  {
    float speed_p;
    float speed_i;
    float speed_d;

    float pos_p;
    float pos_i;
    float pos_d;

    float pos;
    float target;
}pid_parma = {.pos_p = 100,.pos_i = 0.002, .pos_d = 35,.speed_p = 0.25,.speed_i = 0.01,.speed_d = 0,};

float joint5_current;
extern bool is_error;
bool is_clear;
void ArmTask() {
    uint32_t cnt = 0;

    while (true) {
        ++cnt;
        interact.receive_actions_group();
        interact.receive_actions(roboArm, imu.pitch);
        interact.receive_kb();
        roboArm.update_relative_pos();
        roboArm.load_target(interact.joint, interact.joint_slope);
        if (is_clear) {
            is_clear = false;
            interact.joint[3] = roboArm.relative_pos[3];
//                interact.joint[4] = roboArm.relative_pos[4];
            roboArm.joint6.total_position = 0;
            roboArm.relative_pos[5] = 0;
            interact.joint[5] = roboArm.relative_pos[5];
        }
        if (is_error == false) {
            if (cnt % 5 == 0) {
                xSemaphoreTake(CAN1MutexHandle, portMAX_DELAY);
//         roboArm.joint5.read_feedback();
                pid_parma.target = roboArm.target.joint5.angle / 100.f;
                pid_parma.pos = roboArm.joint5.total_position;
                joint5_current = roboArm.joint5.feedback.raw_data.current;
                if (interact.sub_board.custom_frame_rx.s.valve2 > 700) {
                    roboArm.joint5.change_speed_kp(pid_parma.speed_p);
                    roboArm.joint5.change_speed_ki(pid_parma.speed_i);
                    roboArm.joint5.change_speed_kd(pid_parma.speed_d);
                    roboArm.joint5.change_pos_kp(pid_parma.pos_p);
                    roboArm.joint5.change_pos_ki(pid_parma.pos_i);
                    roboArm.joint5.change_pos_kd(pid_parma.pos_d);
                } else {
                    roboArm.joint5.change_speed_kp(pid_parma.speed_p);
                    roboArm.joint5.change_speed_ki(pid_parma.speed_i);
                    roboArm.joint5.change_speed_kd(pid_parma.speed_d);
                    roboArm.joint5.change_pos_kp(pid_parma.pos_p);
                    roboArm.joint5.change_pos_ki(pid_parma.pos_i);
                    roboArm.joint5.change_pos_kd(pid_parma.pos_d);
                }
                roboArm.joint5.set_position(roboArm.target.joint5.angle , 720.f);
//                roboArm.joint5.SingleControl();
                xSemaphoreGive(CAN1MutexHandle);
            }
            if ((cnt + 4) % 5 == 0) {
                xSemaphoreTake(CAN1MutexHandle, portMAX_DELAY);
                roboArm.joint6.set_position(roboArm.target.joint6.angle, 720);
                roboArm.joint4.set_position(roboArm.target.joint4.angle, roboArm.target_speed[3]);
                xSemaphoreGive(CAN1MutexHandle);
            }
        } else {
            if (cnt % 2 == 0) {
                xSemaphoreTake(CAN1MutexHandle, portMAX_DELAY);
                if (roboArm.joint5.close_flag==0) {
                    roboArm.joint5.close();
                } else if (roboArm.joint5.start_flag==0){
                    roboArm.joint5.enable();
                } else if (roboArm.joint5.offset_flag==0) {
//                    roboArm.joint5.read_totalposition();
                }
                if (roboArm.joint6.close_flag==0) {
                    roboArm.joint6.close();
                }else if (roboArm.joint6.start_flag==0){
                    roboArm.joint6.enable();
                } else if (roboArm.joint6.offset_flag==0) {
//                    roboArm.joint6.read_totalposition();
                }
                if (roboArm.joint4.close_flag==0) {
                    roboArm.joint4.close();
                }else if (roboArm.joint4.start_flag==0){
                    roboArm.joint4.enable();
                } else if (roboArm.joint4.offset_flag==0) {
                    roboArm.joint4.read_totalposition();
                }
                xSemaphoreGive(CAN1MutexHandle);
            }
            if (roboArm.joint4.offset_flag&&roboArm.joint5.start_flag&&roboArm.joint6.start_flag) {
                is_error = false;
                is_clear = true;
            }
        }
        if ((cnt) % 5 == 0) {
            xSemaphoreTake(CAN1MutexHandle, portMAX_DELAY);
            roboArm.joint3.set_position(roboArm.target.joint3.angle, roboArm.target_speed[2]);
            xSemaphoreGive(CAN1MutexHandle);
        }
        if ((cnt + 1) % 5 == 0) {
            xSemaphoreTake(CAN1MutexHandle, portMAX_DELAY);
            roboArm.joint2.internal.set_position(roboArm.target.joint2.internal.angle, roboArm.target_speed[1]);
            roboArm.joint2.external.set_position(roboArm.target.joint2.external.angle, roboArm.target_speed[1]);
            xSemaphoreGive(CAN1MutexHandle);
        }
        if ((cnt + 2) % 5 == 0) {
            xSemaphoreTake(CAN1MutexHandle, portMAX_DELAY);
            roboArm.joint1.set_position(roboArm.target.joint1.angle, roboArm.target_speed[0]);  // 位置闭环控制
            xSemaphoreGive(CAN1MutexHandle);
        }

        CANHeapCnt = uxTaskGetStackHighWaterMark(NULL);
        osDelay(1);
    }
}