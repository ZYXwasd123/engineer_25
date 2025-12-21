//
// Created by liaohy on 24-11-15.
//

#include "CppTask.hpp"
#include "Chassis/Chassis.hpp"
#include "Interact/Interact.hpp"
#include "RoboArm/RoboArm.hpp"
#include "MicroTime/MicroTime.hpp"

#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId ERROR_TASKHandle;

#ifdef __cplusplus
}
#endif
extern interact_dep::Actions reset1;

MicroTime kineTime;
extern uint8_t re_flag;
void RemoteCtrlTask() {
    using namespace my_math;
    using namespace roboarm_dep;
    const auto& rc = interact.remote_control.rcInfo;
    while (1) {
        auto now = osKernelSysTick();
        if (!interact.remote_control.detect.isLost) {
            if (rc.left != static_cast<uint8_t>(RemoteControl::lever::lower) && rc.right != static_cast<uint8_t>(RemoteControl::lever::lower)) {
                if (rc.wheel < -500) {
                    interact.kb = interact_dep::kb_state::RC_ENABLE;  // 键盘
                } else if (rc.wheel > 500) {
                    interact.kb = interact_dep::kb_state::DISABLE;  // 纯遥控
                }
                if (interact.kb == interact_dep::kb_state::DISABLE) {
                    if (rc.left == static_cast<uint8_t>(RemoteControl::lever::upper)
                        && rc.right == static_cast<uint8_t>(RemoteControl::lever::upper)) {
                        interact.robo_arm.mode = interact_dep::robo_mode::DRAW;
                        interact.chassis.mode  = interact_dep::chassis_mode::CLIMB;
                    } else if (rc.left == static_cast<uint8_t>(RemoteControl::lever::upper)
                               && rc.right == static_cast<uint8_t>(RemoteControl::lever::middle)) {
                        interact.actions = &reset1;
                        interact.robo_arm.mode = interact_dep::robo_mode::ACTIONS;
                        interact.chassis.mode  = interact_dep::chassis_mode::CLIMB;
                    } else if (rc.left == static_cast<uint8_t>(RemoteControl::lever::middle)
                               && rc.right == static_cast<uint8_t>(RemoteControl::lever::upper)) {
                        interact.robo_arm.mode = interact_dep::robo_mode::DRAW;
                        interact.chassis.mode  = interact_dep::chassis_mode::NORMAL;
                    } else if (rc.left == static_cast<uint8_t>(RemoteControl::lever::middle)
                               && rc.right == static_cast<uint8_t>(RemoteControl::lever::middle)) {
												interact.robo_arm.mode = interact_dep::robo_mode::NONE;  // 只控制底盘
                        interact.chassis.mode  = interact_dep::chassis_mode::NORMAL;
                    }
                }
            } else if (rc.left == static_cast<uint8_t>(RemoteControl::lever::lower)) {
                if (interact.kb == interact_dep::kb_state::DISABLE) {
									interact.chassis.mode = interact_dep::chassis_mode::NONE;  // 只控制机械臂
                    if (rc.right == static_cast<uint8_t>(RemoteControl::lever::upper)) {
                        interact.robo_arm.mode = interact_dep::robo_mode::XYZ;  // xyz模式
                    } else if (rc.right == static_cast<uint8_t>(RemoteControl::lever::middle)) {
                        if (interact.robo_arm.mode != interact_dep::robo_mode::NORMAL1
                            && interact.robo_arm.mode != interact_dep::robo_mode::NORMAL2) {
                            interact.robo_arm.mode = interact_dep::robo_mode::NORMAL1;
                        }
                        if (rc.wheel < -500) {
                            interact.robo_arm.mode = interact_dep::robo_mode::NORMAL1;  // 前四个joint
                        } else if (rc.wheel > 500) {
                            interact.robo_arm.mode = interact_dep::robo_mode::NORMAL2;  // 后两个joint和气泵
                        }
                    }
                }
            } else if (rc.right == static_cast<uint8_t>(RemoteControl::lever::lower)) {
                if (interact.kb == interact_dep::kb_state::DISABLE) {
                    interact.chassis.mode = interact_dep::chassis_mode::NONE;
                    if (interact.robo_arm.mode != interact_dep::robo_mode::CUSTOM
                        && interact.robo_arm.mode != interact_dep::robo_mode::VISION) {
                        interact.robo_arm.mode = interact_dep::robo_mode::VISION;
                    }
                    if (rc.wheel < -500) {
                        interact.robo_arm.mode = interact_dep::robo_mode::VISION;  // 视觉
                    } else if (rc.wheel > 500) {
                        interact.robo_arm.mode = interact_dep::robo_mode::CUSTOM;  // 自定义控制器
                    }
                }
            }

            // 判断了kb    更新chassis的目标
            interact.update_chassis(chassis);
            // 还无判断kb
            interact.update_roboArm(roboArm);
            // interact.receive_kb();

        }
        interact.chassis.last_mode  = interact.chassis.mode;
        interact.robo_arm.last_mode = interact.robo_arm.mode;
        RemoteCtrlHeapCnt           = uxTaskGetStackHighWaterMark(NULL);
        osDelayUntil(&now, 14);
    }
}