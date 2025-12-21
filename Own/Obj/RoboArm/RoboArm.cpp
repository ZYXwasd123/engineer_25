//
// Created by Administrator on 24-10-23.
//

#include "Interact/Interact.hpp"
#include "RoboArm.hpp"
void RoboArm::enable() {
    using namespace roboarm_dep;
    // 先清除错误状态再关闭再开启
    auto close = [](auto& motor) {
        for (uint32_t i = 0; i < MaxTimeOut; i++) {
            if (motor.close_flag) { break; }
            motor.close();
            osDelay(1);
        }
    };
//    close(joint1);
    close(joint2.internal);
    close(joint2.external);
    close(joint3);
    close(joint4);
    close(joint5);
    close(joint6);

    auto clear_error = [](auto& motor) {
        for (uint32_t i = 0; i < MaxTimeOut; i++) {
            if (motor.clear_flag) { break; }
            motor.clear_error();
            osDelay(1);
        }
    };
//    clear_error(joint1);
    clear_error(joint2.internal);
    clear_error(joint2.external);
    clear_error(joint3);
    clear_error(joint4);
    clear_error(joint5);
    clear_error(joint6);

    auto open = [](auto& motor) {
        for (uint32_t i = 0; i < MaxTimeOut; i++) {
            if (motor.start_flag) { break; }
            motor.enable();
            osDelay(1);
        }
    };
    open(joint1);
    open(joint2.internal);
    open(joint2.external);
    open(joint3);
    open(joint4);
    open(joint5);
    open(joint6);
}

void RoboArm::disable() {
    // 不会清除多圈状态
//    joint1.disable();
    joint2.internal.disable();
    joint2.external.disable();
    joint3.disable();
    joint4.disable();
    joint5.disable();
    joint6.disable();

}

void RoboArm::close() {
    // 会清除多圈状态
    joint1.close();
    joint2.internal.close();
    joint2.external.close();
    joint3.close();
    joint4.close();
    joint5.close();
    joint6.close();
}

void RoboArm::reset_offset()
{
    using namespace roboarm_dep;
    j4_offset_val = joint4.total_position;
    offset.joint4 = j4_offset_val;
}

void RoboArm::init_offset(std::array<float, 6>& joint) {
    using namespace roboarm_dep;

    joint6.read_totalposition();
    joint5.read_feedback();
    joint4.read_totalposition();
    joint3.read_totalposition();
    joint2.internal.read_totalposition();
    joint2.external.read_totalposition();
    joint1.require_totalposition();
	osDelay(1);
    //joint1.can_require_ctrl_param();
	osDelay(1);
    //joint1.require_ctrl_param(ID_SPEED_PARAM);
	osDelay(1);
    //joint1.require_ctrl_param(ID_CURRENT_PARAM);


    // target.joint6.angle = 0;
    // target.joint5.angle = 0;
    for (uint32_t i = 0; i < MaxTimeOut; i++) {
        if (joint4.offset_flag) {
            joint[3] = 0;
            if (joint4.feedback.total_position < 0) { offset.joint4 -= 360; }
            break;
        }
        joint4.read_totalposition();
        osDelay(1);
    }
    for (uint32_t i = 0; i < MaxTimeOut; i++) {
        if (joint3.offset_flag) {
            joint[2] = 135;
            if (joint3.feedback.total_position < 0) { offset.joint3 -= 360; }
            break;
        }
        joint3.read_totalposition();
        osDelay(1);
    }
    for (uint32_t i = 0; i < MaxTimeOut; i++) {
        if (joint2.internal.offset_flag) {
            joint[1] = -55;
            if (joint2.internal.feedback.total_position < 0) { offset.joint2.internal -= 360; }
            break;
        }
        joint2.internal.read_totalposition();
        osDelay(1);
    }
    for (uint32_t i = 0; i < MaxTimeOut; i++) {
        if (joint2.external.offset_flag) {
            joint[1] = -55;
//             if (joint2.external.motor.m.feedback.total_position < 0) { offset.joint2.external -= 360; }
            break;
        }
        joint2.external.read_totalposition();
        osDelay(1);
    }


    for (uint32_t i = 0; i < MaxTimeOut; i++) {
        if (joint1.offset_flag) {
            joint[0] = 0;
            if (joint1.total_position < 0) { offset.joint1 -= 360; }
            // target.joint1.angle = offset.joint1 * 100;
            break;
        }
        joint1.require_totalposition();
        osDelay(1);
    }
    // for (uint32_t i = 0; i < MaxTimeOut; i++) {
    //     if (joint5.offset_flag) {
    //         joint[4] = 0;
    //         // if (joint5.feedback.total_position < 0) { offset.joint5 -= 360; }
    //         // target.joint1.angle = offset.joint1 * 100;
    //         break;
    //     }
    //     joint5.require_totalposition();
    //     osDelay(1);
    // }
    for (uint32_t i = 0; i < MaxTimeOut; i++) {
        if (joint6.offset_flag) {
            joint[5] = 0;
            offset.joint6 = joint6.feedback.total_position;
            // if (joint1.feedback.total_position < 0) { offset.joint1 -= 360; }
            break;
        }
        joint6.read_totalposition();
        osDelay(1);
    }
}

void RoboArm::update_relative_pos() {
    relative_pos[0] = (joint1.total_position - offset.joint1);

    relative_pos[1] = -(joint2.external.total_position - offset.joint2.external);

    relative_pos[2] = -(joint3.total_position - offset.joint3);

    relative_pos[3] = (joint4.total_position - offset.joint4);

    relative_pos[4] = -(joint5.total_position - offset.joint5);

    relative_pos[5] = (joint6.total_position/joint6.reduction_ratio - offset.joint6);

    // diff.update_relative_pos(relative_pos[4], relative_pos[5]);

}

void RoboArm::fkine(std::array<float, 3>& position) {
    using namespace my_math;
    using namespace roboarm_dep;
    float cq1;
    float sq1;
    arm_sin_cos_f32(relative_pos[0], &sq1, &cq1);
    float cq2;
    float sq2;
    arm_sin_cos_f32(relative_pos[1], &sq2, &cq2);
    float cq3;
    float sq3;
    arm_sin_cos_f32(relative_pos[2], &sq3, &cq3);
    float cq4;
    float sq4;
    arm_sin_cos_f32(relative_pos[3], &sq4, &cq4);
    float cq5;
    float sq5;
    arm_sin_cos_f32(relative_pos[4], &sq5, &cq5);

    position[0] = 320 * cq1 * sq2 - 122 * sq1 * sq4 * sq5 + (651 * cq1 * cq2 * sq3) / 2 + (651 * cq1 * cq3 * sq2) / 2
                  + 122 * cq1 * cq2 * cq5 * sq3 + 122 * cq1 * cq3 * cq5 * sq2 + 122 * cq1 * cq2 * cq3 * cq4 * sq5
                  - 122 * cq1 * cq4 * sq2 * sq3 * sq5;
    position[1] = 320 * sq1 * sq2 + (651 * cq2 * sq1 * sq3) / 2 + (651 * cq3 * sq1 * sq2) / 2 + 122 * cq1 * sq4 * sq5
                  + 122 * cq2 * cq5 * sq1 * sq3 + 122 * cq3 * cq5 * sq1 * sq2 + 122 * cq2 * cq3 * cq4 * sq1 * sq5
                  - 122 * cq4 * sq1 * sq2 * sq3 * sq5;
    position[2] = 320 * cq2 + (651 * cq2 * cq3) / 2 - (651 * sq2 * sq3) / 2 + 122 * cq2 * cq3 * cq5
                  - 122 * cq5 * sq2 * sq3 - 122 * cq2 * cq4 * sq3 * sq5 - 122 * cq3 * cq4 * sq2 * sq5;

    this->position[0] = position[0];
    this->position[1] = position[1];
    this->position[2] = position[2];
}
void RoboArm::fkine(std::array<float, 3>& position, std::array<float, 3>& posture) {
    using namespace my_math;
    using namespace roboarm_dep;
    float cq1;float sq1;
    arm_sin_cos_f32(relative_pos[0], &sq1,&cq1);
    float cq2;float sq2;
    arm_sin_cos_f32(relative_pos[1], &sq2,&cq2);
    float cq3;float sq3;
    arm_sin_cos_f32(relative_pos[2], &sq3,&cq3);
    float cq4;float sq4;
    arm_sin_cos_f32(relative_pos[3], &sq4,&cq4);
    float cq5;float sq5;
    arm_sin_cos_f32(relative_pos[4], &sq5,&cq5);
    float cq6;float sq6;
    arm_sin_cos_f32(relative_pos[5], &sq6,&cq6);

    position[0] = 320 * cq1 * sq2 - 122 * sq1 * sq4 * sq5 + (651 * cq1 * cq2 * sq3) / 2
                        + (651 * cq1 * cq3 * sq2) / 2 + 122 * cq1 * cq2 * cq5 * sq3 + 122 * cq1 * cq3 * cq5 * sq2
                        + 122 * cq1 * cq2 * cq3 * cq4 * sq5 - 122 * cq1 * cq4 * sq2 * sq3 * sq5;
    position[1] = 320 * sq1 * sq2 + (651 * cq2 * sq1 * sq3) / 2 + (651 * cq3 * sq1 * sq2) / 2
                        + 122 * cq1 * sq4 * sq5 + 122 * cq2 * cq5 * sq1 * sq3 + 122 * cq3 * cq5 * sq1 * sq2
                        + 122 * cq2 * cq3 * cq4 * sq1 * sq5 - 122 * cq4 * sq1 * sq2 * sq3 * sq5;
    position[2] = 320 * cq2 + (651 * cq2 * cq3) / 2 - (651 * sq2 * sq3) / 2 + 122 * cq2 * cq3 * cq5
                        - 122 * cq5 * sq2 * sq3 - 122 * cq2 * cq4 * sq3 * sq5 - 122 * cq3 * cq4 * sq2 * sq5;

    float sq2q3 = sq2 * cq3 + cq2 * sq3;
    float r1_3 = cq1*cq2*cq5*sq3 - sq1*sq4*sq5 + cq1*cq3*cq5*sq2 + cq1*cq2*cq3*cq4*sq5 - cq1*cq4*sq2*sq3*sq5;
    float r2_3 = cq1*sq4*sq5 + cq2*cq5*sq1*sq3 + cq3*cq5*sq1*sq2 + cq2*cq3*cq4*sq1*sq5 - cq4*sq1*sq2*sq3*sq5;
    float r3_3 = cq2*cq3*cq5 - cq5*sq2*sq3 - cq2*cq4*sq3*sq5 - cq3*cq4*sq2*sq5;
    float r3_2 = sq6*(cq2*cq3*sq5 - sq2*sq3*sq5 + cq2*cq4*cq5*sq3 + cq3*cq4*cq5*sq2) + sq2q3*cq6*sq4;
    float r3_1 = sq2q3*sq4*sq6 - cq6*(cq2*cq3*sq5 - sq2*sq3*sq5 + cq2*cq4*cq5*sq3 + cq3*cq4*cq5*sq2);

    float tmp;
    arm_sqrt_f32(r1_3 * r1_3 + r2_3 * r2_3, &tmp);
    arm_atan2_f32(tmp, r3_3, &posture[1]);

    if (is_equal<0.000001f>(posture[1], deg2rad(180))) {
        float sp2 = -my_abs(arm_sin_f32(posture[1]));
        arm_atan2_f32(r2_3/sp2,r1_3/sp2, &posture[0]);
        arm_atan2_f32(r3_2/sp2,-r3_1/sp2, &posture[2]);
    } else {
        float sp2 = arm_sin_f32(posture[1]);
        arm_atan2_f32(r2_3/sp2,r1_3/sp2, &posture[0]);
        arm_atan2_f32(r3_2/sp2,-r3_1/sp2, &posture[2]);
    }

    this->position[0] = position[0];
    this->position[1] = position[1];
    this->position[2] = position[2];

    this->posture[0] = posture[0];
    this->posture[1] = posture[1];
    this->posture[2] = posture[2];
}

bool RoboArm::ikine(const std::array<float, 3>& position) {
    std::array<float, 3> terminal_pitch_position;
    terminal_pitch_position[0] = position[0] - 122 * (arm_cos_f32(posture[0]) * arm_sin_f32(posture[1]));
    terminal_pitch_position[1] = position[1] - 122 * (arm_sin_f32(posture[0]) * arm_sin_f32(posture[1]));
    terminal_pitch_position[2] = position[2] - 122 * (arm_cos_f32(posture[1]));

    bool is_success = false;
    // 选择出当前距离位置更近的q1的解 （q1的另外一个解为q1的对角）
    std::tie(q[0], is_success) = [&terminal_pitch_position, this] -> std::pair<float, bool> {
        using namespace my_math;
        using namespace roboarm_dep;
        float q1;
        arm_atan2_f32(terminal_pitch_position[1], terminal_pitch_position[0], &q1);
        if (my_abs(whileLimit(my_abs(this->relative_pos[0] * d2r - q1), deg2rad(-180), deg2rad(180))) < deg2rad(90))
            if (isInRange(q1 * r2d, limitation.joint1.min, limitation.joint1.max)) return {q1, true};
        if (q1 < 0)
            if (isInRange(q1 * r2d + 180, limitation.joint1.min, limitation.joint1.max))
                return {q1 + deg2rad(180), true};
        if (isInRange(q1 * r2d - 180, limitation.joint1.min, limitation.joint1.max)) return {q1 - deg2rad(180), true};
        return {relative_pos[0], false};
    }();
    if (!is_success) return false;
    {
        // 计算q2的解
        // 为防止除零做的选择
        float r;
        if (is_equal<0.001f>(my_abs(q[0]), deg2rad(90)))
            r = terminal_pitch_position[1] / arm_sin_f32(q[0]);
        else
            r = terminal_pitch_position[0] / arm_cos_f32(q[0]);

        // 中间变量节省计算时间的，同时方便表达式简洁
        auto t0 = 4 * (r * r + terminal_pitch_position[2] * terminal_pitch_position[2]);
        auto t1 = (t0 - 121);
        float t2;
        if (arm_sqrt_f32(-t1 * (t0 - 1666681), &t2) < 0) return false;
        auto t3 = t1 * (2560 * terminal_pitch_position[2] + t0 - 14201);
        auto t4 = t1 * t2;
        auto t5 = (2560 * t0 - 309760) * r;

        float q2[2];

        // 计算出q2的两个解
        q2[0] = 2 * atanf((t5 + t4) / t3);
        q2[1] = 2 * atanf((t5 - t4) / t3);

        // 选择其中最近的a2作为解，并且计算其对应的q3
        if (isInRange<float>(q2[0] * my_math::r2d, roboarm_dep::limitation.joint2.min,
                             roboarm_dep::limitation.joint2.max)
            && Rdistance(q2[1], relative_pos[1] * my_math::d2r) >= Rdistance(q2[0], relative_pos[1] * my_math::d2r)) {
            q[1] = q2[0];
            q[2] = -2 * atanf(t2 / t1);
        } else if (isInRange<float>(q2[1] * my_math::r2d, roboarm_dep::limitation.joint2.min,
                                    roboarm_dep::limitation.joint2.max)) {
            q[1] = q2[1];
            q[2] = 2 * atanf(t2 / t1);
        } else {
            return false;
        }
    }
    float r1_3;
    float r2_3;
    float r3_3;
    float r3_2;
    float r3_1;
    {
        float cq1;
        float sq1;
        arm_sin_cos_f32(q[0] * my_math::r2d, &sq1, &cq1);
        float cq2;
        float sq2;
        arm_sin_cos_f32(q[1] * my_math::r2d, &sq2, &cq2);
        float cq3;
        float sq3;
        arm_sin_cos_f32(q[2] * my_math::r2d, &sq3, &cq3);

        float cp1;
        float sp1;
        arm_sin_cos_f32(posture[0] * my_math::r2d, &sp1, &cp1);
        float cp2;
        float sp2;
        arm_sin_cos_f32(posture[1] * my_math::r2d, &sp2, &cp2);
        float cp3;
        float sp3;
        arm_sin_cos_f32(posture[2] * my_math::r2d, &sp3, &cp3);
        float sq2q3 = sq2 * cq3 + cq2 * sq3;
        float cq2q3 = cq2 * cq3 - sq2 * sq3;

        r3_3 = cp2 * cq2 * cq3 - cp2 * sq2 * sq3 + cq1 * cq2 * cp1 * sp2 * sq3 + cq1 * cq3 * cp1 * sp2 * sq2
               + cq2 * sp2 * sq1 * sq3 * sp1 + cq3 * sp2 * sq1 * sq2 * sp1;

        if (is_equal<0.0001f>(r3_3, 1)) {
            q[4]       = 0;
            float r2_1 = cq1 * (cp1 * sp3 + cp2 * cp3 * sp1) + sq1 * (sp1 * sp3 - cp2 * cp1 * cp3);
            float r2_2 = cq1 * (cp1 * cp3 - cp2 * sp1 * sp3) + sq1 * (cp3 * sp1 + cp2 * cp1 * sp3);
            q[3]       = relative_pos[3];
            float all;
            arm_atan2_f32(r2_1, r2_2, &all);
            q[5] = all - relative_pos[3];
            return true;
        }

        r1_3 = cq1 * cq2 * cq3 * cp1 * sp2 - cp2 * cq3 * sq2 - cp2 * cq2 * sq3 - cq1 * cp1 * sp2 * sq2 * sq3
               + cq2 * cq3 * sp2 * sq1 * sp1 - sp2 * sq1 * sq2 * sq3 * sp1;
        // r2_3 = -sin(q[0] - posture[0])*sp2;
        r2_3 = -(sq1 * cp1 - cq1 * sp1) * sp2;

        r3_2 = sq2q3 * sq1 * (cp1 * cp3 - cp2 * sp1 * sp3) - sq2q3 * cq1 * (cp3 * sp1 + cp2 * cp1 * sp3)
               + cq2q3 * sp2 * sp3;
        r3_1 = sq2q3 * sq1 * (cp1 * sp3 + cp2 * cp3 * sp1) - sq2q3 * cq1 * (sp1 * sp3 - cp2 * cp1 * cp3)
               - cq2q3 * cp3 * sp2;
    }
    {
        float tmp;
        arm_sqrt_f32(r1_3 * r1_3 + r2_3 * r2_3, &tmp);
        float q5[2];
        arm_atan2_f32(tmp, r3_3, &q5[0]);
        arm_atan2_f32(-tmp, r3_3, &q5[1]);

        bool q5_0range = isInRange(q5[0] * my_math::r2d, roboarm_dep::limitation.joint5.min, roboarm_dep::limitation.joint5.max);
        bool q5_1range = isInRange(q5[1] * my_math::r2d, roboarm_dep::limitation.joint5.min, roboarm_dep::limitation.joint5.max);
        if (q5_0range && q5_1range) {
            float q4[2];float q6[2];
            float qr[2];
            qr[0] = Rdistance(q5[0], relative_pos[4] * my_math::d2r);
            qr[1] = Rdistance(q5[1], relative_pos[4] * my_math::d2r);
            float sq5 = arm_sin_f32(q5[0]);
            arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q4[0]);
            arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q6[0]);
            qr[0] += Rdistance(q4[0], relative_pos[3] * my_math::d2r);
            qr[0] += Rdistance(q6[0], relative_pos[5] * my_math::d2r);
            sq5 = arm_sin_f32(q5[1]);
            arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q4[1]);
            arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q6[1]);
            qr[1] += Rdistance(q4[1], relative_pos[3] * my_math::d2r);
            qr[1] += Rdistance(q6[1], relative_pos[5] * my_math::d2r);
            if (qr[0] > qr[1]) {
                q[3] = q4[1];
                q[4] = q5[1];
                q[5] = q6[1];
            } else {
                q[3] = q4[0];
                q[4] = q5[0];
                q[5] = q6[0];
            }
        } else if (q5_0range) {
            q[4] = q5[0];
            float sq5 = arm_sin_f32(q[4]);
            arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q[3]);
            arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q[5]);
        } else if (q5_1range) {
            q[4] = q5[1];
            float sq5 = arm_sin_f32(q[4]);
            arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q[3]);
            arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q[5]);
        } else {
            return false;
        }
    }
    // float sq5 = arm_sin_f32(q[4]);
    // arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q[3]);
    // arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q[5]);
    return true;
}
bool RoboArm::ikine(const std::array<float, 3>& position, const std::array<float, 3>& posture, float imu_pitch) {
    std::array<float, 3> terminal_pitch_position{};
    float cimu,simu;
    if (my_abs(imu_pitch)<90) {
        arm_sin_cos_f32(imu_pitch, &simu, &cimu);
    } else {
        imu_pitch = 0;
    }
    terminal_pitch_position[0] = position[0] - 122 * cimu * (arm_cos_f32(posture[0]) * arm_sin_f32(posture[1])) + 122 * arm_cos_f32(posture[1]) * simu;
    terminal_pitch_position[1] = position[1] - 122 * (arm_sin_f32(posture[0]) * arm_sin_f32(posture[1]));
    terminal_pitch_position[2] = position[2] - 122 * cimu * (arm_cos_f32(posture[1])) - 122 * arm_cos_f32(posture[0])*simu*arm_sin_f32(posture[1]);

    bool is_success = false;
    // 选择出当前距离位置更近的q1的解 （q1的另外一个解为q1的对角）
    std::tie(q[0], is_success) = [&terminal_pitch_position, this] -> std::pair<float, bool> {
        using namespace my_math;
        using namespace roboarm_dep;
        float q1;
        arm_atan2_f32(terminal_pitch_position[1], terminal_pitch_position[0], &q1);
        if (my_abs(whileLimit(my_abs(this->relative_pos[0] * d2r - q1), deg2rad(-180), deg2rad(180))) < deg2rad(90))
            if (isInRange(q1 * r2d, limitation.joint1.min, limitation.joint1.max)) return {q1, true};
        if (q1 < 0)
            if (isInRange(q1 * r2d + 180, limitation.joint1.min, limitation.joint1.max))
                return {q1 + deg2rad(180), true};
        if (isInRange(q1 * r2d - 180, limitation.joint1.min, limitation.joint1.max)) return {q1 - deg2rad(180), true};
        return {relative_pos[0], false};
    }();
    if (!is_success) return false;
    {
        // 计算q2的解
        // 为防止除零做的选择
        float r;
        if (is_equal<0.001f>(my_abs(q[0]), deg2rad(90)))
            r = terminal_pitch_position[1] / arm_sin_f32(q[0]);
        else
            r = terminal_pitch_position[0] / arm_cos_f32(q[0]);

        // 中间变量节省计算时间的，同时方便表达式简洁
        auto t0 = 4 * (r * r + terminal_pitch_position[2] * terminal_pitch_position[2]);
        auto t1 = (t0 - 121);
        float t2;
        if (arm_sqrt_f32(-t1 * (t0 - 1666681), &t2) < 0) return false;
        auto t3 = t1 * (2560 * terminal_pitch_position[2] + t0 - 14201);
        auto t4 = t1 * t2;
        auto t5 = (2560 * t0 - 309760) * r;

        float q2[2];

        // 计算出q2的两个解
        q2[0] = 2 * atanf((t5 + t4) / t3);
        q2[1] = 2 * atanf((t5 - t4) / t3);

        // 选择其中最近的a2作为解，并且计算其对应的q3
        if (isInRange<float>(q2[0] * my_math::r2d, roboarm_dep::limitation.joint2.min,
                             roboarm_dep::limitation.joint2.max)
            && Rdistance(q2[1], relative_pos[1] * my_math::d2r) >= Rdistance(q2[0], relative_pos[1] * my_math::d2r)) {
            q[1] = q2[0];
            q[2] = -2 * atanf(t2 / t1);
        } else if (isInRange<float>(q2[1] * my_math::r2d, roboarm_dep::limitation.joint2.min,
                                    roboarm_dep::limitation.joint2.max)) {
            q[1] = q2[1];
            q[2] = 2 * atanf(t2 / t1);
        } else {
            return false;
        }
    }
    float r1_3;float r2_3;float r3_3;float r3_2;float r3_1;
    {
        float cq1;
        float sq1;
        arm_sin_cos_f32(q[0] * my_math::r2d, &sq1, &cq1);
        float cq2;
        float sq2;
        arm_sin_cos_f32(q[1] * my_math::r2d, &sq2, &cq2);
        float cq3;
        float sq3;
        arm_sin_cos_f32(q[2] * my_math::r2d, &sq3, &cq3);
        float cp1;
        float sp1;
        arm_sin_cos_f32(posture[0] * my_math::r2d, &sp1, &cp1);
        float cp2;
        float sp2;
        arm_sin_cos_f32(posture[1] * my_math::r2d, &sp2, &cp2);
        float cp3;
        float sp3;
        arm_sin_cos_f32(posture[2] * my_math::r2d, &sp3, &cp3);
        float sq2q3 = sq2 * cq3 + cq2 * sq3;
        float cq2q3 = cq2 * cq3 - sq2 * sq3;

//        cos(q2 + q3)*(cos(imu_pitch)*cos(p) + cos(r)*sin(imu_pitch)*sin(p)) - sin(q2 + q3)*cos(q1)*(cos(p)*sin(imu_pitch) - cos(imu_pitch)*cos(r)*sin(p)) + sin(q2 + q3)*sin(p)*sin(q1)*sin(r)
//        r3_3 = cp2 * cq2 * cq3 - cp2 * sq2 * sq3 + cq1 * cq2 * cp1 * sp2 * sq3 + cq1 * cq3 * cp1 * sp2 * sq2
//               + cq2 * sp2 * sq1 * sq3 * sp1 + cq3 * sp2 * sq1 * sq2 * sp1;
        r3_3 = cq2q3*(cimu*cp2+cp1*simu*sp2) - sq2q3*cq1*(cp2*simu-cimu*cp1*sp2) + sq2q3*sp2*sq1*sp1;

        if (is_equal<0.0001f>(r3_3, 1)) {
            q[4]       = 0;
//  sin(q1)*(cos(imu_pitch)*(sin(r)*sin(y) - cos(p)*cos(r)*cos(y)) - cos(y)*sin(imu_pitch)*sin(p)) + cos(q1)*(cos(r)*sin(y) + cos(p)*cos(y)*sin(r))
            float r2_1 = cq1 * (cp1 * sp3 + cp2 * cp3 * sp1) + sq1 * (cimu * (sp1 * sp3 - cp2 * cp1 * cp3) - cp3 * simu * sp2);
//  sin(q1)*(cos(imu_pitch)*(cos(y)*sin(r) + cos(p)*cos(r)*sin(y)) + sin(imu_pitch)*sin(p)*sin(y)) + cos(q1)*(cos(r)*cos(y) - cos(p)*sin(r)*sin(y))
            float r2_2 = cq1 * (cp1 * cp3 - cp2 * sp1 * sp3) + sq1 * (cimu * (cp3 * sp1 + cp2 * cp1 * sp3) + simu * sp2 * sp3);
            q[3]       = relative_pos[3];
            float all;
            arm_atan2_f32(r2_1, r2_2, &all);
            q[5] = all - relative_pos[3];
            return true;
        }

//  cos(q2 + q3)*sin(p)*sin(q1)*sin(r) - cos(q2 + q3)*cos(q1)*(cos(p)*sin(imu_pitch) - cos(imu_pitch)*cos(r)*sin(p)) - sin(q2 + q3)*(cos(imu_pitch)*cos(p) + cos(r)*sin(imu_pitch)*sin(p))
//        r1_3 = cq1 * cq2 * cq3 * cp1 * sp2 - cp2 * cq3 * sq2 - cp2 * cq2 * sq3 - cq1 * cp1 * sp2 * sq2 * sq3
//               + cq2 * cq3 * sp2 * sq1 * sp1 - sp2 * sq1 * sq2 * sq3 * sp1;

        r1_3 = cq2q3*sp2*sq1*sp1-cq2q3*cq1*(cp2*simu-cimu*cp1*sp2) - sq2q3*(cimu*cp2+cp1*simu*sp2);

        // r2_3 = -sin(q[0] - posture[0])*sp2;
        //  sin(q1)*(cos(p)*sin(imu_pitch) - cos(imu_pitch)*cos(r)*sin(p)) + cos(q1)*sin(p)*sin(r)
//        r2_3 = -(sq1 * cp1 - cq1 * sp1) * sp2;
        r2_3 = sq1*(cp2*simu-cimu*cp1*sp2) + cq1*sp2*sp1;

//  sin(q2 + q3)*sin(q1)*(cos(r)*cos(y) - cos(p)*sin(r)*sin(y)) - sin(q2 + q3)*cos(q1)*(sin(imu_pitch)*sin(p)*sin(y) + cos(imu_pitch)*cos(y)*sin(r) + cos(imu_pitch)*cos(p)*cos(r)*sin(y)) - cos(q2 + q3)*(cos(y)*sin(imu_pitch)*sin(r) - cos(imu_pitch)*sin(p)*sin(y) + cos(p)*cos(r)*sin(imu_pitch)*sin(y))
        r3_2 = sq2q3 * sq1 * (cp1 * cp3 - cp2 * sp1 * sp3) - sq2q3 * cq1 * (simu*sp2*sp3 + cimu*(cp3 * sp1 + cp2 * cp1 * sp3))
               + cq2q3 * (simu*(cp1*cp2*sp3+cp3*sp1)+(cimu*sp2 * sp3));
//  sin(q2 + q3)*sin(q1)*(cos(r)*sin(y) + cos(p)*cos(y)*sin(r)) - cos(q2 + q3)*(sin(imu_pitch)*sin(r)*sin(y) + cos(imu_pitch)*cos(y)*sin(p) - cos(p)*cos(r)*cos(y)*sin(imu_pitch)) + sin(q2 + q3)*cos(q1)*(cos(y)*sin(imu_pitch)*sin(p) - cos(imu_pitch)*sin(r)*sin(y) + cos(imu_pitch)*cos(p)*cos(r)*cos(y))
        r3_1 = sq2q3 * sq1 * (cp1 * sp3 + cp2 * cp3 * sp1) - sq2q3 * cq1 * (cimu*(sp1 * sp3 - cp2 * cp1 * cp3) - cp3*simu*sp2)
               - cq2q3 * (simu*sp1*sp3 + cimu * cp3 * sp2 - simu*cp1*cp2*cp3);
    }
    {
        float tmp;
        arm_sqrt_f32(r1_3 * r1_3 + r2_3 * r2_3, &tmp);
        float q5[2];
        arm_atan2_f32(tmp, r3_3, &q5[0]);
        arm_atan2_f32(-tmp, r3_3, &q5[1]);

        bool q5_0range = isInRange(q5[0] * my_math::r2d, roboarm_dep::limitation.joint5.min, roboarm_dep::limitation.joint5.max);
        bool q5_1range = isInRange(q5[1] * my_math::r2d, roboarm_dep::limitation.joint5.min, roboarm_dep::limitation.joint5.max);
        if (q5_0range && q5_1range) {
            float q4[2];float q6[2];
            float qr[2];
            // 附加权重，权重越大，表示越希望该关节活动距离越小
            qr[0] = 2*Rdistance(q5[0], relative_pos[4] * my_math::d2r);
            qr[1] = 2*Rdistance(q5[1], relative_pos[4] * my_math::d2r);
            float sq5 = arm_sin_f32(q5[0]);
            arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q4[0]);
            arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q6[0]);
            qr[0] += Rdistance(q4[0], relative_pos[3] * my_math::d2r);
            qr[0] += Rdistance(q6[0], relative_pos[5] * my_math::d2r);
            sq5 = arm_sin_f32(q5[1]);
            arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q4[1]);
            arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q6[1]);
            qr[1] += Rdistance(q4[1], relative_pos[3] * my_math::d2r);
            qr[1] += Rdistance(q6[1], relative_pos[5] * my_math::d2r);
            if (qr[0] > qr[1]) {
                q[3] = q4[1];
                q[4] = q5[1];
                q[5] = q6[1];
            } else {
                q[3] = q4[0];
                q[4] = q5[0];
                q[5] = q6[0];
            }

        } else if (q5_0range) {
            q[4] = q5[0];
            float sq5 = arm_sin_f32(q[4]);
            arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q[3]);
            arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q[5]);
        } else if (q5_1range) {
            q[4] = q5[1];
            float sq5 = arm_sin_f32(q[4]);
            arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q[3]);
            arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q[5]);
        } else {
            return false;
        }
    }

    // float sq5 = arm_sin_f32(q[4]);
    // arm_atan2_f32(r2_3 / sq5, r1_3 / sq5, &q[3]);
    // arm_atan2_f32(r3_2 / sq5, -r3_1 / sq5, &q[5]);
    return true;
}
float target_joint5;
void RoboArm::load_target(const std::array<float, 6>& joint, std::array<Slope, 4>& slope) {
    using namespace roboarm_dep;
    using namespace my_math;

    //一些电机的转向和人为规定的反了,故加上了负号.
    slope[0].target_set(joint[0]);
    slope[1].target_set(joint[1]);
    slope[2].target_set(joint[2]);
    slope[3].target_set(joint[4]);
    auto joint2_slope_value = slope[1].update();
    target.joint1.angle          = (slope[0].update() + offset.joint1) * scale(360, 36000);
    target.joint2.internal.angle = (-joint2_slope_value + offset.joint2.internal) * scale(360, 36000);
    target.joint2.external.angle = (-joint2_slope_value + offset.joint2.external) * scale(360, 36000);
    target.joint3.angle          = (-slope[2].update() + offset.joint3) * scale(360, 36000);

    auto data3 = joint[3];
    float err3 = joint[3] - relative_pos[3]; // 10 <- 350 + 360 = -340 - 360 // 350 <- 10 = 340
    while (err3 >= 180) { err3 -= 360; }
    while (err3 < -170) { err3 += 360; }
    data3 = relative_pos[3] + err3;
    target.joint4.angle          = (data3 + offset.joint4) * scale(360, 36000);

    float data;
    float err = joint[5] - relative_pos[5]; // 10 <- 350 + 360 = -340 - 360 // 350 <- 10 = 340
    while (err >= 180) { err -= 360; }
    while (err < -170) { err += 360; }
    data = relative_pos[5] + err;
    target_joint5 = data;

    target.joint5.angle = (-slope[3].update() + offset.joint5) * scale(360, 36000);
    target.joint6.angle = (data + offset.joint6) * scale(360, 36000);
}

