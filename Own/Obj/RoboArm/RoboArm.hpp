#pragma once

#include "roboarm_dep.hpp"
#include "Uart/SuperUart.hpp"
#include "W25Q64/W25Q64.hpp"

#define ID_ANGLE_PARAM 0x0A
#define ID_SPEED_PARAM 0x0B
#define ID_CURRENT_PARAM 0x0C


extern uint16_t uartlk_param[9];
extern float j4_offset_val;
class Interact;

struct UartLK
{
    UartLK(SuperCan* can, UART_HandleTypeDef *_uart, uint8_t id)
        : uart(_uart, 100, 50), id(id), canPlus(can), detect(1000) {}
    SuperUart uart;
    uint8_t id;
    SuperCan* canPlus;

    void set_position(float position, float speed = 100)  // 默认100dps，两种协议单位差一百倍
    {
        if (!this->detect.isLost)  // 通过串口发送命令
        {
            totalposition2Control(position * 10,speed*100);  // 协议单位0.01度，控制值单位0.1度
        }
        else
        {
            require_feedback();  // 通过串口查询状态
            can_totalposition2Control(position * 10, speed);
        }
    };

    void totalposition2Control(int64_t position, uint32_t speed)  // 翎控串口多圈位置闭环控制命令2
    {
        // 帧命令与校验
        uart.tx_buffer[0] = 0x3E;
        uart.tx_buffer[1] = 0xA4;
        uart.tx_buffer[2] = id;
        uart.tx_buffer[3] = 0x0c;
        uart.tx_buffer[4] = 0x3E + 0xA4 + id + 0x0c;
        // *((uint8_t *)(&motorAngle)+i)逆处理*(uint16_t *)(data)
        uint8_t *data = &uart.tx_buffer[5];
        memcpy(data, &position, 8);
        memcpy(&data[8], &speed, 4);
        data[12] = 0;
        for (int i = 0; i < 12; ++i) {
            data[12] += data[i];
        }
        uart.transmit(18);
    }

    void require_totalposition()  // 串口读取多圈编码器角度
    {
        uart.tx_buffer[0] = 0x3E;
        uart.tx_buffer[1] = 0x92;
        uart.tx_buffer[2] = id;
        uart.tx_buffer[3] = 0x00;
        uart.tx_buffer[4] = 0x3E + 0x92 + id + 0x00;
        uart.transmit(5);
    }

    void require_feedback()  // 串口读取电机状态2
    {
        uart.tx_buffer[0] = 0x3E;
        uart.tx_buffer[1] = 0x9C;
        uart.tx_buffer[2] = id;
        uart.tx_buffer[3] = 0x00;
        uart.tx_buffer[4] = 0x3E + 0x9C + id + 0x00;
        uart.transmit(5);
    }

    void require_ctrl_param(const uint8_t _param_id)  // 串口读取电机参数
    {
        uart.tx_buffer[0] = 0x3E;
        uart.tx_buffer[1] = 0xC0;
        uart.tx_buffer[2] = id;
        uart.tx_buffer[3] = 0x07;
        uart.tx_buffer[4] = 0x05 + id;
        uint8_t* data = &uart.tx_buffer[5];
        data[0] = _param_id;
        for (uint8_t i = 1; i < 7; ++i)
        {
            data[i] = 0;
        }
        data[7] = _param_id;
        uart.transmit(13);
    }

    void set_ctrl_param(const uint8_t mode, const uint16_t _p, const uint16_t _i, const uint16_t _d)  // 串口设置电机参数
    {
        uart.tx_buffer[0] = 0x3E;
        uart.tx_buffer[1] = 0xC1;
        uart.tx_buffer[2] = id;
        uart.tx_buffer[3] = 0x07;
        uart.tx_buffer[4] = 0x3E + 0xC1 + id + 0x07;
        uint8_t *data = &uart.tx_buffer[5];
        data[0] = mode;
        memcpy(&data[1], &_p, 2);
        memcpy(&data[3], &_i, 2);
        memcpy(&data[5], &_d, 2);
        data[7] = 0;
        for (uint8_t i = 0; i < 7; ++i)
        {
            data[7] += data[i];
        }
        uart.transmit(13);
    }

    void CAN_get_feedback(uint16_t id, uint8_t* data) {}
    void get_feed_back(uint8_t *buffer)  // 串口接收回调
    {
        if (buffer[0] == 0x3E && buffer[2] == id)
        {
            switch (buffer[1])
            {
                case 0x88:
                    start_flag = 1;
                    break;
                case 0xA4:
                case 0x9C:
                    if (buffer[3] == 0x07 && buffer[4] == buffer[0] + buffer[1] + buffer[2] + buffer[3])
                    {
                        uint8_t *data = &buffer[4];
                        feedback.data.last_position = feedback.data.position;
                        feedback.raw_data.temperature = data[1];
                        feedback.raw_data.current = *(int16_t *) (&data[2]);
                        feedback.raw_data.speed = *(int16_t *) (&data[4]);
                        feedback.raw_data.position = *(int16_t *) (&data[6]);

                        feedback.data.position = feedback.raw_data.position * 360.f / 65536;
                        feedback.data.speed = feedback.raw_data.speed;
                        feedback.data.current = feedback.raw_data.current * 33.f / 2048.f;
                        feedback.data.temperature = feedback.raw_data.temperature;

                        float dPos = feedback.data.position - feedback.data.last_position;
                        if (dPos > 180) dPos = dPos - 360;
                        else if (dPos < -180) dPos = dPos + 360;
                        total_position += dPos;
                    }
                    break;
                case 0x92:
                    if (buffer[3] == 0x08 && buffer[4] == buffer[0] + buffer[1] + buffer[2] + buffer[3])
                    {
                        uint8_t* data = &buffer[5];
                        int64_t total = 0;
                        memcpy(&total, data, 8);

																																																																																																							total_position = static_cast<float>(total) / 100.f / 10; // 0.01°/LSB
                        float tmp = total_position;
                        feedback.data.position = whileLimit<float>(tmp, 0, 360);
                        feedback.data.last_position = feedback.data.position;
                        offset_flag = 1;
                    }
                    break;
                case 0xC0:
                    if (buffer[3] == 0x07 && buffer[4] == buffer[0] + buffer[1] + buffer[2] + buffer[3])
                        memcpy(&uartlk_param[(buffer[5] - 0x0A) * 3], &buffer[6], 6);
                    break;
                default:
                    break;
            }
            detect.update();
        }
    }

    void close() {
        uart.tx_buffer[0] = 0x3E;
        uart.tx_buffer[1] = 0x80;
        uart.tx_buffer[2] = id;
        uart.tx_buffer[3] = 0x00;
        uart.tx_buffer[4] = 0x3E + 0x80 + id + 0x00;
        uart.transmit(5);
    }

    void enable() {
        uart.tx_buffer[0] = 0x3E;
        uart.tx_buffer[1] = 0x88;
        uart.tx_buffer[2] = id;
        uart.tx_buffer[3] = 0x00;
        uart.tx_buffer[4] = 0x3E + 0x88 + id + 0x00;
        uart.transmit(5);
    }

    void can_enable() {
        uint8_t data[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};
        canPlus->transmit_pdata(id + LKMotor::foc.TX_LOW_ID, data);
    }
    void can_close() {
        uint8_t data[8] = {0x80, 0, 0, 0, 0, 0, 0, 0};
        canPlus->transmit_pdata(id + LKMotor::foc.TX_LOW_ID, data);
    }
    void can_require_feedback() {
        uint8_t data[8] = {0x9c, 0, 0, 0, 0, 0, 0, 0};
        canPlus->transmit_pdata(id + LKMotor::foc.TX_LOW_ID, data);
    }
    void can_totalposition2Control(uint16_t speed, int32_t totalposition)  // 多圈位置闭环控制命令2（CAN）
    {
        uint8_t data[8] = {0xa4,
                           0,
                           *(uint8_t*)(&speed),
                           *((uint8_t*)(&speed) + 1),
                           *(uint8_t*)(&totalposition),
                           *((uint8_t*)(&totalposition) + 1),
                           *((uint8_t*)(&totalposition) + 2),
                           *((uint8_t*)(&totalposition) + 3)};
        canPlus->transmit_pdata(id + LKMotor::foc.TX_LOW_ID, data);
    }
    void can_require_ctrl_param()
    {
        uint8_t data[8] = {0x30, 0, 0, 0, 0, 0, 0, 0};
        canPlus->transmit_pdata(id + LKMotor::foc.TX_LOW_ID, data);
    }

    void detect_lost(Fun callback) { this->detect.lostFun = callback; }
    void detect_recover(Fun callback) { this->detect.recoverFun = callback; }
    void start() {uart.receive_dma_idle(100);};
    Detect detect;
    struct {
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
    } feedback;
    float total_position;
    uint8_t offset_flag  = 0;
    uint8_t start_flag  = 0;
};


class RoboArm {
public:
    RoboArm(SuperCan *canPlus,UART_HandleTypeDef *uart, uint32_t id1, uint32_t range1, const float ratio1, uint32_t id2_internal,
            uint32_t range2_internal, const float ratio2_internal, uint32_t id2_external, uint32_t range2_external,
            const float ratio2_external, uint32_t id3, uint32_t range3, const float ratio3, uint32_t id4,
            uint32_t range4, const float ratio4,
            float alpha, const Pid &pid_pos5, const Pid &pid_speed5,
            uint32_t id5, const uint32_t range5, const float ratio5,
            const uint32_t id6, const uint32_t range6, const float ratio6,
            roboarm_dep::offset &&offset)
            :
            joint1(canPlus,uart, id1),
            joint2{Motor<LKMotorSingle>(canPlus, id2_internal, range2_internal, ratio2_internal),
                   Motor<LKMotorSingle>(canPlus, id2_external, range2_external, ratio2_external)},
            joint3(canPlus, id3, range3, ratio3), joint4(canPlus, id4, range4, ratio4),
            joint5(alpha, pid_pos5, pid_speed5, canPlus, id5, range5, ratio5), joint6(canPlus, id6, range6, ratio6),
            offset{offset}
    {
        // osMu texWait(flash_mutex_id, osWaitForever);
        // reset_error = w25q64.read_buffer(MOTOR_OFFSET_ADDRESS, reinterpret_cast<uint8_t*>(&j4_offset_val), 4);
        // osMutexRelease(flash_mutex_id);
        // offset.joint4 = j4_offset_val;
    }

    void enable();

    void disable();

    void close();

    /* 正运动学解 位置x y z  单位mm */
    std::array<float, 3> position = {0, 0, 0};
    /* 运动学解 姿态z y z    单位为rad*/
    std::array<float, 3> posture = {0, 0, 0};

    /* 逆运动学解 顺序q1 q2 q3 q4 q5 q6 */
    std::array<float, 6> q = {};

    void fkine(std::array<float, 3> &position);

    void fkine(std::array<float, 3> &position, std::array<float, 3> &posture);

    bool ikine(const std::array<float, 3> &position);

    bool ikine(const std::array<float, 3> &position, const std::array<float, 3> &posture, float imu_pitch = 0);

    UartLK joint1;
    struct {
        Motor<LKMotorSingle> internal;
        Motor<LKMotorSingle> external;
    } joint2;
    Motor<LKMotorSingle> joint3;
    Motor<LKMotorSingle> joint4;
    Motor<LKMotorPid> joint5;
    Motor<LKMotorSingle> joint6;

    roboarm_dep::offset offset{};
    roboarm_dep::target target{};
    std::array<float, 4> target_speed{roboarm_dep::default_speed};
    std::array<float, 6> relative_pos;

    void init_offset(std::array<float, 6> &joint);
    void reset_offset();

    void load_target(const std::array<float, 6> &joint, std::array<Slope, 4> &Slope);

    void update_relative_pos();

private:
};

extern RoboArm roboArm;
