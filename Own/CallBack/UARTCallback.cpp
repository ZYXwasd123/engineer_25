//
// Created by Administrator on 24-10-3.
//

#include "Interact/Interact.hpp"
#include "Judge/ui.hpp"
#include "ThreadConfig.h"
#include "RoboArm/RoboArm.hpp"

extern "C" {
extern osThreadId ERROR_TASKHandle;
}
extern std::atomic<bool> rc_ready;
extern uint16_t power_buffer;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    UNUSED(huart);
#if USING_UART_IT

    auto& serial = liaohy::serial;

    if (huart == serial.uart) {
        //    liaohy::serial.println("Hello, World!");
        serial.tx_buffer = serial.rx_buffer->read(reinterpret_cast<uint16_t&>(serial.len));
        serial.print();
        //    uint8_t ch[15] = {0};
        //    HAL_UART_Receive_IT(&huart1, (uint8_t *) ch, 15);

        serial.read_it(15);
        //        HAL_UART_Receive_IT(serial.uart, serial.rx_buffer->next(15)->data, 15);
    }

#endif
}

static uint8_t cnt = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UNUSED(Size);
#if USING_UART_IDLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart == interact.remote_control.uartPlus.uart) {
        ++interact.remote_control.uartPlus.rx_cnt;
        interact.remote_control.update();
//        interact.remote_control.update(interact.key_board);
        if (++cnt > 5) {
            rc_ready.store(true);
            xEventGroupSetBitsFromISR(osEventGroup, REMOTE_CONTROL_START_EVENT, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        if (interact.remote_control.rcInfo.right == 2 && interact.remote_control.rcInfo.left == 2)
            osThreadResume(ERROR_TASKHandle);  // 左右摇杆拨下，进入急停任务

    } else if (huart == interact.image_trans.uartPlus.uart) {
        using namespace crc;
        ++interact.image_trans.uartPlus.rx_cnt;
        interact.image_trans.rb.write_data(interact.image_trans.uartPlus.rx_buffer, Size);
        interact.image_trans.start_receive();
    } else if (huart == interact.sub_board.uartPlus.uart) {
        ++interact.sub_board.uartPlus.rx_cnt;
        interact.sub_board.get_feedback();
        interact.sub_board.start_receive();
    } else if (huart == ui.uartPlus.uart)  // UART7
    {
//        using namespace crc;
//        for (uint8_t i = 0; i < Size - 9; ++i)
//        {
//            auto data = &ui.uartPlus.rx_buffer[i];
//            if (data[0] == 0xA5)
//            {
//                uint16_t len = (data[2] << 8 | data[1]);
//                auto rx_cmd_id = data[6] << 8 | data[5];
//                if (verify_crc16_check_sum(data, len + 9))
//                {
//                    switch (rx_cmd_id)
//                    {
//                        case 0x202:
//                            power_buffer = static_cast<uint16_t>(data[16]) << 8 | data[15];
//                        default:
//                            break;
//                    }
//                }
//            }
//        }
//				ui.start_receive();
//    }
        using namespace crc;
        ++ui.uartPlus.rx_cnt;
        for (int i = 0; i < Size - 9; ++i) {
            auto data = &ui.uartPlus.rx_buffer[i];
            if (data[0] == 0xA5) {
                uint16_t len = (data[2] << 8 | data[1]);
                auto rx_cmd_id = data[6] << 8 | data[5];
                if (verify_crc16_check_sum(data, len + 9)) {
                    switch (rx_cmd_id) {
                        case 0x201:
                            ui.ui_frame->data_frame.sender_id = data[7];
                            switch (data[7]) {
                                case 2:
                                    ui.ui_frame->data_frame.receiver_id = 0x102;
                                    break;
                                case 102:
                                    ui.ui_frame->data_frame.receiver_id = 0x166;
                                    break;
                            }
                            ui.is_get_id.store(true);
                            break;
                        case 0x301:
                            ++ui.rx_cnt;
                            break;
                        case 0x306:
                            break;
                        case 0x309:
                            break;
												case 0x202:
														power_buffer = static_cast<uint16_t>(data[16]) << 8 | data[15];
                        default:
                            break;
                    }
                    i += len + 8;
                }
            }
        }
        ui.start_receive();
    }
    else if (huart == roboArm.joint1.uart.uart) {
        roboArm.joint1.get_feed_back(roboArm.joint1.uart.rx_buffer);
        roboArm.joint1.start();
    }
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == interact.sub_board.uartPlus.uart) {
        ++interact.sub_board.uartPlus.err_cnt;
        interact.sub_board.start_receive();
    } else if (huart == interact.image_trans.uartPlus.uart) {
        ++interact.image_trans.uartPlus.err_cnt;
        interact.image_trans.start_receive();
    } else if (huart == interact.remote_control.uartPlus.uart) {
        ++interact.remote_control.uartPlus.err_cnt;
    } else if (huart == ui.uartPlus.uart) {
        ++ui.uartPlus.err_cnt;
        ui.start_receive();
    } else if (huart == roboArm.joint1.uart.uart) {
        ++roboArm.joint1.uart.err_cnt;
        roboArm.joint1.start();
        // 重新启动 DMA 接收
    }
};

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == interact.sub_board.uartPlus.uart) {
        ++interact.sub_board.uartPlus.err_cnt;
        interact.sub_board.start_receive();
    } else if (huart == interact.image_trans.uartPlus.uart) {
        ++interact.image_trans.uartPlus.err_cnt;
    } else if (huart == interact.remote_control.uartPlus.uart) {
        ++interact.remote_control.uartPlus.uart;
    } else if (huart == ui.uartPlus.uart) {
        ++ui.uartPlus.err_cnt;
        ui.start_receive();
    }
};
