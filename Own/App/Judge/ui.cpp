#include "ui.hpp"




void UI::delete_layer(operate_delete_layer delete_layer, uint8_t layer_id) {
    interaction_layer_delete_t layer_delete {.delete_type = static_cast<uint8_t>(delete_layer), .layer = layer_id};
    lock();
    type = UI::types::DELETE;
    memcpy(ui_frame->data_frame.user_data, &layer_delete, sizeof(interaction_layer_delete_t));
    num = 0;
    len = sizeof(interaction_layer_delete_t);
    unlock();
}

void UI::add_frame_header() {
    switch (type) {
        case UI::types::DELETE:
            ui_frame->data_frame.data_cmd_id = 0x100;
            ui_frame->header.data_length     = len + 6;
            crc::append_crc8_check_sum(reinterpret_cast<uint8_t*>(&(ui_frame->header)), sizeof(frame_header));
            crc::append_crc16_check_sum(reinterpret_cast<uint8_t*>(ui_frame), len + 15);
            break;
        case UI::types::STRING:
            ui_frame->data_frame.data_cmd_id = 0x110;
            ui_frame->header.data_length     = len + 6;
            crc::append_crc8_check_sum(reinterpret_cast<uint8_t*>(&(ui_frame->header)), sizeof(frame_header));
            crc::append_crc16_check_sum(reinterpret_cast<uint8_t*>(ui_frame), len + 15);
            break;
        case UI::types::FIGURE:
            switch (num) {
                case 1:
                    ui_frame->data_frame.data_cmd_id = 0x101;
                    ui_frame->header.data_length     = len + 6;
                    crc::append_crc8_check_sum(reinterpret_cast<uint8_t*>(&(ui_frame->header)), sizeof(frame_header));
                    crc::append_crc16_check_sum(reinterpret_cast<uint8_t*>(ui_frame), len + 15);
                    break;
                case 2:
                    ui_frame->data_frame.data_cmd_id = 0x102;
                    ui_frame->header.data_length     = len + 6;
                    crc::append_crc8_check_sum(reinterpret_cast<uint8_t*>(&(ui_frame->header)), sizeof(frame_header));
                    crc::append_crc16_check_sum(reinterpret_cast<uint8_t*>(ui_frame), len + 15);
                    break;
                case 3:
                    memset(&(ui_frame->data_frame.user_data[len]), 0, 30);
                    len                              += 30;
                    ui_frame->data_frame.data_cmd_id  = 0x103;
                    ui_frame->header.data_length      = len + 6;
                    crc::append_crc8_check_sum(reinterpret_cast<uint8_t*>(&(ui_frame->header)), sizeof(frame_header));
                    crc::append_crc16_check_sum(reinterpret_cast<uint8_t*>(ui_frame), len + 15);
                    break;
                case 4:
                    memset(&ui_frame->data_frame.user_data[len], 0, 15);
                    len                              += 15;
                    ui_frame->data_frame.data_cmd_id  = 0x103;
                    ui_frame->header.data_length      = len + 6;
                    crc::append_crc8_check_sum(reinterpret_cast<uint8_t*>(&(ui_frame->header)), sizeof(frame_header));
                    crc::append_crc16_check_sum(reinterpret_cast<uint8_t*>(ui_frame), len + 15);
                    break;
                case 5:
                    ui_frame->data_frame.data_cmd_id = 0x103;
                    ui_frame->header.data_length     = len + 6;
                    crc::append_crc8_check_sum(reinterpret_cast<uint8_t*>(&(ui_frame->header)), sizeof(frame_header));
                    crc::append_crc16_check_sum(reinterpret_cast<uint8_t*>(ui_frame), len + 15);
                    break;
                case 6:
                    memset(&(ui_frame->data_frame.user_data[len]), 0, 15);
                    len                              += 15;
                    ui_frame->data_frame.data_cmd_id  = 0x104;
                    ui_frame->header.data_length      = len + 6;
                    crc::append_crc8_check_sum(reinterpret_cast<uint8_t*>(&(ui_frame->header)), sizeof(frame_header));
                    crc::append_crc16_check_sum(reinterpret_cast<uint8_t*>(ui_frame), len + 15);
                    break;
                case 7:
                    ui_frame->data_frame.data_cmd_id = 0x104;
                    ui_frame->header.data_length     = len + 6;
                    crc::append_crc8_check_sum(reinterpret_cast<uint8_t*>(&(ui_frame->header)), sizeof(frame_header));
                    crc::append_crc16_check_sum(reinterpret_cast<uint8_t*>(ui_frame), len + 15);
                    break;
            }
            break;
        case types::NONE: break;
        default: break;
    }
}
bool UI::update() {
    if (type!=types::DELETE) {
        for (uint8_t i = 0; i < str_index; i++) {
            if (num < 7 && str_list[i].control.display_num > 0) {
                if (str_list[i].control.visible == true) {
                    type = types::STRING;
                    memcpy(&((ui_frame->data_frame.user_data)[len]), &str_list[i], 15);
                    len += 15;
                    memcpy(&((ui_frame->data_frame.user_data)[len]), str_list[i].graphic.str,
                           str_list[i].graphic.details_b);
                    memset(&((ui_frame->data_frame.user_data)[len]) + str_list[i].graphic.details_b, 0,
                           30 - str_list[i].graphic.details_b);
                    len += 30;
                    ++num;
                    str_list[i].control.display_num--;
                    str_list[i].control.visible = false;
                    break;
                }
                str_list[i].control.display_num--;
            }
        }

        if (type!=types::STRING) {
            for (uint8_t i = 0; i < figure_index; i++) {
                if (num < 7 && figure_list[i].control.display_num > 0) {
                    if (figure_list[i].control.visible == true) {
                        type = types::FIGURE;
                        memcpy(&((ui_frame->data_frame.user_data)[len]), &figure_list[i], 15);
                        len += 15;
                        ++num;
                        figure_list[i].control.visible = false;
                    }
                    figure_list[i].control.display_num--;
                }
            }
        }
    }
    this->add_frame_header();
    uartPlus.transmit_dma_pdata(uartPlus.tx_buffer, len + 15);
    const auto no_empty = (len != 0);
    this->clear();
    return no_empty;
}
void UI::start_receive() { uartPlus.receive_dma_idle(200); }
