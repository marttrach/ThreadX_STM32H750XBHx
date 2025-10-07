#include "modbus_rtu_client.h"
// #include "iot.h"

uint8_t MODBUS_CRC_LENGTH = 0x02;
uint8_t MODBUS_ERROR_BIAS = 0x80;
uint8_t MODBUS_RESPONSE_HDR_LENGTH = 0x02;
uint8_t MODBUS_ERROR_RESP_LEN = 0x05;
uint8_t MODBUS_FIXED_RESP_LEN = 0x08;
uint8_t MODBUS_MBAP_HDR_LENGTH = 0x07;

void modbus_build_request(Modbus_RTU_Frame *req) {
    uint16_t len = 0;
    req->frame[len++] = req->slave_addr;
    req->frame[len++] = req->function_code;

    req->frame[len++] = req->reg_addr >> 8;
    req->frame[len++] = req->reg_addr & 0xFF;

    req->frame[len++] = req->reg_count >> 8;
    req->frame[len++] = req->reg_count & 0xFF;

    if (req->function_code == 0x10) { // 寫多個保持寄存器
        req->byte_count = req->reg_count * 2;
        req->frame[len++] = req->byte_count;
        for (uint16_t i = 0; i < req->reg_count; i++) {
            req->frame[len++] = req->data[i * 2];
            req->frame[len++] = req->data[i * 2 + 1];
        }
    } else if (req->function_code == 0x06) { // 寫單一保持寄存器
        req->frame[len++] = req->data[0];
        req->frame[len++] = req->data[1];
    }

    // 計算 CRC
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= req->frame[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    req->crc = crc;
    req->frame[len++] = crc & 0xFF;
    req->frame[len++] = crc >> 8;
    req->frame_len = len;
}

uint8_t modbus_parse_response(Modbus_RTU_Frame *resp, const uint8_t *raw, uint16_t len) {
    if (len < MODBUS_ERROR_RESP_LEN) return MODBUS_EX_CRC_ERROR;

    // 驗證 CRC
    uint16_t crc_calc = 0xFFFF;
    for (uint16_t i = 0; i < len - 2; i++) {
        crc_calc ^= raw[i];
        for (int j = 0; j < 8; j++) {
            crc_calc = (crc_calc & 1) ? (crc_calc >> 1) ^ 0xA001 : (crc_calc >> 1);
        }
    }
    uint16_t crc_recv = raw[len - 2] | (raw[len - 1] << 8);
    if (crc_calc != crc_recv) return MODBUS_EX_CRC_ERROR;

    if (len == MODBUS_ERROR_RESP_LEN) {
        if ((raw[1] & MODBUS_ERROR_BIAS) == 0) 
            return MODBUS_EX_FORMAT_ERROR;
        else
            return (Modbus_RTU_Exception_Code)raw[2];
    }

    // 填入結構體
    resp->slave_addr    = raw[0];
    resp->function_code = raw[1];
    resp->byte_count    = raw[2];
    memcpy(resp->data, &raw[3], resp->byte_count);
    resp->crc           = crc_recv;
    resp->frame_len     = len;
    memcpy(resp->frame, raw, len);

    // DEBUG_DUMP(IOT_LOG_DEBUG, "response data. content: ");
    // for (int i = 0; i < resp->byte_count; i++)
    //     DEBUG_DUMP(IOT_LOG_DEBUG, "%d|", resp->data[i]);
    // DEBUG_DUMP(IOT_LOG_DEBUG, "response data. length: %d\r\n", resp->byte_count);
    return MODBUS_EX_OK;
}
