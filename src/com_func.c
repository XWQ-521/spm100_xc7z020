/*
 * com_func.c
 *
 *  Created on: 2025年10月26日
 *      Author: dulc
 */
#include "com_func.h"
#include "xil_printf.h"

/**
 * @brief 计算CRC校验值
 *
 * @param buf 数据缓冲区
 * @param data_len 数据长度
 * @return uint16_t 计算出的CRC值
 */
uint16_t gen_crc(uint8_t *buf, uint16_t data_len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < data_len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    // 高低字节交换后返回
    return (crc << 8) | (crc >> 8);
}

/**
 * @brief 验证数据包完整性
 *
 * @param data 数据包指针
 * @param len 数据包长度
 * @return int 验证成功返回0，失败返回负数错误码
 */
int validate_packet(uint8_t *data, uint32_t len)
{
    // 最小数据包长度检查
    if (len < 9) return -1;

    // 帧头和帧尾标识检查
    if (data[0] != 0xAA || data[1] != 0x55 || data[len - 3] != 0xCC) {
        xil_printf("err_data\r\n");
        return -2;
    }

    // 数据长度字段验证
    uint8_t data_len = data[3];
    if (len != (uint32_t)(data_len + 8)) {
        xil_printf("err_len\r\n");
        return -3; // 总长 = Len字段 + 固定7字节(含头尾)
    }

    // CRC校验
    uint16_t crc_received = (data[len - 2] << 8) | data[len - 1];
    uint16_t crc_calc = gen_crc(data+2, len - 4); // 不包含 CRC 和结尾 CC BC

    if (crc_received != crc_calc) {
        xil_printf("err_crc\r\n");
        return -4;
    }

    return 0;
}
