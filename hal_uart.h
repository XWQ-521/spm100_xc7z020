/*
 * hal_uart.h
 *
 *  Created on: 2025年11月6日
 *      Author: dulc
 */

#ifndef SRC_HAL_UART_H_
#define SRC_HAL_UART_H_
#include "stdint.h"

// 定义回调函数类型
typedef void (*uart_packet_handler_t)(uint8_t *data, uint32_t len);
extern uint8_t g_joystick_version;
extern uint8_t g_joystick_ver_updated;

// 供 IAP 调用的底层发送接口
void joystick_uart_send_byte(uint8_t byte);
void joystick_uart_send_bytes(const uint8_t *buf, uint16_t len);
int uart_init( void );

void register_uart_packet_handler(uart_packet_handler_t handler);

void protocol_unpack( void );

#endif /* SRC_HAL_UART_H_ */
