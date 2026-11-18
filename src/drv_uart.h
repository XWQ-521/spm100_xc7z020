/*
 * drv_uart.h
 *
 *  Created on: 2025年11月5日
 *      Author: dulc
 */

#ifndef SRC_DRV_UART_H_
#define SRC_DRV_UART_H_
#include"stdint.h"
#include "xparameters.h"
#include "xuartps.h"
#include "xil_printf.h"
#include "xscugic.h"
#include "stdio.h"


//串口波特率
#define USART_BAUDRATE          115200
#define UART_MAX_BUF            1024


typedef struct
{
    uint8_t buf[UART_MAX_BUF];
    uint16_t head;
    uint16_t tail;
} uart_t;



extern uart_t m_recv_queue;



int uart_driver_init(XUartPs* uart_ps);
//UART 中断处理函数
void uart_intr_handler(void *call_back_ref);
 //串口中断初始化
 int uart_intr_init(XScuGic *intc, XUartPs *uart_ps);


#endif /* SRC_DRV_UART_H_ */
