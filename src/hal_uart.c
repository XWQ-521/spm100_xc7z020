/*
 * hal_uart.c
 *
 *  Created on: 2025年11月6日
 *      Author: dulc
 */

#include "hal_uart.h"
#include "drv_uart.h"
#include "joystick_iap.h"
// 定义通信协议中的特殊字节常量
#define SYNC_0                      0xAA  // 同步字节0
#define SYNC_1                      0x01  // 同步字节1
#define ETX                         0x55  // 结束字节


typedef enum {
	SYNC_STATE_0 = 0,
	SYNC_STATE_1,
	DATA_STATE,
	FCS_STATE_0,
	ETX_STATE,
}	unpack_e;


typedef struct{
    uint8_t  buf[UART_MAX_BUF];
    uint8_t  state;
    uint16_t tmp_len;
    uint16_t fcs;
    uint16_t deal_flag;
}   pkt_ctrl_t;


XScuGic Intc; //中断控制器驱动程序实例
XUartPs Uart_Ps; //串口驱动程序实例
// 全局数据包处理回调函数指针，初始化为NULL
static uart_packet_handler_t g_uart_packet_handler = NULL;

pkt_ctrl_t m_pkt_ctrl = { {0}, SYNC_STATE_0, 0, 0, 0 };




int uart_init( void )
{
	int status;

	status = uart_driver_init(&Uart_Ps); //串口初始化
	if (status == XST_FAILURE) {
		xil_printf("串口初始化失败\r\n");
		return XST_FAILURE;
	}

	uart_intr_init(&Intc, &Uart_Ps); //串口中断初始化
	xil_printf("串口初始化成功\r\n");

	return XST_SUCCESS;
}

void joystick_uart_send_byte(uint8_t byte)
{
    while (XUartPs_IsTransmitFull(Uart_Ps.Config.BaseAddress));
    XUartPs_WriteReg(Uart_Ps.Config.BaseAddress, XUARTPS_FIFO_OFFSET, byte);
}

void joystick_uart_send_bytes(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        joystick_uart_send_byte(buf[i]);
    }
}

/**
 * @brief 注册数据包处理回调函数
 *
 * @param handler 回调函数指针
 */
void register_uart_packet_handler(uart_packet_handler_t handler)
{
	g_uart_packet_handler = handler;
}

uint8_t g_joystick_version = 0;
uint8_t g_joystick_ver_updated = 0; // 打印标志位，防止主循环死刷日志

void protocol_unpack( void )
{
    uint8_t ch;

    while ( m_recv_queue.head != m_recv_queue.tail )
    {
    	if ( g_is_joystick_iap_active )
		{
    		ch = m_recv_queue.buf[m_recv_queue.head++];
			if ( m_recv_queue.head >= UART_MAX_BUF )
				m_recv_queue.head = 0;

			// 抓到 APP 确认帧(0x82)、YModem 握手('C'/ACK/NAK/CAN) 或非坐标数据时打印
			if (ch == 0x82 || ch == 0x28 || ch == 0x06 || ch == 0x43 || ch == 0x15 || ch == 0x18) {
				xil_printf("[IAP_RX] 0x%02X (%c)\r\n", ch, (ch >= 32 && ch <= 126) ? ch : '.');
				joystick_iap_feed_rx_byte(ch);
			}
			//joystick_iap_feed_rx_byte(ch);
			continue;
		}
        // to avoid block process,add a flag, if flag equal 1,break
        if ( m_pkt_ctrl.deal_flag == 1 )
        {
            m_pkt_ctrl.deal_flag = 0;
            break;
        }

        ch = m_recv_queue.buf[m_recv_queue.head++];
        if ( m_recv_queue.head >= UART_MAX_BUF )
            m_recv_queue.head = 0;

        switch ( m_pkt_ctrl.state )
        {
        case SYNC_STATE_0:
            if ( ch == SYNC_0 )
                m_pkt_ctrl.state = SYNC_STATE_1;
            break;

        case SYNC_STATE_1:
        	if ( ch == 0x01 || ch == 0x05 || ch == 0x06 || ch == 0x07|| ch == 0x08|| ch == 0x09||ch == 0x0A||ch == 0x0B)
			{
				g_joystick_version = ch;
				g_joystick_ver_updated = 1;

				m_pkt_ctrl.state = DATA_STATE;
				m_pkt_ctrl.tmp_len = 0;
				m_pkt_ctrl.fcs = 0;
				m_pkt_ctrl.fcs ^= SYNC_0;
				m_pkt_ctrl.fcs ^= ch;
			}
			else
			{
				// 不是 0x01 也不是 0x05，说明前一个 0xAA 是坐标/校验伪造的，立刻退回寻找真包头
				// 特殊处理：若当前字节恰好又是 0xAA，直接保持在 SYNC_STATE_1
				if ( ch == SYNC_0 )
					m_pkt_ctrl.state = SYNC_STATE_1;
				else
					m_pkt_ctrl.state = SYNC_STATE_0;
			}
			break;

        case DATA_STATE:
            m_pkt_ctrl.buf[m_pkt_ctrl.tmp_len++] = ch;
            m_pkt_ctrl.fcs ^= ch;
            // 固定长度
            if(m_pkt_ctrl.tmp_len >= 8 )
                m_pkt_ctrl.state = FCS_STATE_0;
            break;


        case FCS_STATE_0:
            if ( ch != (uint8_t)(m_pkt_ctrl.fcs) )
                m_pkt_ctrl.state = SYNC_STATE_0;
            else
                m_pkt_ctrl.state = ETX_STATE;
            break;

        case ETX_STATE:
            if ( ch != ETX )
                m_pkt_ctrl.state = SYNC_STATE_0;
            else
            {
            	g_uart_packet_handler( m_pkt_ctrl.buf, m_pkt_ctrl.tmp_len );
                m_pkt_ctrl.deal_flag = 1;
                //m_pkt_ctrl.state = SYNC_STATE_0;
            }
            break;


        default:
            m_pkt_ctrl.state = SYNC_STATE_0;
            break;
        }
    }
}




