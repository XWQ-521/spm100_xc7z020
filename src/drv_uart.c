/*
 * drv_uart.c
 *
 *  Created on: 2025年11月5日
 *      Author: dulc
 */



#include "drv_uart.h"



#define UART_DEVICE_ID XPAR_XUARTPS_1_DEVICE_ID     //串口设备 ID
#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID //中断 ID
#define UART_INT_IRQ_ID XPAR_XUARTPS_1_INTR //串口中断 ID
#define INTC_BASE_ADDR		XPAR_SCUGIC_0_CPU_BASEADDR	// 中断控制器基地址
#define INTC_DIST_BASE_ADDR	XPAR_SCUGIC_0_DIST_BASEADDR	//中断分发器基地址


uart_t m_recv_queue = {{0}, 0, 0};



//UART 初始化函数
int uart_driver_init(XUartPs* uart_ps)
{
	int status;
	XUartPs_Config *uart_cfg;

	uart_cfg = XUartPs_LookupConfig(UART_DEVICE_ID);
	if (NULL == uart_cfg)
	return XST_FAILURE;
	status = XUartPs_CfgInitialize(uart_ps, uart_cfg, uart_cfg->BaseAddress);
	if (status != XST_SUCCESS)
	return XST_FAILURE;

	//UART 设备自检
	status = XUartPs_SelfTest(uart_ps);
	if (status != XST_SUCCESS)
	return XST_FAILURE;

	//设置工作模式:正常模式
	XUartPs_SetOperMode(uart_ps, XUARTPS_OPER_MODE_NORMAL);
	//设置波特率:115200
	XUartPs_SetBaudRate(uart_ps,115200);
	//设置 RxFIFO 的中断触发等级
	XUartPs_SetFifoThreshold(uart_ps, 1);

    m_recv_queue.head = 0;
    m_recv_queue.tail = 0;

	return XST_SUCCESS;
}




//int uart_recv_count = 0;

////UART 中断处理函数
//void uart_intr_handler(void *call_back_ref)
//{
//	XUartPs *uart_instance_ptr = (XUartPs *) call_back_ref;
//	u32 rec_data = 0 ;
//	u32 isr_status ; //中断状态标志
//    uint16_t tmp;
//
//	//读取中断 ID 寄存器，判断触发的是哪种中断
//	isr_status = XUartPs_ReadReg(uart_instance_ptr->Config.BaseAddress,
//	XUARTPS_IMR_OFFSET);
//	isr_status &= XUartPs_ReadReg(uart_instance_ptr->Config.BaseAddress,
//	XUARTPS_ISR_OFFSET);
//
//	//判断中断标志位 RxFIFO 是否触发
//	if (isr_status & (u32)XUARTPS_IXR_RXOVR){
//		rec_data = XUartPs_RecvByte(XPAR_PS7_UART_1_BASEADDR);
//		//清除中断标志
//		XUartPs_WriteReg(uart_instance_ptr->Config.BaseAddress,
//		XUARTPS_ISR_OFFSET, isr_status) ;
//
//		uart_recv_count++;
//
//        // 计算下一个队列尾部位置
//        tmp = (m_recv_queue.tail + 1);
//        // 如果超出缓冲区大小，则循环回到起始位置
//        if (tmp >= UART_MAX_BUF) {
//            tmp = 0;
//        }
//
//        // 检查队列是否已满
//        if (tmp == m_recv_queue.head){
//            // 如果队列已满，则不存入新数据
//            return;
//        }
//
//        // 将接收到的数据存入队列
//        m_recv_queue.buf[m_recv_queue.tail] = rec_data;
//        // 更新队列尾部位置
//        m_recv_queue.tail = tmp;
//
//
//	}
//	//XUartPs_SendByte(XPAR_PS7_UART_1_BASEADDR,rec_data);
//}



void uart_intr_handler(void *call_back_ref)
{
    XUartPs *uart_instance_ptr = (XUartPs *)call_back_ref;
    u32 isr_status;
    u32 base = uart_instance_ptr->Config.BaseAddress;

    isr_status = XUartPs_ReadReg(base, XUARTPS_ISR_OFFSET);
    XUartPs_WriteReg(base, XUARTPS_ISR_OFFSET, isr_status);

    while (XUartPs_IsReceiveData(base)) {
        u8 data = XUartPs_ReadReg(base, XUARTPS_FIFO_OFFSET);
        uint16_t next_tail = (m_recv_queue.tail + 1) % UART_MAX_BUF;
        if (next_tail != m_recv_queue.head) {
            m_recv_queue.buf[m_recv_queue.tail] = data;
            m_recv_queue.tail = next_tail;
        }
        // 队列满时静默丢弃，严禁在中断里 printf！
    }
}




//
// //串口中断初始化
// int uart_intr_init(XScuGic *intc, XUartPs *uart_ps)
// {
//	 int status;
//	// 初始化中断控制器
//	 XScuGic_Config *intc_cfg;
//	 intc_cfg = XScuGic_LookupConfig(INTC_DEVICE_ID);
//	 if (NULL == intc_cfg)
//	 return XST_FAILURE;
//	 status = XScuGic_CfgInitialize(intc, intc_cfg,
//	 intc_cfg->CpuBaseAddress);
//	 if (status != XST_SUCCESS)
//	 return XST_FAILURE;
////
////	 //设置并打开中断异常处理功能
////	 Xil_ExceptionInit();
////	 Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
////	 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
////	 (void *)intc);
////	 Xil_ExceptionEnable();
//
//	//为中断设置中断处理函数
//	 XScuGic_Connect(intc, UART_INT_IRQ_ID,
//	 (Xil_ExceptionHandler) uart_intr_handler,(void *) uart_ps);
////	 //设置 UART 的中断触发方式
//	 XUartPs_SetInterruptMask(uart_ps,  XUARTPS_IXR_RXFULL | XUARTPS_IXR_RXOVR);
////	 //使能 GIC 中的串口中断
//	 XScuGic_Enable(intc, UART_INT_IRQ_ID);
//	 return XST_SUCCESS;
// }


//#include "xscugic.h"
//#include "xparameters.h"
//
//// 假设你已经定义了这些（与定时器一致）
//#define INTC_BASE_ADDR      XPAR_SCUGIC_0_CPU_BASEADDR
//#define INTC_DIST_BASE_ADDR XPAR_SCUGIC_0_DIST_BASEADDR
//
//extern XUartPs UartInstance; // 你的 UART 实例，需全局或传入

int uart_intr_init(XScuGic *intc, XUartPs *uart_ps)
{
    // 1. 设置 UART 中断掩码（使能接收满和溢出中断）
    XUartPs_SetInterruptMask(uart_ps, XUARTPS_IXR_RXFULL | XUARTPS_IXR_RXOVR);

    // 2. 注册串口中断处理函数（使用旧式 API！）
    XScuGic_RegisterHandler(
        INTC_BASE_ADDR,
        UART_INT_IRQ_ID,                     // 中断号，如 59
        (Xil_ExceptionHandler)uart_intr_handler,
        (void *)uart_ps                // 传给 ISR 的参数
    );

    // 3. 使能该中断（旧式）
    XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, UART_INT_IRQ_ID);

    return XST_SUCCESS;
}



