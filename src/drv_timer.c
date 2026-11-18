/*
 * drv_timer.c
 *
 *  Created on: 2025年9月11日
 *      Author: dulc
 */

// global_timer.c
// Zynq PS Global Timer Driver Implementation
// Author: User + Assistant
// Reference: Zynq-7000 TRM (UG585) Chapter 11

#include "drv_timer.h"
#include "xparameters.h"
#include "xil_io.h"
#include "xparameters_ps.h"



// PS 系统控制器基地址
//#define XPS_SYS_CTRL_BASEADDR   0xF8000000

// 或者手动定义：
#define XSCUGTIMER_BASEADDR   0xF8F00200
#define GTIMER_COUNTER_LOW    (XSCUGTIMER_BASEADDR + 0x00)
#define GTIMER_COUNTER_HIGH   (XSCUGTIMER_BASEADDR + 0x04)
#define GTIMER_CONTROL        (XSCUGTIMER_BASEADDR + 0x08)

// 控制寄存器位定义
#define GTIMER_CONTROL_ENABLE   0x01  // 使能计数
#define GTIMER_CONTROL_COMP_EN  0x02  // 比较中断使能
#define GTIMER_CONTROL_AUTO_INC 0x04  // 自动重载（用于中断）



/**
 * @brief 读取 Global Timer 的当前 64 位计数值
 * @return 64 位计数值
 */
u64 GlobalTimer_GetCount(void) {
    u32 low, high, high2;
    do {
        high = Xil_In32(GTIMER_COUNTER_HIGH);
        low  = Xil_In32(GTIMER_COUNTER_LOW);
        high2 = Xil_In32(GTIMER_COUNTER_HIGH);
    } while (high != high2);  // 防止 32 位溢出导致数据不一致

    return (((u64)high) << 32) | low;
}

/**
 * @brief 获取当前时间（单位：微秒）
 * @return 时间（微秒）
 */
u64 GlobalTimer_GetTimeUs(void) {
    u64 count = GlobalTimer_GetCount();
    return (count * 1000000ULL) / GLOBAL_TIMER_FREQ_HZ;
}

/**
 * @brief 获取当前时间（单位：秒，浮点数）
 * @return 时间（秒）
 */
double GlobalTimer_GetTimeSec(void) {
    u64 count = GlobalTimer_GetCount();
    return (double)count / (double)GLOBAL_TIMER_FREQ_HZ;
}

/**
 * @brief 使能 Global Timer 计数
 */
void GlobalTimer_Enable(void) {
    u32 ctrl = Xil_In32(GTIMER_CONTROL);
    Xil_Out32(GTIMER_CONTROL, ctrl | GTIMER_CONTROL_ENABLE);
}

/**
 * @brief 禁用 Global Timer 计数
 */
void GlobalTimer_Disable(void) {
    u32 ctrl = Xil_In32(GTIMER_CONTROL);
    Xil_Out32(GTIMER_CONTROL, ctrl & ~GTIMER_CONTROL_ENABLE);
}



void GlobalTimer_Init(void) {
    // 可选：清零计数器
    Xil_Out32(GTIMER_COUNTER_LOW,  0);
    Xil_Out32(GTIMER_COUNTER_HIGH, 0);

    // 使能计数器
    Xil_Out32(GTIMER_CONTROL, GTIMER_CONTROL_ENABLE);
}




