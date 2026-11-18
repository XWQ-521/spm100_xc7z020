/*
 * drv_timer.h
 *
 *  Created on: 2025年9月11日
 *      Author: dulc
 */

#ifndef SRC_DRV_DRV_TIMER_H_
#define SRC_DRV_DRV_TIMER_H_
// global_timer.h
// Zynq PS Global Timer Driver (64-bit counter)
// Compatible with Xilinx Vivado 2018.3 SDK/Vitis


#include "xil_types.h"

// Global Timer 频率 (333.333 MHz)
#define GLOBAL_TIMER_FREQ_HZ      333333333UL


// 函数声明
u64 GlobalTimer_GetCount(void);
u64 GlobalTimer_GetTimeUs(void);
double GlobalTimer_GetTimeSec(void);
void GlobalTimer_Enable(void);
void GlobalTimer_Disable(void);
void GlobalTimer_Init(void);


#endif /* SRC_DRV_DRV_TIMER_H_ */
