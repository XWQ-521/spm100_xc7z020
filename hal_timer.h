/*
 * hal_timer.h
 *
 *  Created on: 2025Äê9ÔÂ12ÈÕ
 *      Author: dulc
 */

#ifndef SRC_HAL_HAL_TIMER_H_
#define SRC_HAL_HAL_TIMER_H_
#include "xil_types.h"



void timer_init( void );

uint64_t timer_get_1us( void );

uint64_t timer_get_diff_1us( uint64_t start_time, uint64_t end_time );

void timer_delay_1us( uint32_t time );

#endif /* SRC_HAL_HAL_TIMER_H_ */
