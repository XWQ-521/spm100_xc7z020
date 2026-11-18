/*
 * hal_timer.c
 *
 *  Created on: 2025年9月12日
 *      Author: dulc
 */
#include "hal_timer.h"
#include "drv_timer.h"



#define GTIMER_1US_MAX_COUNT    128849020   // count (0xFFFF FFFF FFFF FFFF * 1000000 / 33333333)


void timer_init( void )
{
    GlobalTimer_Init();
    GlobalTimer_Enable();
}



uint64_t timer_get_1us( void )
{
    return GlobalTimer_GetTimeUs();
}



/**
 * 计算两次时间读数之间的差异，单位为1微秒。
 *
 * 该函数旨在用于计算从start_time到end_time的时间差，考虑到时间计数器可能在此期间溢出的情况。
 * 当end_time大于或等于start_time时，直接计算差值。如果end_time小于start_time，则假设计数器已溢出，
 * 并相应地计算差值。
 *
 * @param start_time 时间测量的开始时刻。
 * @param end_time 时间测量的结束时刻。
 * @return 从start_time到end_time的时间差，单位为1微秒。
 */
uint64_t timer_get_diff_1us( uint64_t start_time, uint64_t end_time )
{
    uint64_t start, end;

    start = start_time;
    end = end_time;

    if ( end >= start )
    {
        // 如果结束时间大于或等于开始时间，直接返回时间差。
        return (end - start);
    }
    else
    {
        // 如果结束时间小于开始时间，说明计数器溢出，需要加上最大计数值再减去开始时间。
        return ( end + GTIMER_1US_MAX_COUNT - start );
    }
}



void timer_delay_1us( uint32_t time )
{
    uint64_t start = timer_get_1us();

    while( 1 )
    {
        if ( timer_get_diff_1us( start, timer_get_1us() ) >= time )
            break;
    }
}

