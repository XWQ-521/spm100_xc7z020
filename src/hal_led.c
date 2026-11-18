/*
 * hal_led.c
 *
 *  Created on: 2025年10月25日
 *      Author: dulc
 */
#include "drv_pl.h"
#include "hal_led.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "hal_timer.h"
#include "xil_io.h"
/**
 * @brief 控制LED灯
 * 
 * @param cmd 控制命令
 * @param id LED编号
 * @param value LED亮度值
 */
void led_control(CMD_T cmd, uint8_t id, uint8_t value)
{
    uint32_t write_val = 0;

    // 构造32位控制数据，按位打包
    write_val += cmd;
    write_val = write_val << 8;
    write_val += id;
    write_val = write_val << 8;
    write_val += value;
    write_val = write_val << 8;
    write_val += 0;

    xil_printf("灯控制：  指令码：  %d;	灯ID：  %d;	亮度值：  %d;	timer:%lld   \r\n", cmd, id, value, timer_get_1us());
    pl_write_u32(WRITE_ADDR_LIGHT, write_val);

    uint32_t read_val_2 = Xil_In32(0x10000014);
    uint8_t read_val_3 = Xil_In8(0x1000002C);
    xil_printf("[HW_IO] Addr: 0x%08X,  Get: 0x%X;Addr: 0x%08X,  Get: 0x%X\r\n", WRITE_ADDR_LIGHT, read_val_2,0x1000002C,read_val_3);

}


// 控制所有对齐灯光
void led_align_all_control(uint8_t value)
{
    led_control(CMD_ALIGN_RIGHT, 0, value);
    timer_delay_1us(100);
    led_control(CMD_ALIGN_LEFT, 0, value);
    timer_delay_1us(100);
    led_control(CMD_CENTER, 0, value);
    xil_printf("灯控制：  指令码：  0x2C;	亮度值：  %d;	timer:%lld   \r\n", value, timer_get_1us());
}


// 控制所有补光灯光
void led_fill_light_all_control(uint8_t value)
{
#define FILL_LIGHT_COUNT 6
    int i = 0;

    for (i = 0; i < FILL_LIGHT_COUNT; i++) {
        led_control(CMD_FILL_LIGHT, i, value);
        timer_delay_1us(100);
    }
    xil_printf("灯控制：  指令码：  0x2D;	亮度值：  %d;	timer:%lld   \r\n", value, timer_get_1us());
}



