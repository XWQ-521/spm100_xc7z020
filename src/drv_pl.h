/*
 * drv_pl.h
 *
 *  Created on: 2025年10月24日
 *      Author: dulc
 */

#ifndef SRC_DRV_DRV_PL_H_
#define SRC_DRV_DRV_PL_H_
#include <stdio.h>


#define WRITE_ADDR_START1		0x10000000
#define WRITE_ADDR_END1			0x1000002F

// 写入地址定义 (Write Address Definitions)
#define WRITE_ADDR_MOTOR1       0x10000000  // 电机1控制寄存器地址
#define WRITE_ADDR_MOTOR2       0x10000004  // 电机2控制寄存器地址
#define WRITE_ADDR_MOTOR3       0x10000008  // 电机3控制寄存器地址
#define WRITE_ADDR_MOTOR4       0x1000000C  // 电机4控制寄存器地址
#define WRITE_ADDR_MOTOR5       0x10000010  // 电机5控制寄存器地址
#define WRITE_ADDR_LIGHT        0x10000014  // 灯光控制寄存器地址

#define WRITE_ADDR_FOCUS_Y		0x10000018	// Y对焦位置配置
#define WRITE_ADDR_FOCUS_X		0x10000019	// X对焦位置配置
#define WRITE_ADDR_FOCUS_Z		0x1000001E	// X对焦位置配置		// 小端模式原因导致

#define WRITE_ADDR_CMOS_EXPO	0x1000001A	// CMOS曝光参数
#define WRITE_ADDR_MOTOR_LIMIT	0x1000001C	// 电机限位状态
#define WRITE_ADDR_MICRO_FOCUS	0x1000001D	// 精对焦状态

#define WRITE_ADDR_FOCUS_PARAM	0x10000020	// 对焦参数
#define WRITE_ADDR_SLIT_PARAM	0x10000024  // 裂隙参数
#define WRITE_ADDR_COORD_XYZ	0x10000025	// xyz允差参数
#define WRITE_ADDR_PSD_EXPO		0x10000026
//#define WRITE_ADDR_BUTTON_TRIG	0x10000027	// 按键拍照指令

#define WRITE_ADDR_TRIGGER		0x10000028	// 触发拍照
#define WRITE_ADDR_MEASURE		0x10000029  // 测量模式
#define WRITE_ADDR_SEMI_AUTO    0x1000002A  // 半自动模式
#define WRITE_ADDR_SLIT_PWM		0x1000002B  // 裂隙灯周期占空比参数


#define READ_ADDR_START1		0x20000000
#define READ_ADDR_END1			0x20001000

#define READ_ADDR_START2		0x30000000
#define READ_ADDR_END2			0x300000FF


#define READ_ADDR_CMOS_DATA		0x20000000
#define READ_ADDR_XY_DATA		0x20000400


// 读取地址定义 (Read Address Definitions)
#define READ_ADDR_PSD           0x20000400  // PSD传感器数据地址
#define READ_ADDR_CMOS          0x20000408  // CMOS图像传感器数据地址

#define READ_ADDR_ENCODE_MOTOR1 0x30000000  // 电机1编码器反馈地址
#define READ_ADDR_ENCODE_MOTOR2 0x30000004  // 电机2编码器反馈地址
#define READ_ADDR_ENCODE_MOTOR3 0x30000008  // 电机3编码器反馈地址
#define READ_ADDR_ENCODE_MOTOR4 0x3000000C  // 电机4编码器反馈地址
#define READ_ADDR_ENCODE_MOTOR5 0x30000010  // 电机5编码器反馈地址
//#define READ_ADDR_PHOTO_DONE    0x30000016  // 图像采集完成标志地址
//#define READ_ADDR_BUTTON        0x30000017  // 按键状态读取地址
#define READ_ADDR_PSD_READY		0x30000014	// PSD数据准备就绪
#define READ_ADDR_TAKE_PHOTO	0x30000015	// PL端拍照

#define READ_ADDR_XYZ_COORD		0x30000018  // XYZ最佳位置坐标

// 错误返回值定义 (Error Return Values)
#define READ_ERROR_U8           0xFF        // u8 类型错误返回值
#define READ_ERROR_U16          0xFFFF      // u16 类型错误返回值
#define READ_ERROR_U32          0xFFFFFFFF  // u32 类型错误返回值




void pl_write_u8( uint32_t addr, uint8_t data );

void pl_write_u16( uint32_t addr, uint16_t data );

void pl_write_u32( uint32_t addr, uint32_t data );

uint8_t pl_read_u8( uint32_t addr );

uint16_t pl_read_u16( uint32_t addr );

uint32_t pl_read_u32( uint32_t addr );

int pl_read_u16_array(uint32_t addr, uint16_t *buf, uint16_t count);

int pl_read_u32_array(uint32_t addr, uint32_t *buf, uint16_t count);

#endif /* SRC_DRV_DRV_PL_H_ */
