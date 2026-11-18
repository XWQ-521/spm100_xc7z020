/*
 * drv_motor.h
 *
 *  Created on: 2025年9月10日
 *      Author: dulc
 */

#ifndef SRC_DRV_DRV_MOTOR_H_
#define SRC_DRV_DRV_MOTOR_H_

// 包含必要的头文件
#include "xil_types.h"
#include "xspips.h"
#include "xgpiops.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "sleep.h"
#include "xparameters.h"
#include "xspips_hw.h"


//#define MOTOR_COMPATIBLE	// FPGA的电机兼容原三轴

// 电机个数定义
#define MOTOR_COUNT 5

// GPIO引脚基地址
#define GPIO_PIN_BASE	54

// SPI片选引脚定义 (EMIO引脚编号 = 基础编号 + 54)
#define SPI0_CS0_PIN 0  // SPI0 CS0对应的引脚编号 54
#define SPI0_CS1_PIN 1  // SPI0 CS1对应的引脚编号 55
#define SPI0_CS2_PIN 2  // SPI0 CS2对应的引脚编号 56
#define SPI0_CS3_PIN 3  // SPI0 CS3对应的引脚编号 57
#define SPI0_CS4_PIN 4  // SPI0 CS4对应的引脚编号 58

// 限位开关引脚定义
#define M1_LIMIT 64
#define M2_LIMIT 65
#define M3_LIMIT 66
#define M4_LIMIT 67
#define M5_LIMIT 68

// TMC2660失速检测引脚定义
#define TMC2660_M1_GPIO_SG_PIN 20
#define TMC2660_M2_GPIO_SG_PIN 21
#define TMC2660_M3_GPIO_SG_PIN 22
#define TMC2660_M4_GPIO_SG_PIN 23
#define TMC2660_M5_GPIO_SG_PIN 24



// 电机地址枚举定义
typedef enum {
    TMC2660_M1 = 1,
    TMC2660_M2,
    TMC2660_M3,
    TMC2660_M4,
    TMC2660_M5,
} MOTOR_ADDR;



// 电机引脚映射枚举 (EMIO引脚 = 基础编号 + 54)
typedef enum {
    M1_CS = SPI0_CS0_PIN + GPIO_PIN_BASE,      M1_SG = TMC2660_M1_GPIO_SG_PIN + GPIO_PIN_BASE,
    M2_CS = SPI0_CS1_PIN + GPIO_PIN_BASE,      M2_SG = TMC2660_M2_GPIO_SG_PIN + GPIO_PIN_BASE,
    M3_CS = SPI0_CS2_PIN + GPIO_PIN_BASE,      M3_SG = TMC2660_M3_GPIO_SG_PIN + GPIO_PIN_BASE,
    M4_CS = SPI0_CS3_PIN + GPIO_PIN_BASE,      M4_SG = TMC2660_M4_GPIO_SG_PIN + GPIO_PIN_BASE,
    M5_CS = SPI0_CS4_PIN + GPIO_PIN_BASE,      M5_SG = TMC2660_M5_GPIO_SG_PIN + GPIO_PIN_BASE,
} MotorPinMap;


typedef struct {
    uint16_t div;                  // 细分数
    uint8_t  current;              // 电流设置值
}	motor_param_t;





int motor_spi_init( void );

int motor_gpio_init( XGpioPs *GpioInst );

void motor_init( MOTOR_ADDR addr, uint16_t subdiv, uint8_t current );

void motor_set_current( MOTOR_ADDR motor_id, uint8_t current );

uint8_t motor_get_current( MOTOR_ADDR motor_id );

void motor_set_subdiv( MOTOR_ADDR motor_id, uint16_t subdiv );

uint16_t motor_get_subdiv( MOTOR_ADDR motor_id );

int motor_addr_check(uint8_t motor_id);                // 检查电机地址有效性
void motor_set_precision_mode(MOTOR_ADDR addr, uint8_t is_low_precision);//修改电机高精度/低精度模式


#endif /* SRC_DRV_DRV_MOTOR_H_ */
