/*
 * hal_motor.c
 *
 *  Created on: 2025年10月26日
 *      Author: dulc
 */
#include "hal_motor.h"
#include "drv_motor.h"
#include "xil_cache.h"
#include "xgpiops.h"
#include "drv_pl.h"
#include "pid.h"
#include "motorctrl.h"
#include "hal_timer.h"
#include "version.h"
// 电机每转步数（步进电机分辨率）
#define DRV_STEPS_PER_REV       1600
// 编码器每转脉冲数
#define DRV_PULSES_PER_REV      4096
// 计算每脉冲对应的步数（用于位置换算）
#define DRV_STEPS_PER_PULSE     (DRV_STEPS_PER_REV / (float)DRV_PULSES_PER_REV)
// 编码器数据基准值（10万）
#define DRV_ENCODER_BASE_VALUE  100000



// GPIO实例结构体变量 (用于初始化GPIO)
XGpioPs GpioInst;

motor_info_t m_motor[MOTOR_COUNT] = {
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}
};


uint8_t last_dir[MOTOR_COUNT] = {0};



/**
 * @brief 系统硬件初始化函数
 * 
 * 该函数完成以下初始化操作：
 * 1. 初始化GPIO控制器
 * 2. 初始化TMC2660电机驱动芯片GPIO接口
 * 3. 初始化SPI通信接口
 * 4. 配置电机参数（细分数和电流）
 * 
 * @return XST_SUCCESS 初始化成功
 *         XST_FAILURE 初始化失败
 */
int motor_device_init( MOTOR_ADDR addr, uint16_t sub_div, uint8_t current )
{
    int Status;
    XGpioPs_Config *GpioConfig;

    // GPIO初始化
    GpioConfig = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_DEVICE_ID);
    Status = XGpioPs_CfgInitialize(&GpioInst, GpioConfig, GpioConfig->BaseAddr);

    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    
    // TMC2660电机驱动初始化
    if (motor_gpio_init(&GpioInst) != XST_SUCCESS) {
        return XST_FAILURE;
    }
    
    // SPI接口初始化
    if (motor_spi_init() != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // 电机初始化：细分数32，电流20（最大值：32）
    motor_init( addr, sub_div, current );

    // 禁用数据缓存
    Xil_DCacheDisable();
    usleep(100000);

    return XST_SUCCESS;
}

/**
 * @brief 电机控制寄存器初始化
 * 
 * 初始化电机控制内存区域，清零控制寄存器
 */
void motor_ctrlreg_init(void)
{
#define MOTOR_CTRLREG_LEN 44
    // 初始化指向电机控制内存区域的指针
    uint32_t *mem_ptr = (uint32_t *)WRITE_ADDR_START1;

    // 清零控制寄存器内存区域
    memset(mem_ptr, 0, MOTOR_CTRLREG_LEN);
    
    // 内存屏障，确保写操作完成
    __sync_synchronize();

    // 刷新数据缓存，确保数据写入物理内存
    Xil_DCacheFlushRange((UINTPTR)&mem_ptr[0], sizeof(uint32_t));
}

/**
 * @brief 获取指定电机的限位开关状态
 * 
 * 通过GPIO读取限位开关状态，并进行消抖处理
 * 
 * @param addr 电机地址，有效值为1-5
 * @return int8_t 返回限位开关状态值
 *         -1: 地址无效
 *          0: 未触发限位
 *          1: 触发限位
 */
#if 1
int8_t motor_get_limit_center(MOTOR_ADDR addr)
{
#define GPIO_DEBOUNCE_CNT 10   // 消抖检测次数
#define GPIO_CONFIRM_CNT  6    // 确认定次次数

    uint8_t stat = 0;
    uint8_t gpio_zero_cnt = 0;
    MOTOR_ADDR tmp_addr = addr;

    // 检查电机地址有效性
    if (!motor_addr_check(tmp_addr))
        return -1;

#ifdef MOTOR_COMPATIBLE
    if ( tmp_addr == TMC2660_M1 ){
    	tmp_addr = TMC2660_M2;
    }else if ( tmp_addr == TMC2660_M2 ){
    	tmp_addr = TMC2660_M1;
    }else{
    }
#else
#endif

    // 通过多次采样进行消抖处理
    for (uint8_t i = 0; i < GPIO_DEBOUNCE_CNT; i++) {
        // 读取对应电机的限位开关GPIO状态
        if (XGpioPs_ReadPin(&GpioInst, M1_LIMIT + tmp_addr - 1) == 0) {
            gpio_zero_cnt++;
        }
        usleep(100);
    }

    // 根据检测结果判断限位状态
    if (gpio_zero_cnt > GPIO_CONFIRM_CNT)
        stat = 0;  // 未触发限位
    else
        stat = 1;  // 触发限位

    return stat;
}
#endif

// 破坏结构，临时测试
extern motor_ctrl_t m_motorctrl[MOTOR_COUNT];
extern void SetMotorOutput(uint8_t motor_id, float output);
extern volatile uint8_t motor_move_flag[];
extern volatile uint8_t is_pid_flag[];
volatile uint64_t g_motor_expected_end_us[MOTOR_COUNT] = {0};
/**
 * @brief 控制电机运动
 * 
 * 向PL端写入电机控制寄存器，设置速度、方向和步数
 * 
 * @param addr 电机地址
 * @param dir 运动方向(1:正向, 2:反向)
 * @param speed 运动速度
 * @param step 运动步数
 */
void motor_control(MOTOR_ADDR addr, uint8_t dir, uint32_t speed, uint32_t step)
{
    uint8_t tmp_dir = dir;
    MOTOR_ADDR tmp_addr = addr;
    uint32_t tmp_speed = speed;
    uint32_t tmp_step = step;

    if (!motor_addr_check(tmp_addr))
        return;

#ifdef MOTOR_COMPATIBLE
    if ( tmp_addr == TMC2660_M1 ){
        tmp_addr = TMC2660_M2;
    }else if ( tmp_addr == TMC2660_M2 ){
        tmp_addr = TMC2660_M1;
    }
#endif

    // 额托电机特殊处理
    if ( tmp_addr == TMC2660_M4 )
    {
        if (dir == MOTOR_DIR_FORWARD)
            tmp_dir = 1;
        else
            tmp_dir = 0;
        tmp_speed = speed * 3;
        if ( tmp_speed > 65000 ){
            tmp_speed = 65000;
        }
        tmp_step = (uint32_t)(step * 4 / 10);
    }
    else {
        if (dir == MOTOR_DIR_FORWARD)
            tmp_dir = 0;
        else
            tmp_dir = 1;
    }

    // 写入 PL 电机控制寄存器
    pl_write_u32(WRITE_ADDR_MOTOR1 + (tmp_addr - 1) * 4, (tmp_speed << 16) | (tmp_dir << 15) | (tmp_step & 0x7FFF));

    // 记录该批脉冲的理论执行截止时间戳（增加 60ms 缓冲裕量，覆盖高频短步数指令）
	if (dir != MOTOR_DIR_NONE && tmp_speed > 0 && (tmp_step & 0x7FFF) > 0) {
		uint64_t run_us = ((uint64_t)(tmp_step & 0x7FFF) * 1000000ULL) / tmp_speed;
		g_motor_expected_end_us[tmp_addr - 1] = timer_get_1us() + run_us + 60000ULL;
	} else {
		g_motor_expected_end_us[tmp_addr - 1] = 0;
	}
}

// 3. 完整替换 motor_stop 函数：
void motor_stop(MOTOR_ADDR addr)
{
    if (!motor_addr_check(addr))
        return;

    // 彻底切断后台 1ms PID 闭环输出，防止停机后下一毫秒被重新拉起
    motor_move_flag[addr - 1] = 0;
    is_pid_flag[addr - 1] = 0;
    g_motor_expected_end_us[addr - 1] = 0;

    xil_printf("MOTOR: %d =============电机停止=============\r\n", addr);
    motor_control(addr, !last_dir[addr-1], 0, 0);

    SetMotorOutput( addr - 1, 0 );
}
/**
 * @brief 获取磁编码器的编码值并转换为步进值
 *
 * @param encoder_id 编码器ID
 * @return int32_t 转换后的步进值
 */
int32_t motor_get_real_encode(MOTOR_ADDR addr)
{
    int32_t cali_value = 0;  // 校准后的步进值
    MOTOR_ADDR tmp_addr = addr;

    // 检查编码器ID是否有效
    if (!motor_addr_check(tmp_addr))
        return -1;

#ifdef MOTOR_COMPATIBLE
    if ( tmp_addr == TMC2660_M1 ){
    	tmp_addr = TMC2660_M2;
    }else if ( tmp_addr == TMC2660_M2 ){
    	tmp_addr = TMC2660_M1;
    }else{
    }
#else
#endif

    // 从PL端读取原始编码值
    uint32_t raw_value = pl_read_u32(READ_ADDR_ENCODE_MOTOR1 + (tmp_addr-1) * 4);

    // 将编码值转换为步进值
    // 转换公式：(原始值 - 基准值) * 每脉冲对应的步进数
    cali_value = ((int32_t)raw_value - DRV_ENCODER_BASE_VALUE) * DRV_STEPS_PER_PULSE;

    return cali_value;
}

/**
 * @brief 更新归中后的磁编码数值(设置中心点)
 *
 * @param addr 电机地址
 * @param offset 偏移量
 */
void motor_center_step_init(MOTOR_ADDR addr, int32_t offset)
{
    // 检查编码器ID是否有效
    if (!motor_addr_check(addr))
        return;

    // 设置当前电机的中心步进值
    // 中心步进值 = 当前编码值 + 偏移量
    m_motor[addr-1].center_step = motor_get_real_encode(addr) + offset;
}

/**
 * @brief 获取当前相对于中心点的步进值
 *
 * @param addr 电机地址
 * @return int32_t 相对于中心点的步进值
 */
int32_t motor_get_cur_step(MOTOR_ADDR addr)
{
    int32_t cur_real_encode = 0;  // 当前实际编码值

    // 检查编码器ID是否有效
    if (!motor_addr_check(addr))
        return -1;

    // 获取当前编码值
    cur_real_encode = motor_get_real_encode(addr);

    // 返回相对于中心点的步进值
    return (cur_real_encode - m_motor[addr-1].center_step);
}



uint8_t set_bit(uint8_t byte, int n, int bit_value)
{
    if (n < 0 || n > 7) {
        // 可选：非法输入处理（这里简单返回原值）
        return byte;
    }

    if (bit_value) {
        // 置 1：使用按位或 | 和左移
        return byte | (1U << n);
    } else {
        // 置 0：使用按位与 & 和取反 ~
        return byte & ~(1U << n);
    }
}



void motor_limit_update_to_pl( MOTOR_ADDR addr, uint8_t forward, uint8_t backward )
{
	// 升降上下左右前后

	// 保存当前限位状态
	static uint8_t limit_state = 0;

	uint8_t tmp_limit = limit_state;

	limit_state = set_bit( tmp_limit, (addr-1)*2, forward );
	limit_state = set_bit( tmp_limit, (addr-1)*2+1, backward );

	pl_write_u8( WRITE_ADDR_MOTOR_LIMIT, limit_state );
}



