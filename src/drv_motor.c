/*
 * drv_motor.c
 *
 *  Created on: 2025年9月10日
 *      Author: dulc
 */
#include "drv_motor.h"
#include "drv_pl.h"
#include "version.h"

// 寄存器初始化值定义
#define INIT_DRVCTRL            0x00000000  // DRVCTRL寄存器初始值
#define INIT_CHOPCONF           0x000801B1//0x000901B2//0x000801B1  // CHOPCONF寄存器初始值
#define INIT_SMARTEN            0X000A0202//0x000A0000//0X000A0202  // SMARTEN寄存器初始值
#define INIT_SGCSCONF           0X000C0000  // SGCSCONF寄存器初始值
#define INIT_DRVCONF            0x000EA140//0x000EA150//0x000EA140  // DRVCONF寄存器初始值
#define INIT_DRVCONF_L          0x000EA110
// 寄存器地址定义
#define REG_DRVCTRL             0X00000000  // 控制步进/方向接口模式及微步细分设置
#define REG_CHOPCONF            0X00080000  // 配置斩波器参数（blank time、chopper模式、Hysteresis等），影响电机噪声和平滑性
#define REG_SMARTEN             0X000A0000  // 电流控制智能调节，支持CoolStep技术以优化能耗和扭矩
#define REG_SGCSCONF            0X000C0000  // 设置失速检测（StallGuard）阈值和电流调节精度
#define REG_DRVCONF             0X000E0000  // 驱动核心配置（斜率控制、短路检测、步进/方向接口使能）



/*
1初始化配置：配置TMC2660驱动器的寄存器（如斩波配置、电流控制、驱动配置等）。
2电机控制：支持电机的正转、反转、停止操作，并实现S型加减速曲线控制。
3位置管理：记录和更新电机的当前位置（步数），并实现限位保护。
4通信接口：通过SPI与TMC2660通信，设置驱动器参数。
5中断处理：利用定时器中断实现PWM控制，以驱动步进电机。
*/

// 全局变量
static XSpiPs SpiInst;
XGpioPs *GpioInstance = NULL;


// 仅限临时存储当前电流，细分，用于上位机查询
motor_param_t m_motor_param[MOTOR_COUNT] = { {32,20}, {32,20}, {32,20}, {32,20}, {32,20} };



void Motor_ConfigRegister(MOTOR_ADDR motor_id, uint32_t reg_addr, uint32_t value);




/**
 * @brief 初始化SPI接口
 * 
 * 该函数完成以下初始化操作：
 * 1. 查找并配置SPI设备
 * 2. 执行硬件自检
 * 3. 设置SPI工作模式和时钟参数
 * 4. 启用SPI接口
 * 
 * @return int 初始化状态（XST_SUCCESS表示成功）
 */
int motor_spi_init(void)
{
    XSpiPs_Config *SpiConfig;
    int status;

    // 查找设备配置（设备ID需匹配硬件）
    SpiConfig = XSpiPs_LookupConfig(XPAR_XSPIPS_0_DEVICE_ID);
    if (SpiConfig == NULL) {
        return XST_FAILURE;  // 配置无效
    }

    // 初始化SPI控制器
    status = XSpiPs_CfgInitialize(&SpiInst, SpiConfig, SpiConfig->BaseAddress);
    if (status != XST_SUCCESS) {
        return status;       // 初始化失败
    }

    // 硬件自检
    status = XSpiPs_SelfTest(&SpiInst);
    if (status != XST_SUCCESS) {
        return status;       // 自检失败
    }

    // 设置SPI选项
    // 主模式 + CPOL=1（高电平空闲）+ CPHA=1（第二边沿采样）
    uint32_t options = XSPIPS_MASTER_OPTION |
                       XSPIPS_CLK_ACTIVE_LOW_OPTION |  // CPOL=1（高电平空闲）
                       XSPIPS_CLK_PHASE_1_OPTION;      // CPHA=1（第二边沿采样）

    if (XSpiPs_SetOptions(&SpiInst, options) != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // 配置时钟分频（PS端SPI需显式设置）
    // 使用256分频降低SPI时钟频率
    XSpiPs_SetClkPrescaler(&SpiInst, XSPIPS_CLK_PRESCALE_256);

    // 启用SPI
    XSpiPs_Enable(&SpiInst);

    return XST_SUCCESS;
}



/**
 * @brief 初始化GPIO引脚（TMC2660片选引脚）
 * 
 * @param GpioInst GPIO实例指针
 * @return int 初始化状态（XST_SUCCESS表示成功）
 */
int motor_gpio_init(XGpioPs *GpioInst)
{
    GpioInstance = GpioInst;

    // 配置所有片选引脚为输出模式
    for (int i = TMC2660_M1; i <= MOTOR_COUNT; i++) {
        uint32_t cs_pin;

        // 根据电机ID选择对应的片选引脚
        switch (i) {
#ifdef MOTOR_COMPATIBLE
			case TMC2660_M1: cs_pin = M2_CS; break;
			case TMC2660_M2: cs_pin = M1_CS; break;
#else
            case TMC2660_M1: cs_pin = M1_CS; break;
            case TMC2660_M2: cs_pin = M2_CS; break;
#endif
            case TMC2660_M3: cs_pin = M3_CS; break;
            case TMC2660_M4: cs_pin = M4_CS; break;
            case TMC2660_M5: cs_pin = M5_CS; break;
            default: continue;
        }

        // 设置片选引脚为输出模式并初始化为高电平（禁用状态）
        XGpioPs_SetDirectionPin(GpioInstance, cs_pin, 1);
        XGpioPs_SetOutputEnablePin(GpioInstance, cs_pin, 1);
        XGpioPs_WritePin(GpioInstance, cs_pin, 1);
    }

    // 初始化限位状态
    XGpioPs_SetDirectionPin(GpioInstance, M1_LIMIT, 0);
    XGpioPs_SetDirectionPin(GpioInstance, M2_LIMIT, 0);
    XGpioPs_SetDirectionPin(GpioInstance, M3_LIMIT, 0);
    XGpioPs_SetDirectionPin(GpioInstance, M4_LIMIT, 0);
    XGpioPs_SetDirectionPin(GpioInstance, M5_LIMIT, 0);

    return XST_SUCCESS;
}



/**
 * @brief 向指定TMC2660电机驱动芯片写入数据
 * 
 * 该函数通过SPI接口向指定ID的TMC2660电机驱动芯片发送20位命令数据。
 * TMC2660使用高位在前的传输方式，需要将32位数据转换为3个8位字节进行传输。
 * 
 * @param addr 电机地址（TMC2660_M1到TMC2660_M5）
 * @param data 要写入的20位数据
 */
void motor_tmc2660_write(MOTOR_ADDR motor_id, uint32_t data)
{
    uint32_t cs_pin;        // 当前电机的片选引脚
    uint8_t wbuf[3] = {0};  // 写入数据缓冲区（三字节）
    uint8_t rbuf[3] = {0};  // 读取数据缓冲区（三字节，虽然不使用但API要求提供）

    // 根据电机ID选择对应的片选引脚
    switch (motor_id) {
#ifdef MOTOR_COMPATIBLE
		case TMC2660_M1: cs_pin = M2_CS; break;
		case TMC2660_M2: cs_pin = M1_CS; break;
#else
        case TMC2660_M1: cs_pin = M1_CS; break;
        case TMC2660_M2: cs_pin = M2_CS; break;
#endif
        case TMC2660_M3: cs_pin = M3_CS; break;
        case TMC2660_M4: cs_pin = M4_CS; break;
        case TMC2660_M5: cs_pin = M5_CS; break;
        default:
            // 无效的电机ID，直接返回
            return;
    }

    // 将32位数据转换为3个8位字节（TMC2660使用20位数据，高位在前）
    wbuf[0] = (data >> 16) & 0xFF;  // 提取高8位
    wbuf[1] = (data >> 8)  & 0xFF;  // 提取中8位
    wbuf[2] = (data & 0xFF);        // 提取低8位

    // 激活片选信号（低电平有效）
    XGpioPs_WritePin(GpioInstance, cs_pin, 0);

    // 通过SPI总线传输数据
    // 注意：虽然我们只关心写入数据，但XSpiPs_PolledTransfer要求提供接收缓冲区
    XSpiPs_PolledTransfer(&SpiInst, wbuf, rbuf, 3);

    // 释放片选信号（恢复高电平）
    XGpioPs_WritePin(GpioInstance, cs_pin, 1);

    // 添加短延迟确保片选切换稳定
    usleep(1);
}



/**
 * @brief 设置电机电流
 * 
 * @param motor_id 电机ID
 * @param current 电流值(0-31)
 */
void motor_set_current(MOTOR_ADDR motor_id, uint8_t current)
{
#define MOTOR_MAX_CURRENT 0x1F  // 最大电流值(31)
    uint8_t tmp_cur = current;
    
    // 检查电机ID有效性
    if (!motor_addr_check(motor_id)) {
        return;
    }

    // 限制电流范围
    if (tmp_cur > MOTOR_MAX_CURRENT) tmp_cur = MOTOR_MAX_CURRENT;

    // 配置电流寄存器
    motor_tmc2660_write(motor_id, REG_SGCSCONF | INIT_SGCSCONF | tmp_cur);
    m_motor_param[motor_id-1].current = tmp_cur;
}

/**
 * @brief 获取电机电流设置值
 * 
 * @param motor_id 电机ID
 * @return uint8_t 电流值
 */
uint8_t motor_get_current(MOTOR_ADDR motor_id)
{
    return m_motor_param[motor_id-1].current;
}

/**
 * @brief 设置电机细分数
 * 
 * @param motor_id 电机ID
 * @param subdiv 细分数(1,2,4,8,16,32,64,128,256)
 */
void motor_set_subdiv(MOTOR_ADDR motor_id, uint16_t subdiv)
{
    // 根据细分数配置DRVCTRL寄存器
    switch(subdiv) {
        case 1:   Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x08); break;
        case 2:   Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x07); break;
        case 4:   Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x06); break;
        case 8:   Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x05); break;
        case 16:  Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x04); break;
        case 32:  Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x03); break;
        case 64:  Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x02); break;
        case 128: Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x01); break;
        case 256: Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x00); break;
        default:  Motor_ConfigRegister(motor_id, REG_DRVCTRL, INIT_DRVCTRL | 0x03); break; // 默认32细分
    }
    m_motor_param[motor_id-1].div = subdiv;
}

/**
 * @brief 获取电机细分数
 * 
 * @param motor_id 电机ID
 * @return uint16_t 细分数
 */
uint16_t motor_get_subdiv(MOTOR_ADDR motor_id)
{
    return m_motor_param[motor_id-1].div;
}

/**
 * @brief 初始化所有电机
 * 
 * @param subdiv 细分数
 * @param current 电流值
 */
void motor_init( MOTOR_ADDR addr, uint16_t subdiv, uint8_t current )
{
	// 设置细分数
	motor_set_subdiv( addr, subdiv);

	// 配置其他必要寄存器
	Motor_ConfigRegister( addr, REG_CHOPCONF, INIT_CHOPCONF);
	Motor_ConfigRegister( addr, REG_SMARTEN, INIT_SMARTEN);
	Motor_ConfigRegister( addr, REG_DRVCONF, INIT_DRVCONF);
#if 0
	if(addr == TMC2660_M2)
	{
		Motor_ConfigRegister( addr, REG_DRVCONF, INIT_DRVCONF_L);
		xil_printf("M2: 设置为低精度模式\r\n");
	}
#endif
	// 设置电流
	motor_set_current( addr, current);
}

/**
 * @brief 配置指定电机的寄存器
 * 
 * @param motor_id 电机ID
 * @param reg_addr 寄存器地址
 * @param value 要写入的值(20位)
 */
void Motor_ConfigRegister(MOTOR_ADDR motor_id, uint32_t reg_addr, uint32_t value)
{
    motor_tmc2660_write(motor_id, reg_addr | (value & 0xFFFFF));
}

/**
 * @brief 检查电机地址有效性
 * 
 * @param motor_id 电机ID
 * @return int 1-有效 0-无效
 */
int motor_addr_check(uint8_t motor_id)
{
    if ((motor_id != TMC2660_M1) &&
        (motor_id != TMC2660_M2) &&
        (motor_id != TMC2660_M3) &&
        (motor_id != TMC2660_M4) &&
        (motor_id != TMC2660_M5)) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * @brief 获取指定电机的故障状态(失步检测)
 * 
 * @param motor_id 电机ID
 * @return uint8_t 故障状态 0-正常 1-故障
 */
uint8_t motor_get_motor_fault(uint8_t motor_id)
{
    uint8_t motor_fault = 0;

    if (!motor_addr_check(motor_id)) {
        return 0;
    }

    // 读取对应电机的故障引脚状态
    switch(motor_id) {
#ifdef MOTOR_COMPATIBLE
		case TMC2660_M1: motor_fault = XGpioPs_ReadPin(GpioInstance, M2_SG); break;
		case TMC2660_M2: motor_fault = XGpioPs_ReadPin(GpioInstance, M1_SG); break;
#else
        case TMC2660_M1: motor_fault = XGpioPs_ReadPin(GpioInstance, M1_SG); break;
        case TMC2660_M2: motor_fault = XGpioPs_ReadPin(GpioInstance, M2_SG); break;
#endif
        case TMC2660_M3: motor_fault = XGpioPs_ReadPin(GpioInstance, M3_SG); break;
        case TMC2660_M4: motor_fault = XGpioPs_ReadPin(GpioInstance, M4_SG); break;
        case TMC2660_M5: motor_fault = XGpioPs_ReadPin(GpioInstance, M5_SG); break;
        default:
            // 无效的电机ID，直接返回
            break;
    }

    return motor_fault;
}
// 记录各电机当前的 DRVCONF 模式 (0: 默认/高精度, 1: 低精度)
static uint8_t s_current_drv_mode[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
void motor_set_precision_mode(MOTOR_ADDR addr, uint8_t is_low_precision)
{
    if (!motor_addr_check(addr)) return;
    uint8_t idx = addr - 1;

    // 状态缓存：如果当前模式已经符合预期，直接返回，绝对不发无意义的 SPI 阻塞 CPU
    if (s_current_drv_mode[idx] == is_low_precision) {
        return;
    }

    if (is_low_precision) {
        // 写入低精度/低灵敏度配置
        Motor_ConfigRegister(addr, REG_DRVCONF, INIT_DRVCONF_L);
        xil_printf("M%d: 切换至低精度模式 (DRVCONF_L)\r\n", addr);
    } else {
        // 恢复默认高精度配置
        Motor_ConfigRegister(addr, REG_DRVCONF, INIT_DRVCONF);
        xil_printf("M%d: 恢复高精度模式 (DRVCONF)\r\n", addr);
    }

    s_current_drv_mode[idx] = is_low_precision;
}
