#ifndef SRC_JOYSTICK_H_
#define SRC_JOYSTICK_H_

#include "stdint.h"

// 摇杆状态数据结构
typedef struct {
    int16_t   x;             // X轴物理位置
    int16_t   y;             // Y轴物理位置
    int16_t   z;             // Z轴物理位置
    uint16_t  buttons;       // 按键掩码状态
    uint8_t   x_dir;         // X轴当前动作方向
    uint8_t   y_dir;         // Y轴当前动作方向
    uint64_t  timestamp_us;  // 微秒级时间戳
} joystick_state_t;

// 外部全局导出变量
extern uint8_t g_joystick_active_flag;
extern joystick_state_t g_current_joystick_data;
extern uint8_t g_joystick_data_updated;
extern uint8_t m_joystick_mode;
extern uint8_t m_joystick_ctrl;
extern volatile uint8_t g_motor_owner;
// 原声明：extern uint8_t m_joystick_mode;
// 改为三轴独立：
extern uint8_t m_joystick_mode_x;
extern uint8_t m_joystick_mode_y;
extern uint8_t m_joystick_mode_z;

// 函数声明增加 axis 参数 (0:全部, 1:X轴, 2:Y轴, 3:Z轴)
void joystick_mode_config(uint8_t mode, uint8_t val_x, uint8_t val_y, uint8_t val_z);
// 基础接口
void joystick_init(void);
void joystick_process(void);
//void joystick_mode_config(uint8_t mode, uint8_t value);
void joystick_control_config(uint8_t mode);
void check_and_report_motor_status(void);
void check_joystick_disconnect_watchdog(void);
void Joystick_Filter_Flush(void);

// 令牌与主动打断接口
void Host_Claim_Motor_Control(void);
void Host_Release_Motor_Control(void);

// ===================================================================
// 【新增】FPGA 接管保护核心接口
// ===================================================================
/**
 * @brief FPGA 开始接管电机控制（禁能摇杆，安全清除当前驱动输出）
 */
void Joystick_FPGA_Takeover_Enter(void);

/**
 * @brief FPGA 退出接管电机（硬同步当前实际编码器，重置采样差分，从安全区起跳）
 */
void Joystick_FPGA_Takeover_Exit(void);
void Joy_Stall_Blackbox_Poll(void);
void Joy_Stress_Test_Poll(void);
#endif /* SRC_JOYSTICK_H_ */
