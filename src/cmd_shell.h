#ifndef SRC_CMD_SHELL_H_
#define SRC_CMD_SHELL_H_

#include <stdint.h>
#include "xil_printf.h"
typedef int  (*shell_cmd_func_t)(int argc, char *argv[]);
typedef void (*shell_help_func_t)(void);
// ===================================================================
// 日志模块掩码定义 (支持位与运算)
// ===================================================================
#define LOG_MOD_NONE      0x00000000
#define LOG_MOD_JOY       (1 << 0)  // 摇杆高频心跳/数据
#define LOG_MOD_MOTOR     (1 << 1)  // 电机加减速/停止/曲线
#define LOG_MOD_LIMIT     (1 << 2)  // 限位/纠偏
#define LOG_MOD_PID       (1 << 3)  // PID 调优日志
#define LOG_MOD_TCP       (1 << 4)  // TCP 协议与数据包
#define LOG_MOD_SYS       (1 << 5)  // 系统关键警告/故障
#define LOG_MOD_LED       (1 << 6)  // 红外补光灯交互日志
#define LOG_MOD_ALL       0xFFFFFFFF

extern uint32_t g_log_mask;

// 条件打印宏
#define LOG_PRINT(mod, ...) do { \
    if (g_log_mask & (mod)) { \
        xil_printf(__VA_ARGS__); \
    } \
} while(0)

// 对外接口
void cmd_shell_init(void);
void cmd_shell_process(void);
/**
 * @brief 注册外部业务功能槽位
 * @param mod_name 模块名（如 "BGLAMP"、"CAM"）
 * @param cmd_cb   业务命令执行槽函数
 * @param help_cb  业务命令帮助打印槽函数
 * @return 0: 成功; -1: 槽位已满
 */
int cmd_shell_register_ext(const char *mod_name, shell_cmd_func_t cmd_cb, shell_help_func_t help_cb);
#endif /* SRC_CMD_SHELL_H_ */
