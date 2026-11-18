#ifndef BGLAMP_DRV_H_
#define BGLAMP_DRV_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================================
// 1. 基地址与寄存器定义 (严格对齐 Document Version: 1.2.0.0)
// =========================================================================
#define BGLAMP_BASEADDR             0x40000000U

// 版本寄存器: 3.1.0.0 -> 0x03010000
#define REG_FPGA_VERSION            (BGLAMP_BASEADDR + 0x0000U)
extern uint32_t g_pl_fpga_version;

// 固视灯 MOSI 寄存器基地址 (0x0200 ~ 0x0238, 共 15 路)
#define REG_MOSI_FIXATION_BASE      (BGLAMP_BASEADDR + 0x0200U)
#define FIXATION_LAMP_NUM           15U

// 红外补光灯 MOSI 寄存器基地址 (0x023C ~ 0x0250, 共 6 路)
#define REG_MOSI_FILL_BASE          (BGLAMP_BASEADDR + 0x023CU)
#define FILL_LAMP_NUM               6U

// 左右对位灯 MOSI 寄存器
#define REG_MOSI_LEFT_ALIGN         (BGLAMP_BASEADDR + 0x0254U)
#define REG_MOSI_RIGHT_ALIGN        (BGLAMP_BASEADDR + 0x0258U)

// 执行触发寄存器 (写 1 触发整帧 384 位移出并锁存，硬件自动清零)
#define REG_TLC5940_COMMIT          (BGLAMP_BASEADDR + 0x025CU)

// MISO 移位状态字读回基地址 (对应 MOSI 偏移 + 0x100)
#define REG_MISO_FIXATION_BASE      (BGLAMP_BASEADDR + 0x0300U)
#define REG_MISO_FILL_BASE          (BGLAMP_BASEADDR + 0x033CU)
#define REG_MISO_LEFT_ALIGN         (BGLAMP_BASEADDR + 0x0354U)
#define REG_MISO_RIGHT_ALIGN        (BGLAMP_BASEADDR + 0x0358U)

// 全局硬件总线门限标志 (pl_on / pl_off 控制)
extern uint8_t g_pl_lamp_ready;

// =========================================================================
// 2. 业务驱动接口
// =========================================================================
int  bglamp_drv_init(void);
void bglamp_set_gate(uint8_t enable);
void bglamp_set_mode(uint8_t mode); // 保留接口兼容

// 控制接口 (入参 level 范围: 0 ~ 63)
void bglamp_turn_off_all(void);
void bglamp_set_fixation_single(uint8_t lamp_id, uint8_t level);
void bglamp_set_fixation_all(uint8_t level);
void bglamp_set_infrared_single(uint8_t lamp_id, uint8_t level);
void bglamp_set_infrared_all(uint8_t level);
void bglamp_set_align_left(uint8_t level);
void bglamp_set_align_right(uint8_t level);
void bglamp_set_align_both(uint8_t level);

// 状态回读与打印
void bglamp_dump_status(void);
void bglamp_print_infrared_status(void);

// Shell 槽位函数
int  bglamp_shell_cmd(int argc, char *argv[]);
void bglamp_shell_help(void);

#ifdef __cplusplus
}
#endif

#endif /* BGLAMP_DRV_H_ */
