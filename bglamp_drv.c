#if 0
#include "bglamp_drv.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"

// 默认开启保护门限：若开机未检测到 3.0.0.0 则禁止直接访问 AXI 总线
uint8_t g_pl_lamp_ready = 0;
static uint8_t s_last_fixation_id = 0xFF;
int bglamp_drv_init(void)
{
    // 如果硬件就绪标志已开启，检查 FPGA 版本
    if (g_pl_lamp_ready) {
        uint32_t ver = Xil_In32(REG_FPGA_VERSION);
        if (ver == EXPECTED_FPGA_VERSION) {
            xil_printf("[BGLAMP] FPGA Version Matched: 0x%08X (3.0.0.0)\r\n", ver);
            return 0;
        } else {
            xil_printf("[BGLAMP_WARN] FPGA Version Mismatch: 0x%08X (Expected 0x03000000)\r\n", ver);
            return -1;
        }
    }
    return 0;
}
uint16_t map_level_to_gs(uint8_t level)
{
	if(level == 0)
		return 0;
	else if(level >= 100)
		return 4095;
	else
		return (uint16_t)(((uint32_t)level * 4095 + 50) / 100);



}
void bglamp_set_gate(uint8_t enable)
{
    g_pl_lamp_ready = enable;
    xil_printf("[BGLAMP] Hardware gate is %s\r\n", enable ? "ENABLED" : "DISABLED");
}

int bglamp_commit(void)
{
    if (!g_pl_lamp_ready) {
        return -1;
    }

    // 检查提交寄存器 [0] 是否已是 0
    if (Xil_In32(REG_TLC5940_COMMIT) & 0x01) {
        return -2; // 硬件正忙
    }

    // 写 1 请求更新
    Xil_Out32(REG_TLC5940_COMMIT, 0x00000001U);

    // 等待 PL 移出 192 位并拉高 xlat 后硬件清 0
    uint32_t timeout = 50000;
    while ((Xil_In32(REG_TLC5940_COMMIT) & 0x01) && --timeout) {
        usleep(10);
    }

    if (timeout == 0) {
        xil_printf("[BGLAMP_ERR] TLC5940 commit timeout!\r\n");
        return -3;
    }

    return 0;
}

/**
 * @brief 单固视灯互斥控制 (同一时刻仅允许点亮一个固视灯)
 * @param id    固视灯号 (0 ~ 14)
 * @param level 亮度等级 (0 ~ 63，0 为熄灭)
 */
void bglamp_set_fixation_single(uint8_t id, uint8_t level)
{
    if (!g_pl_lamp_ready || id > 14) return;
    if (level > 63) level = 63;

    // 1. 如果之前有点亮的固视灯，且不是当前这个灯，先将旧灯寄存器写 0 清除
    if (s_last_fixation_id <= 14 && s_last_fixation_id != id) {
        Xil_Out32(REG_MOSI_FIXATION_BASE + (s_last_fixation_id * 4), 0);
    }

    // 2. 写入新灯的亮度
    Xil_Out32(REG_MOSI_FIXATION_BASE + (id * 4), (uint32_t)(level & 0x3F));

    // 3. 更新当前点亮灯号记录
    if (level > 0) {
        s_last_fixation_id = id;
    } else {
        // 如果把当前灯也关掉了，清除记录
        if (s_last_fixation_id == id) {
            s_last_fixation_id = 0xFF;
        }
    }

    // 4. 一次性移位提交，硬件同时完成旧灯熄灭与新灯点亮
    bglamp_commit();
}
void bglamp_set_fixation_all(uint8_t level)
{
    if (!g_pl_lamp_ready) return;
    if (level > 63) level = 63;

    for (int i = 0; i < 15; i++) {
        Xil_Out32(REG_MOSI_FIXATION_BASE + (i * 4), (uint32_t)(level & 0x3F));
    }
    bglamp_commit();
}

void bglamp_set_infrared_single(uint8_t id, uint8_t level)
{
    if (!g_pl_lamp_ready || id > 5) return;
    if (level > 63) level = 63;

    Xil_Out32(REG_MOSI_INFRARED_BASE + (id * 4), (uint32_t)(level & 0x3F));
    bglamp_commit();
}

void bglamp_set_infrared_all(uint8_t level)
{
    if (!g_pl_lamp_ready) return;
    if (level > 63) level = 63;

    for (int i = 0; i < 6; i++) {
        Xil_Out32(REG_MOSI_INFRARED_BASE + (i * 4), (uint32_t)(level & 0x3F));
    }
    bglamp_commit();
}

void bglamp_set_align_left(uint8_t level)
{
    if (!g_pl_lamp_ready) return;
    if (level > 63) level = 63;

    Xil_Out32(REG_MOSI_ALIGN_LEFT, (uint32_t)(level & 0x3F));
    bglamp_commit();
}

void bglamp_set_align_right(uint8_t level)
{
    if (!g_pl_lamp_ready) return;
    if (level > 63) level = 63;

    Xil_Out32(REG_MOSI_ALIGN_RIGHT, (uint32_t)(level & 0x3F));
    bglamp_commit();
}

void bglamp_set_align_both(uint8_t level)
{
    if (!g_pl_lamp_ready) return;
    if (level > 63) level = 63;

    Xil_Out32(REG_MOSI_ALIGN_LEFT, (uint32_t)(level & 0x3F));
    Xil_Out32(REG_MOSI_ALIGN_RIGHT, (uint32_t)(level & 0x3F));
    bglamp_commit();
}

/**
 * @brief 关闭全部灯时，同步复位记录
 */
void bglamp_turn_off_all(void)
{
    if (!g_pl_lamp_ready) return;

    for (int i = 0; i < 15; i++) Xil_Out32(REG_MOSI_FIXATION_BASE + (i * 4), 0);
    for (int i = 0; i < 6; i++)  Xil_Out32(REG_MOSI_INFRARED_BASE + (i * 4), 0);
    Xil_Out32(REG_MOSI_ALIGN_LEFT, 0);
    Xil_Out32(REG_MOSI_ALIGN_RIGHT, 0);

    // 复位固视灯记录
    s_last_fixation_id = 0xFF;

    bglamp_commit();
}
/**
 * @brief 格式化打印当前 6 路红外补光灯状态
 */
void bglamp_print_infrared_status(void)
{
    if (!g_pl_lamp_ready) {
        xil_printf("[BGLAMP] PL access is currently blocked (Safe Mode).\r\n");
        return;
    }

    xil_printf("\r\n--- [Infrared Fill Light Status] ---\r\n");
    for (int i = 0; i < 6; i++) {
        // 读取 MOSI 亮度寄存器 (0x4000_023C ~ 0x4000_0250)[cite: 4, 5]
        uint32_t mosi_val = Xil_In32(REG_MOSI_INFRARED_BASE + (i * 4)) & 0x3F; //[cite: 4, 5]
        // 读取 MISO 移位回读寄存器 (0x4000_033C ~ 0x4000_0350)[cite: 4, 5]
        uint32_t miso_val = Xil_In32(REG_MISO_INFRARED_BASE + (i * 4)) & 0x3F; //[cite: 4, 5]

        xil_printf("  Lamp_%d: Level %2d [%s] | MISO(prev): %2d\r\n",
                   i, mosi_val, (mosi_val > 0) ? "ON" : "OFF", miso_val);
    }
    xil_printf("------------------------------------\r\n");
}
void bglamp_dump_status(void)
{
    if (!g_pl_lamp_ready) {
        xil_printf("[BGLAMP] PL access is currently blocked (Safe Mode).\r\n");
        return;
    }

    uint32_t ver = Xil_In32(REG_FPGA_VERSION);
    xil_printf("\r\n================ TLC5940 Status (Doc v1.1.0.0) ================\r\n");
    xil_printf(" FPGA Version: 0x%08X %s\r\n", ver, (ver == EXPECTED_FPGA_VERSION) ? "(OK: 3.0.0.0)" : "(MISMATCH!)");
    xil_printf(" Commit State: %s\r\n", (Xil_In32(REG_TLC5940_COMMIT) & 0x01) ? "BUSY" : "READY");

    xil_printf("\r\n [Fixation Lamps (15)]\r\n");
    for (int i = 0; i < 15; i++) {
        uint32_t mosi_val = Xil_In32(REG_MOSI_FIXATION_BASE + i * 4) & 0x3F;
        uint32_t miso_val = Xil_In32(REG_MISO_FIXATION_BASE + i * 4) & 0x3F;
        xil_printf("  #%02d: MOSI=%2d | MISO(prev)=%2d%s", i, mosi_val, miso_val, ((i % 3 == 2) || (i == 14)) ? "\r\n" : "  ||  ");
    }

    xil_printf("\r\n [Infrared Fill Lamps (6)]\r\n");
    for (int i = 0; i < 6; i++) {
        uint32_t mosi_val = Xil_In32(REG_MOSI_INFRARED_BASE + i * 4) & 0x3F;
        uint32_t miso_val = Xil_In32(REG_MISO_INFRARED_BASE + i * 4) & 0x3F;
        xil_printf("  #%d: MOSI=%2d | MISO(prev)=%2d%s", i, mosi_val, miso_val, (i % 2 == 1) ? "\r\n" : "  ||  ");
    }

    xil_printf("\r\n [Alignment Lamps (2)]\r\n");
    xil_printf("  Left : MOSI=%2d | MISO(prev)=%2d\r\n", Xil_In32(REG_MOSI_ALIGN_LEFT) & 0x3F, Xil_In32(REG_MISO_ALIGN_LEFT) & 0x3F);
    xil_printf("  Right: MOSI=%2d | MISO(prev)=%2d\r\n", Xil_In32(REG_MOSI_ALIGN_RIGHT) & 0x3F, Xil_In32(REG_MISO_ALIGN_RIGHT) & 0x3F);
    xil_printf("================================================================\r\n");
}
#endif
#if 1
#include "bglamp_drv.h"
#include "cmd_shell.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"
#include <stdlib.h>
#include <string.h>
#include "xtime_l.h"

uint8_t g_pl_lamp_ready = 0;
static uint8_t s_last_fixation_id = 0xFF; // 记录上一盏点亮的固视灯，用于互斥关灯

// 本模块私有日志开关 (可通过 log_module_register 注册或内部控制)
static uint32_t s_log_led = 0;

/**
 * @brief 核心换算：将 0~63 的控制量等比放大到 12-bit PWM (0~4095)
 * @note 63 * 65 = 4095，满度完全契合，无浮点除法开销
 */
static inline uint32_t bglamp_calc_reg_val(uint8_t level)
{
    if (level == 0) return 0U;
    if (level > 63) level = 63;
    return (uint32_t)level * 65U;
}

/**
 * @brief 触发 TLC5940 整帧 384 位移出与 XLAT 锁存
 */
//static void bglamp_commit(void)
//{
//    if (!g_pl_lamp_ready) return;
//
//    // 写 [0] 为 1 触发更新
//    Xil_Out32(REG_TLC5940_COMMIT, 0x01U);
//
//    // 等待硬件移位完成并清零 (极快，通常几个微秒)
//    uint32_t timeout = 50000;
//    while ((Xil_In32(REG_TLC5940_COMMIT) & 0x01U) && timeout--) {
//        usleep(1);
//    }
//}
static int bglamp_commit(void)
{
    if (!g_pl_lamp_ready) {
        xil_printf("[COMMIT_ERR] Gate locked! pl_on not active.\r\n");
        return -1;
    }

    XTime t_start, t_end;
    uint32_t pre_state = Xil_In32(REG_TLC5940_COMMIT);

    // 1. 监测写前状态：如果已经是 1，说明上一帧根本没跑完，本次写入会被 PL 直接吞掉！
    if (pre_state & 0x01U) {
        xil_printf("[COMMIT_WARN] Pre-check: [0]==1 (BUSY)! Hardware did not clear.\r\n");
    }

    // 2. 记录触发前的时间戳
    XTime_GetTime(&t_start);

    // 3. 写入 1 触发更新
    Xil_Out32(REG_TLC5940_COMMIT, 0x01U);

    // 4. 等待 PL 清回 0，并记录循环轮询次数[cite: 1, 2]
    uint32_t loop_cnt = 0;
    uint32_t timeout = 100000; // 适当放宽保护门限
    while ((Xil_In32(REG_TLC5940_COMMIT) & 0x01U) && timeout--) {
        loop_cnt++;
    }

    XTime_GetTime(&t_end);

    // 5. 计算 PL 执行耗时 (微秒 us)
    // Cortex-A9 Global Timer 计数值转微秒: (t_diff * 1000000) / COUNTS_PER_SECOND
    uint32_t elapsed_us = (uint32_t)((t_end - t_start) * 1000000ULL / COUNTS_PER_SECOND);

    // 6. 关键调试打印：输出触发时刻、补光灯当前写入值、PL硬件清零耗时[cite: 1, 2]
    uint32_t ir0_val = Xil_In32(REG_MOSI_FILL_BASE) & 0x0FFFU;

    if (timeout == 0) {
        xil_printf("\r\n[COMMIT_FAIL] IR0_PWM=%u | TIMEOUT > %u us | [0] STUCK AT 1!\r\n",
                   ir0_val, elapsed_us);
        return -2;
    } else {
        xil_printf("\r\n[COMMIT_OK] IR0_PWM=%4u | PL_Ack_Time=%u us (loops=%u)\r\n",
                   ir0_val, elapsed_us, loop_cnt);
        return 0;
    }
}
// =========================================================================
// 驱动初始化与门限控制
// =========================================================================
uint32_t g_pl_fpga_version = 0;

int bglamp_drv_init(void)
{
    // 挂载到统一 Shell 扩展槽位
    cmd_shell_register_ext("BGLAMP", bglamp_shell_cmd, bglamp_shell_help);

    // 默认开启门限以便读写
    g_pl_lamp_ready = 1;

    // 动态读取 PL 固件版本字 (0x4000_0000)
    g_pl_fpga_version = Xil_In32(REG_FPGA_VERSION);

    // 仅做 AXI 总线物理存活检测，不限制具体版本号
    if (g_pl_fpga_version == 0x00000000U || g_pl_fpga_version == 0xFFFFFFFFU) {
        xil_printf("[BGLAMP_WARN] PL Bitstream not loaded or AXI bus offline! (Raw: 0x%08X)\r\n", g_pl_fpga_version);
    } else {
        uint8_t major = (uint8_t)((g_pl_fpga_version >> 24) & 0xFF);
        uint8_t minor = (uint8_t)((g_pl_fpga_version >> 16) & 0xFF);
        uint8_t patch = (uint8_t)((g_pl_fpga_version >> 8)  & 0xFF);
        uint8_t build = (uint8_t)(g_pl_fpga_version & 0xFF);

        xil_printf("[BGLAMP] PL FPGA Firmware: v%d.%d.%d.%d (0x%08X) Online.\r\n",
                   major, minor, patch, build, g_pl_fpga_version);
    }

    // 上电默认关闭全部灯光
    bglamp_turn_off_all();
    return 0;
}

void bglamp_set_gate(uint8_t enable)
{
    g_pl_lamp_ready = enable ? 1 : 0;
    xil_printf("[BGLAMP] AXI Gate 0x4000_xxxx is now %s\r\n", g_pl_lamp_ready ? "UNLOCKED (ON)" : "LOCKED (OFF)");
}

void bglamp_set_mode(uint8_t mode)
{
    (void)mode;
    xil_printf("[BGLAMP] Fixed 12-bit GS Mode (0~4095 PWM) by FPGA 3.1.0.0 design.\r\n");
}

// =========================================================================
// 灯光业务控制接口
// =========================================================================

void bglamp_turn_off_all(void)
{
    if (!g_pl_lamp_ready) return;

    // 15 路固视灯写 0
    for (uint32_t i = 0; i < FIXATION_LAMP_NUM; i++) {
        Xil_Out32(REG_MOSI_FIXATION_BASE + (i * 4U), 0U);
    }
    // 6 路红外补光灯写 0
    for (uint32_t i = 0; i < FILL_LAMP_NUM; i++) {
        Xil_Out32(REG_MOSI_FILL_BASE + (i * 4U), 0U);
    }
    // 2 路对位灯写 0
    Xil_Out32(REG_MOSI_LEFT_ALIGN, 0U);
    Xil_Out32(REG_MOSI_RIGHT_ALIGN, 0U);

    s_last_fixation_id = 0xFF;
    bglamp_commit();
}

void bglamp_set_fixation_single(uint8_t lamp_id, uint8_t level)
{
    if (!g_pl_lamp_ready || lamp_id >= FIXATION_LAMP_NUM) return;

    // 互斥：先熄灭上一盏
    if (s_last_fixation_id < FIXATION_LAMP_NUM && s_last_fixation_id != lamp_id) {
        Xil_Out32(REG_MOSI_FIXATION_BASE + ((uint32_t)s_last_fixation_id * 4U), 0U);
    }

    uint32_t pwm = bglamp_calc_reg_val(level);
    Xil_Out32(REG_MOSI_FIXATION_BASE + ((uint32_t)lamp_id * 4U), pwm);
    s_last_fixation_id = (level > 0) ? lamp_id : 0xFF;

    bglamp_commit();
}

void bglamp_set_fixation_all(uint8_t level)
{
    if (!g_pl_lamp_ready) return;

    uint32_t pwm = bglamp_calc_reg_val(level);
    for (uint32_t i = 0; i < FIXATION_LAMP_NUM; i++) {
        Xil_Out32(REG_MOSI_FIXATION_BASE + (i * 4U), pwm);
    }
    s_last_fixation_id = 0xFF; // 全开状态打破单灯互斥
    bglamp_commit();
}

void bglamp_set_infrared_single(uint8_t lamp_id, uint8_t level)
{
    if (!g_pl_lamp_ready || lamp_id >= FILL_LAMP_NUM) return;

    uint32_t pwm = bglamp_calc_reg_val(level);
    Xil_Out32(REG_MOSI_FILL_BASE + ((uint32_t)lamp_id * 4U), pwm);
    bglamp_commit();
}

void bglamp_set_infrared_all(uint8_t level)
{
    if (!g_pl_lamp_ready) return;

    uint32_t pwm = bglamp_calc_reg_val(level);
    for (uint32_t i = 0; i < FILL_LAMP_NUM; i++) {
        Xil_Out32(REG_MOSI_FILL_BASE + (i * 4U), pwm);
    }
    bglamp_commit();
}

void bglamp_set_align_left(uint8_t level)
{
    if (!g_pl_lamp_ready) return;
    Xil_Out32(REG_MOSI_LEFT_ALIGN, bglamp_calc_reg_val(level));
    bglamp_commit();
}

void bglamp_set_align_right(uint8_t level)
{
    if (!g_pl_lamp_ready) return;
    Xil_Out32(REG_MOSI_RIGHT_ALIGN, bglamp_calc_reg_val(level));
    bglamp_commit();
}

void bglamp_set_align_both(uint8_t level)
{
    if (!g_pl_lamp_ready) return;
    uint32_t pwm = bglamp_calc_reg_val(level);
    Xil_Out32(REG_MOSI_LEFT_ALIGN, pwm);
    Xil_Out32(REG_MOSI_RIGHT_ALIGN, pwm);
    bglamp_commit();
}

// =========================================================================
// 状态打印与遥测
// =========================================================================
void bglamp_print_infrared_status(void)
{
    xil_printf("\r\n--- [Infrared Fill Light Status (12-bit GS)] ---\r\n");
    for (uint32_t i = 0; i < FILL_LAMP_NUM; i++) {
        uint32_t mosi = Xil_In32(REG_MOSI_FILL_BASE + (i * 4U)) & 0x0FFFU;
        uint32_t sout = Xil_In32(REG_MISO_FILL_BASE + (i * 4U)) & 0x0FFFU;
        xil_printf("  Lamp_%u: PWM=%4u [%s] | SOUT_Bits=0x%03X\r\n",
                   i, mosi, (mosi > 0) ? "ON" : "OFF", sout);
    }
}

void bglamp_dump_status(void)
{
    uint32_t ver = Xil_In32(REG_FPGA_VERSION);
    uint32_t cmt = Xil_In32(REG_TLC5940_COMMIT);

    uint8_t major = (uint8_t)((ver >> 24) & 0xFF);
    uint8_t minor = (uint8_t)((ver >> 16) & 0xFF);
    uint8_t patch = (uint8_t)((ver >> 8)  & 0xFF);
    uint8_t build = (uint8_t)(ver & 0xFF);

    xil_printf("\r\n================ TLC5940 Status ================\r\n");
    xil_printf("  FPGA Version: v%d.%d.%d.%d (Raw: 0x%08X)\r\n", major, minor, patch, build, ver);
    xil_printf("  Commit State: %s (0x%08X)\r\n", (cmt & 1) ? "BUSY" : "READY", cmt);

    xil_printf("\r\n  [Fixation Lamps (15)]\r\n");
    for (uint32_t i = 0; i < FIXATION_LAMP_NUM; i++) {
        uint32_t mosi = Xil_In32(REG_MOSI_FIXATION_BASE + (i * 4U)) & 0x0FFFU;
        uint32_t sout = Xil_In32(REG_MISO_FIXATION_BASE + (i * 4U)) & 0x0FFFU;
        xil_printf("  #%02u: MOSI=%4u | SOUT=%4u%s", i, mosi, sout, ((i + 1) % 3 == 0) ? "\r\n" : "  ||  ");
    }

    xil_printf("\r\n  [Infrared Fill Lamps (6)]\r\n");
    for (uint32_t i = 0; i < FILL_LAMP_NUM; i++) {
        uint32_t mosi = Xil_In32(REG_MOSI_FILL_BASE + (i * 4U)) & 0x0FFFU;
        uint32_t sout = Xil_In32(REG_MISO_FILL_BASE + (i * 4U)) & 0x0FFFU;
        xil_printf("  #%u: MOSI=%4u | SOUT=%4u%s", i, mosi, sout, ((i + 1) % 2 == 0) ? "\r\n" : "  ||  ");
    }

    xil_printf("\r\n  [Alignment Lamps (2)]\r\n");
    uint32_t l_mosi = Xil_In32(REG_MOSI_LEFT_ALIGN) & 0x0FFFU;
    uint32_t l_sout = Xil_In32(REG_MISO_LEFT_ALIGN) & 0x0FFFU;
    uint32_t r_mosi = Xil_In32(REG_MOSI_RIGHT_ALIGN) & 0x0FFFU;
    uint32_t r_sout = Xil_In32(REG_MISO_RIGHT_ALIGN) & 0x0FFFU;
    xil_printf("  Left : MOSI=%4u | SOUT=%4u\r\n", l_mosi, l_sout);
    xil_printf("  Right: MOSI=%4u | SOUT=%4u\r\n", r_mosi, r_sout);
    xil_printf("  *Note: SOUT is 384-bit TLC5940 cascade shift register status, not PWM readback.\r\n");
    xil_printf("================================================\r\n");
}

// =========================================================================
// 嵌入式 Shell 扩展槽位实现
// =========================================================================
int bglamp_shell_cmd(int argc, char *argv[])
{
    if (argc == 0 || argv[0] == NULL) return 0;

    if (strcmp(argv[0], "pl_on") == 0) {
        bglamp_set_gate(1);
        return 1;
    }
    if (strcmp(argv[0], "pl_off") == 0) {
        bglamp_set_gate(0);
        return 1;
    }

    if (strcmp(argv[0], "led") == 0) {
        if (argc < 2) {
            xil_printf("Usage: led <status|off|fix|fixall|ir|irall|align>\r\n");
            return 1;
        }

        if (strcmp(argv[1], "status") == 0) {
            bglamp_dump_status();
            return 1;
        }
        if (strcmp(argv[1], "off") == 0) {
            bglamp_turn_off_all();
            xil_printf("[LED] All 23 lamps turned OFF\r\n");
            return 1;
        }
        if (strcmp(argv[1], "fix") == 0 && argc >= 4) {
            bglamp_set_fixation_single((uint8_t)atoi(argv[2]), (uint8_t)atoi(argv[3]));
            xil_printf("[LED] Fixation #%s set to %s\r\n", argv[2], argv[3]);
            return 1;
        }
        if (strcmp(argv[1], "fixall") == 0 && argc >= 3) {
            bglamp_set_fixation_all((uint8_t)atoi(argv[2]));
            xil_printf("[LED] All Fixation set to %s\r\n", argv[2]);
            return 1;
        }
        if (strcmp(argv[1], "ir") == 0 && argc >= 4) {
            bglamp_set_infrared_single((uint8_t)atoi(argv[2]), (uint8_t)atoi(argv[3]));
            xil_printf("[LED] Infrared #%s set to %s\r\n", argv[2], argv[3]);
            return 1;
        }
        if (strcmp(argv[1], "irall") == 0 && argc >= 3) {
            bglamp_set_infrared_all((uint8_t)atoi(argv[2]));
            xil_printf("[LED] All Infrared set to %s\r\n", argv[2]);
            return 1;
        }
        if (strcmp(argv[1], "align") == 0 && argc >= 4) {
            uint8_t lvl = (uint8_t)atoi(argv[3]);
            if (strcmp(argv[2], "l") == 0)         bglamp_set_align_left(lvl);
            else if (strcmp(argv[2], "r") == 0)    bglamp_set_align_right(lvl);
            else if (strcmp(argv[2], "both") == 0) bglamp_set_align_both(lvl);
            xil_printf("[LED] Alignment %s set to %d\r\n", argv[2], lvl);
            return 1;
        }

        xil_printf("Unknown led command. Type 'help'.\r\n");
        return 1;
    }

    return 0; // 不是本模块命令
}

void bglamp_shell_help(void)
{
    xil_printf("  [Ophthalmic Lamps (TLC5940 v1.2.0.0)]\r\n");
    xil_printf("    pl_on / pl_off              : Enable/Disable AXI gate\r\n");
    xil_printf("    led status                  : View all 23 lamps & SOUT telemetry\r\n");
    xil_printf("    led off                     : Turn off all 23 lamps\r\n");
    xil_printf("    led fix <id:0-14> <0-63>    : Set single fixation lamp (exclusive)\r\n");
    xil_printf("    led fixall <0-63>           : Set all 15 fixation lamps\r\n");
    xil_printf("    led ir <id:0-5> <0-63>      : Set single infrared lamp\r\n");
    xil_printf("    led irall <0-63>            : Set all 6 infrared lamps\r\n");
    xil_printf("    led align <l|r|both> <0-63> : Set alignment lamps\r\n");
}


#endif
