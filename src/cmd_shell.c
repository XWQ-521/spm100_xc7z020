#include "cmd_shell.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "xil_io.h"
#include "xil_printf.h"
#include "xuartps.h"
#include "sleep.h"
#include "joystick_upgrade.h"
uint32_t g_log_mask = LOG_MOD_SYS | LOG_MOD_LIMIT | LOG_MOD_MOTOR | LOG_MOD_TCP | LOG_MOD_LED;

#define SHELL_RX_BUF_SIZE  128
#define HISTORY_DEPTH      16
#define MAX_EXT_SLOTS      8
#define CMD_DELIM          " \t\r\n,"

// =========================================================================
// 1. 业务扩展槽位管理
// =========================================================================
typedef struct {
    const char        *mod_name;
    shell_cmd_func_t   cmd_cb;
    shell_help_func_t  help_cb;
} shell_slot_t;

static shell_slot_t s_slots[MAX_EXT_SLOTS];
static uint8_t      s_slot_count = 0;

int cmd_shell_register_ext(const char *mod_name, shell_cmd_func_t cmd_cb, shell_help_func_t help_cb)
{
    if (s_slot_count >= MAX_EXT_SLOTS) {
        xil_printf("[SHELL_ERR] Ext slots full!\r\n");
        return -1;
    }
    s_slots[s_slot_count].mod_name = mod_name;
    s_slots[s_slot_count].cmd_cb   = cmd_cb;
    s_slots[s_slot_count].help_cb  = help_cb;
    s_slot_count++;
    return 0;
}

// =========================================================================
// 2. VT100 终端与历史记录引擎 (移植自 CAS 项目)
// =========================================================================
static char s_history[HISTORY_DEPTH][SHELL_RX_BUF_SIZE];
static int  s_history_len = 0;

static void out_char(char c) {
    xil_printf("%c", c);
}

static void emit_csi(char final) {
    out_char('\x1b'); out_char('['); out_char(final);
}
static void cursor_left_n(int n)  { while (n-- > 0) emit_csi('D'); }

static void history_push(const char *line) {
    if (line[0] == '\0') return;
    if ((s_history_len > 0) && (strcmp(s_history[s_history_len - 1], line) == 0)) return;
    if (s_history_len == HISTORY_DEPTH) {
        memmove(s_history[0], s_history[1], (size_t)(HISTORY_DEPTH - 1) * SHELL_RX_BUF_SIZE);
        s_history_len--;
    }
    strncpy(s_history[s_history_len], line, SHELL_RX_BUF_SIZE - 1);
    s_history[s_history_len][SHELL_RX_BUF_SIZE - 1] = '\0';
    s_history_len++;
}

static void replace_line(char *buf, int *len, int *cursor, const char *src) {
    cursor_left_n(*cursor);
    emit_csi('K'); // 清除当前行光标后内容
    *len = 0;
    while ((src[*len] != '\0') && (*len < SHELL_RX_BUF_SIZE - 1)) {
        buf[*len] = src[*len];
        out_char(src[*len]);
        (*len)++;
    }
    buf[*len] = '\0';
    *cursor = *len;
}

// =========================================================================
// 3. 通用系统基础命令实现 (所有项目 100% 共享)
// =========================================================================
static uint32_t parse_hex(const char *str) {
    return (uint32_t)strtoul(str, NULL, 0);
}

static void print_unified_help(void) {
    xil_printf("\r\n================ Global Shell Command Menu ================\r\n");
    xil_printf("  [Common System & Register Tools]\r\n");
    xil_printf("    help / ?                    : Show this menu\r\n");
    xil_printf("    history                     : View command history\r\n");
    xil_printf("    r8  / w8  <addr> [val]      : 8-bit register Read / Write\r\n");
    xil_printf("    r16 / w16 <addr> [val]      : 16-bit register Read / Write\r\n");
    xil_printf("    r32 / w32 <addr> [val]      : 32-bit register Read / Write\r\n");
    xil_printf("    dump   <addr> [count]       : Dump 32-bit continuous memory\r\n");
    xil_printf("    setbit / clrbit <addr> <bit>: Bit Set / Clear (0~31)\r\n");
    xil_printf("    poll   <addr> <bit> <0|1>   : Poll bit until matched\r\n");

    // 核心预留槽位：自动遍历调用各模块专属的帮助打印槽函数
    for (int i = 0; i < s_slot_count; i++) {
        if (s_slots[i].help_cb != NULL) {
            s_slots[i].help_cb();
        }
    }
    xil_printf("===========================================================\r\nCMD> ");
}

static int execute_common_commands(int argc, char *argv[]) {
    if (strcmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0) {
        print_unified_help();
        return 1;
    }
    if (strcmp(argv[0], "history") == 0) {
        xil_printf("\r\n--- Command History (%d) ---\r\n", s_history_len);
        for (int i = 0; i < s_history_len; i++) xil_printf("  %2d  %s\r\n", i + 1, s_history[i]);
        xil_printf("----------------------------\r\n");
        return 1;
    }
//    if (strcmp(argv[0], "log") == 0) {
//        if (argc >= 2 && strcmp(argv[1], "status") == 0) {
//            xil_printf("Log Mask: 0x%08X\r\n", g_log_mask);
//            return 1;
//        }
//        if (argc >= 3) {
//            uint8_t en = (strcmp(argv[1], "on") == 0);
//            uint32_t mod = 0;
//            if (strcmp(argv[2], "joy") == 0)        mod = LOG_MOD_JOY;
//            else if (strcmp(argv[2], "motor") == 0) mod = LOG_MOD_MOTOR;
//            else if (strcmp(argv[2], "limit") == 0) mod = LOG_MOD_LIMIT;
//            else if (strcmp(argv[2], "pid") == 0)   mod = LOG_MOD_PID;
//            else if (strcmp(argv[2], "tcp") == 0)   mod = LOG_MOD_TCP;
//            else if (strcmp(argv[2], "led") == 0)   mod = LOG_MOD_LED;
//            else if (strcmp(argv[2], "all") == 0)   mod = LOG_MOD_ALL;
//            if (mod) {
//                if (en) g_log_mask |= mod; else g_log_mask &= ~mod;
//                xil_printf("[LOG] %s is %s\r\n", argv[2], en ? "ON" : "OFF");
//            }
//            return 1;
//        }
//    }
    // r8 / w8 / r16 / w16 / r32 / w32
    if (strcmp(argv[0], "r32") == 0 && argc >= 2) {
        uint32_t a = parse_hex(argv[1]);
        if (a & 3) xil_printf("Addr must be 4-byte aligned!\r\n");
        else xil_printf("[R32] 0x%08X = 0x%08X (%u)\r\n", a, Xil_In32(a), Xil_In32(a));
        return 1;
    }
    if (strcmp(argv[0], "w32") == 0 && argc >= 3) {
        uint32_t a = parse_hex(argv[1]), v = parse_hex(argv[2]);
        if (a & 3) xil_printf("Addr must be 4-byte aligned!\r\n");
        else { Xil_Out32(a, v); xil_printf("[W32] 0x%08X <= 0x%08X | Rb: 0x%08X\r\n", a, v, Xil_In32(a)); }
        return 1;
    }
    if (strcmp(argv[0], "r8") == 0 && argc >= 2) {
        uint32_t a = parse_hex(argv[1]);
        xil_printf("[R8]  0x%08X = 0x%02X (%u)\r\n", a, Xil_In8(a), Xil_In8(a));
        return 1;
    }
    if (strcmp(argv[0], "w8") == 0 && argc >= 3) {
        uint32_t a = parse_hex(argv[1]); uint8_t v = (uint8_t)parse_hex(argv[2]);
        Xil_Out8(a, v); xil_printf("[W8]  0x%08X <= 0x%02X | Rb: 0x%02X\r\n", a, v, Xil_In8(a));
        return 1;
    }
    if (strcmp(argv[0], "r16") == 0 && argc >= 2) {
        uint32_t a = parse_hex(argv[1]);
        xil_printf("[R16] 0x%08X = 0x%04X (%u)\r\n", a, Xil_In16(a), Xil_In16(a));
        return 1;
    }
    if (strcmp(argv[0], "w16") == 0 && argc >= 3) {
        uint32_t a = parse_hex(argv[1]); uint16_t v = (uint16_t)parse_hex(argv[2]);
        Xil_Out16(a, v); xil_printf("[W16] 0x%08X <= 0x%04X | Rb: 0x%04X\r\n", a, v, Xil_In16(a));
        return 1;
    }
    if (strcmp(argv[0], "dump") == 0 && argc >= 2) {
        uint32_t a = parse_hex(argv[1]) & ~3;
        uint32_t cnt = (argc >= 3) ? (uint32_t)atoi(argv[2]) : 16;
        if (cnt > 64) cnt = 64;
        xil_printf("\r\n=== Dump @ 0x%08X (%d words) ===\r\n", a, cnt);
        for (uint32_t i = 0; i < cnt; i++) {
            if (i % 4 == 0) xil_printf("\r\n0x%08X: ", a + i * 4);
            xil_printf("%08X  ", Xil_In32(a + i * 4));
        }
        xil_printf("\r\n================================\r\n");
        return 1;
    }
    if ((strcmp(argv[0], "setbit") == 0 || strcmp(argv[0], "clrbit") == 0) && argc >= 3) {
        uint32_t a = parse_hex(argv[1]); uint8_t bit = (uint8_t)atoi(argv[2]);
        if (bit <= 31 && !(a & 3)) {
            uint32_t v = Xil_In32(a);
            v = (strcmp(argv[0], "setbit") == 0) ? (v | (1U << bit)) : (v & ~(1U << bit));
            Xil_Out32(a, v);
            xil_printf("[BIT] 0x%08X bit %d -> 0x%08X\r\n", a, bit, Xil_In32(a));
        }
        return 1;
    }
    if (strcmp(argv[0], "poll") == 0 && argc >= 4) {
        uint32_t a = parse_hex(argv[1]); uint8_t bit = (uint8_t)atoi(argv[2]);
        uint8_t target = (uint8_t)atoi(argv[3]) & 1; uint32_t to = 100000;
        while (to--) {
            if (((Xil_In32(a) >> bit) & 1) == target) {
                xil_printf("[POLL_OK] Bit %d matched %d!\r\n", bit, target);
                return 1;
            }
            usleep(10);
        }
        xil_printf("[POLL_TIMEOUT] Value: 0x%08X\r\n", Xil_In32(a));
        return 1;
    }
    return 0; // 不是系统通用命令
}

// =========================================================================
// 4. 总调度派发器 (Dispatcher)
// =========================================================================
static void shell_dispatch_line(char *line) {
    char *argv[10];
    int argc = 0;

    char *token = strtok(line, CMD_DELIM);
    while (token != NULL && argc < 10) {
        argv[argc++] = token;
        token = strtok(NULL, CMD_DELIM);
    }
    if (argc == 0) return;

    // 1. 优先尝试系统级基础通用指令
    if (execute_common_commands(argc, argv)) {
        return;
    }

    // 2. 核心预留槽位：依次轮询各独立模块注册的命令槽函数
    for (int i = 0; i < s_slot_count; i++) {
        if (s_slots[i].cmd_cb != NULL) {
            if (s_slots[i].cmd_cb(argc, argv) == 1) {
                return; // 该模块已成功命中并执行
            }
        }
    }

    xil_printf("Unknown command: '%s'. Type 'help'.\r\n", argv[0]);
}

// =========================================================================
// 5. 交互式字符处理主循环 (非阻塞，带 VT100 上下键历史)
// =========================================================================
static char s_line_buf[SHELL_RX_BUF_SIZE];
static int  s_buf_len = 0;
static int  s_cursor = 0;
static int  s_browse_idx = -1;
static char s_draft[SHELL_RX_BUF_SIZE];

void cmd_shell_init(void) {
    s_buf_len = 0; s_cursor = 0; s_browse_idx = -1;
    s_draft[0] = '\0';
    xil_printf("\r\n=======================================================\r\n");
    xil_printf("   Modular Standard Shell Engine Inited (Slots: %d/%d)  \r\n", s_slot_count, MAX_EXT_SLOTS);
    xil_printf("   Type 'help' for full command list                   \r\n");
    xil_printf("=======================================================\r\nCMD> ");
}

void cmd_shell_process(void) {
#ifdef STDIN_BASEADDRESS
    while (XUartPs_IsReceiveData(STDIN_BASEADDRESS)) {
        char c = (char)XUartPs_ReadReg(STDIN_BASEADDRESS, XUARTPS_FIFO_OFFSET);

        // 处理 VT100 方向键
        if (c == '\x1b') {
            char c2 = (char)XUartPs_ReadReg(STDIN_BASEADDRESS, XUARTPS_FIFO_OFFSET);
            char arrow = 0;
            if (c2 == '[' || c2 == 'O') arrow = (char)XUartPs_ReadReg(STDIN_BASEADDRESS, XUARTPS_FIFO_OFFSET);
            if (arrow == 'A') { // 上键: 历史向上
                if (s_history_len > 0) {
                    if (s_browse_idx < 0) {
                        s_line_buf[s_buf_len] = '\0';
                        strncpy(s_draft, s_line_buf, SHELL_RX_BUF_SIZE - 1);
                        s_browse_idx = s_history_len - 1;
                    } else if (s_browse_idx > 0) {
                        s_browse_idx--;
                    }
                    replace_line(s_line_buf, &s_buf_len, &s_cursor, s_history[s_browse_idx]);
                }
            } else if (arrow == 'B') { // 下键: 历史向下
                if (s_browse_idx >= 0) {
                    if (s_browse_idx < s_history_len - 1) {
                        s_browse_idx++;
                        replace_line(s_line_buf, &s_buf_len, &s_cursor, s_history[s_browse_idx]);
                    } else {
                        s_browse_idx = -1;
                        replace_line(s_line_buf, &s_buf_len, &s_cursor, s_draft);
                    }
                }
            }
            continue;
        }

        // 回车执行
        if (c == '\r' || c == '\n') {
            out_char('\r'); out_char('\n');
            if (s_buf_len > 0) {
                s_line_buf[s_buf_len] = '\0';
                history_push(s_line_buf);
                shell_dispatch_line(s_line_buf);
                s_buf_len = 0; s_cursor = 0; s_browse_idx = -1;
            }
            xil_printf("CMD> ");
            continue;
        }

        // 退格
        if (c == '\b' || c == 0x7F) {
            if (s_cursor > 0) {
                for (int i = s_cursor - 1; i < s_buf_len - 1; i++) s_line_buf[i] = s_line_buf[i + 1];
                s_buf_len--; s_cursor--; s_line_buf[s_buf_len] = '\0';
                out_char('\b');
                for (int i = s_cursor; i < s_buf_len; i++) out_char(s_line_buf[i]);
                out_char(' ');
                cursor_left_n(s_buf_len - s_cursor + 1);
            }
            continue;
        }

        // 可打印字符输入
        if ((s_buf_len < SHELL_RX_BUF_SIZE - 1) && isprint((unsigned char)c)) {
            for (int i = s_buf_len; i > s_cursor; i--) s_line_buf[i] = s_line_buf[i - 1];
            s_line_buf[s_cursor] = c;
            s_buf_len++; s_line_buf[s_buf_len] = '\0';
            for (int i = s_cursor; i < s_buf_len; i++) out_char(s_line_buf[i]);
            s_cursor++;
            cursor_left_n(s_buf_len - s_cursor);
        }
    }
#endif
}
