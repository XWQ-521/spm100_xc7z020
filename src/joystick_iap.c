#include "joystick_iap.h"
#include "hal_uart.h"
#include "hal_motor.h"
#include "cmd_shell.h"
#include "xil_printf.h"
#include "sleep.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// =========================================================================
// HID 协议与 YModem 协议宏定义
// =========================================================================
#define HID_REPORT_SIZE        64
#define HID_REPORT_ID          0x01

#define YM_SOH                 0x01
#define YM_STX                 0x02
#define YM_EOT                 0x04
#define YM_ACK                 0x06
#define YM_NAK                 0x15
#define YM_CAN                 0x18
#define YM_C                   0x43

#define PACKET_PAYLOAD_SIZE    1024
#define TOTAL_FRAME_SIZE       (1 + 1 + 1 + PACKET_PAYLOAD_SIZE + 2) // 1029 字节
#define MAX_RETRIES            3
#define ACK_TIMEOUT_MS         5000





// 升级会话控制块
typedef struct {
    iap_state_t state;
    uint8_t err_code;
    const uint8_t *fw_data;
    uint32_t fw_size;
    uint32_t bytes_sent;
    char file_name[32];

    uint8_t packet_num;
    uint8_t retry_count;
    uint32_t timer_ms;

    volatile uint8_t rx_byte;
    volatile bool has_rx;

    bool got_ack;
    bool got_c;
} joystick_iap_ctrl_t;

static joystick_iap_ctrl_t s_ctx;
volatile uint8_t g_is_joystick_iap_active = 0;

// =========================================================================
// 1. 本地辅助：CRC16-CCITT 计算 (多项式 0x1021，初值 0x0000)
// =========================================================================
static uint16_t calc_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0x0000;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc = (crc << 1);
        }
    }
    return crc;
}

// =========================================================================
// 2. 本地辅助：串口 HID 报文封装与切片发送
// =========================================================================

// 发送单个固定 64 字节的 HID Output Report
static void send_hid_report(const uint8_t *payload, uint16_t len)
{
    uint8_t report[HID_REPORT_SIZE];
    memset(report, 0, sizeof(report));

    report[0] = HID_REPORT_ID; // Report ID: 0x01
    if (payload != NULL && len > 0) {
        uint16_t copy_len = (len > (HID_REPORT_SIZE - 1)) ? (HID_REPORT_SIZE - 1) : len;
        memcpy(&report[1], payload, copy_len);
    }

    // 经底层串口发出
    joystick_uart_send_bytes(report, HID_REPORT_SIZE);
}

// 发送进入 Bootloader 的 JSON 复位指令: {"func":"system reset"}
static void send_hid_reset_cmd(void)
{
    const uint8_t reset_frame[4] = { 0xAA, 0x02, 0xA8, 0x55 };
    joystick_uart_send_bytes(reset_frame, sizeof(reset_frame));
}

// 将 1029 字节的大帧切片封装为多个 64 字节 HID 报文顺序发出
static void send_hid_ymodem_slices(const uint8_t *frame_data, uint16_t frame_len)
{
    uint16_t sent = 0;
    while (sent < frame_len) {
        uint16_t chunk = frame_len - sent;
        if (chunk > (HID_REPORT_SIZE - 1)) {
            chunk = (HID_REPORT_SIZE - 1); // 每次有效载荷最多 63 字节
        }

        send_hid_report(&frame_data[sent], chunk);
        sent += chunk;
        usleep(300); // 片间防硬件 FIFO 溢出
    }
}

// 构建标准 YModem 帧并切片发送
// 直接通过串口发送标准 YModem 1029 字节原始帧（不加 0x01 HID 头）
static void send_stx_frame(uint8_t packet_num, const uint8_t *payload)
{
    uint8_t frame[TOTAL_FRAME_SIZE];
    frame[0] = YM_STX;
    frame[1] = packet_num;
    frame[2] = (uint8_t)(~packet_num);

    memcpy(&frame[3], payload, PACKET_PAYLOAD_SIZE);

    uint16_t crc = calc_crc16(payload, PACKET_PAYLOAD_SIZE);
    frame[3 + PACKET_PAYLOAD_SIZE]     = (uint8_t)(crc >> 8);
    frame[3 + PACKET_PAYLOAD_SIZE + 1] = (uint8_t)(crc & 0xFF);

    joystick_uart_send_bytes(frame, TOTAL_FRAME_SIZE);
}
// =========================================================================
// 3. IAP 核心状态机实现
// =========================================================================

void joystick_iap_feed_rx_byte(uint8_t byte)
{
	s_ctx.rx_byte = byte;
	s_ctx.has_rx = true;
	if (byte == YM_ACK) s_ctx.got_ack = true;
	if (byte == YM_C)   s_ctx.got_c   = true;
}

int joystick_iap_start(const uint8_t *fw_bin, uint32_t fw_size, const char *file_name)
{
    if (fw_bin == NULL || fw_size == 0) {
        xil_printf("[IAP_ERR] Firmware buffer empty!\r\n");
        return -1;
    }
    if (fw_size > JOYSTICK_FLASH_APP_MAX_SIZE) {
        xil_printf("[IAP_ERR] Firmware size (%u B) > APP partition limit (%u B)!\r\n",
                   fw_size, JOYSTICK_FLASH_APP_MAX_SIZE);
        return -2;
    }

    // 升级启动时强制关停所有电机，保证设备安全
    motor_stop(TMC2660_M1);
    motor_stop(TMC2660_M2);
    motor_stop(TMC2660_M3);

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.fw_data = fw_bin;
    s_ctx.fw_size = fw_size;
    s_ctx.packet_num = 1;
    strncpy(s_ctx.file_name, file_name ? file_name : "joystick.bin", sizeof(s_ctx.file_name) - 1);

    // 开启串口门禁分流
    g_is_joystick_iap_active = 1;

    s_ctx.state = IAP_STATE_SEND_RESET_CMD;
    s_ctx.timer_ms = 0;
    s_ctx.retry_count = 0;

    xil_printf("\r\n=======================================================\r\n");
    xil_printf("[IAP] Joystick UART-HID Upgrade Started.\r\n");
    xil_printf("      Target Size: %u B, Image Name: %s\r\n", fw_size, s_ctx.file_name);
    xil_printf("=======================================================\r\n");
    return 0;
}

void joystick_iap_poll(void)
{
    if (!g_is_joystick_iap_active) {
        return;
    }
    static uint64_t last_tick_us = 0;
	uint64_t now_us = timer_get_1us();
	if (timer_get_diff_1us(last_tick_us, now_us) < 1000) {
		return; // 严格 1ms 节拍
	}
	last_tick_us = now_us;
    s_ctx.timer_ms++;

    uint8_t rx = 0;
    bool has_new_rx = false;
    if (s_ctx.has_rx) {
        rx = s_ctx.rx_byte;
        s_ctx.has_rx = false;
        has_new_rx = true;
    }

    switch (s_ctx.state) {
        case IAP_STATE_SEND_RESET_CMD:
        	xil_printf("[IAP] 1. Sending UART reset command: AA 02 A8 55...\r\n");
			s_ctx.has_rx  = false;
			s_ctx.got_ack = false;
			s_ctx.got_c   = false;
			send_hid_reset_cmd();
			s_ctx.state = IAP_STATE_WAIT_RESET_STABLE;
			s_ctx.timer_ms = 0;
            break;

        case IAP_STATE_WAIT_RESET_STABLE:
            if (s_ctx.timer_ms >= 600) { // 等待从机 MCU 软复位进入 Bootloader
                s_ctx.has_rx = false;
                s_ctx.got_ack = false;
                s_ctx.got_c = false;
                s_ctx.state = IAP_STATE_SEND_PACKET_0;
            }
            break;

        case IAP_STATE_SEND_PACKET_0: {
            xil_printf("[IAP] 2. Sending Packet 0 (Header)...\r\n");
            uint8_t payload[PACKET_PAYLOAD_SIZE];
            memset(payload, 0x00, PACKET_PAYLOAD_SIZE);

            int name_len = strlen(s_ctx.file_name);
            memcpy(payload, s_ctx.file_name, name_len);
            payload[name_len] = '\0';
            snprintf((char*)&payload[name_len + 1], PACKET_PAYLOAD_SIZE - name_len - 1, "%u", s_ctx.fw_size);

            send_stx_frame(0, payload);
            s_ctx.state = IAP_STATE_WAIT_ACK_AND_C;
            s_ctx.timer_ms = 0;
            break;
        }

        case IAP_STATE_WAIT_ACK_AND_C:
            if (has_new_rx) {
                if (rx == YM_ACK) s_ctx.got_ack = true;
                if (rx == YM_C)   s_ctx.got_c = true;
            }

            if (s_ctx.got_ack && s_ctx.got_c) {
                xil_printf("[IAP] 3. Packet 0 ACK + C received! Waiting 1s for Flash Erase...\r\n");
                s_ctx.state = IAP_STATE_FLASH_ERASE_DELAY;
                s_ctx.timer_ms = 0;
            } else if (s_ctx.timer_ms > ACK_TIMEOUT_MS) {
                if (++s_ctx.retry_count >= MAX_RETRIES) {
                	s_ctx.err_code = IAP_ERR_PKT0_TIMEOUT; // <--- 0x04
                    xil_printf("[IAP_ERR] Packet 0 ACK+C timeout!\r\n");
                    s_ctx.state = IAP_STATE_FAILED;
                } else {
                    xil_printf("[IAP_WARN] Retrying Packet 0...\r\n");
                    s_ctx.got_ack = false;
                    s_ctx.got_c = false;
                    s_ctx.state = IAP_STATE_SEND_PACKET_0;
                }
            }
            break;

        case IAP_STATE_FLASH_ERASE_DELAY:
            if (s_ctx.timer_ms >= 1000) { // 严格等待 1.0 秒擦除耗时
                s_ctx.bytes_sent = 0;
                s_ctx.packet_num = 1;
                s_ctx.retry_count = 0;
                xil_printf("[IAP] 4. Flash Erased. Starting data transmission...\r\n");
                s_ctx.state = IAP_STATE_SEND_DATA_PACKET;
            }
            break;

        case IAP_STATE_SEND_DATA_PACKET: {
            uint32_t remaining = s_ctx.fw_size - s_ctx.bytes_sent;
            if (remaining == 0) {
            	xil_printf("[IAP] 5. All data sent. Sending raw EOT...\r\n");
				uint8_t eot_val = YM_EOT;
				joystick_uart_send_bytes(&eot_val, 1);
				s_ctx.state = IAP_STATE_WAIT_EOT_ACK;
				s_ctx.timer_ms = 0;
				break;
            }

            uint8_t payload[PACKET_PAYLOAD_SIZE];
            memset(payload, 0x1A, PACKET_PAYLOAD_SIZE); // 不足处填充 0x1A

            uint32_t chunk = (remaining > PACKET_PAYLOAD_SIZE) ? PACKET_PAYLOAD_SIZE : remaining;
            memcpy(payload, &s_ctx.fw_data[s_ctx.bytes_sent], chunk);

            send_stx_frame(s_ctx.packet_num, payload);
            s_ctx.state = IAP_STATE_WAIT_DATA_ACK;
            s_ctx.timer_ms = 0;
            break;
        }

        case IAP_STATE_WAIT_DATA_ACK:
            if (has_new_rx) {
                if (rx == YM_ACK) {
                    uint32_t remaining = s_ctx.fw_size - s_ctx.bytes_sent;
                    uint32_t chunk = (remaining > PACKET_PAYLOAD_SIZE) ? PACKET_PAYLOAD_SIZE : remaining;
                    s_ctx.bytes_sent += chunk;
                    s_ctx.packet_num++;
                    s_ctx.retry_count = 0;

                    uint8_t progress = (uint8_t)((s_ctx.bytes_sent * 100ULL) / s_ctx.fw_size);
                    xil_printf("[IAP Progress] %u%% (%u/%u B) Pkt#%u ACKed\r\n",
                               progress, s_ctx.bytes_sent, s_ctx.fw_size, s_ctx.packet_num - 1);

                    s_ctx.state = IAP_STATE_PACKET_DELAY;
                    s_ctx.timer_ms = 0;
                } else if (rx == YM_NAK) {
                    if (++s_ctx.retry_count >= MAX_RETRIES) {
                    	s_ctx.err_code = IAP_ERR_DATA_NAK; // <--- 0x05
                        xil_printf("[IAP_ERR] Max NAK retries exceeded on pkt %u!\r\n", s_ctx.packet_num);
                        s_ctx.state = IAP_STATE_FAILED;
                    } else {
                        s_ctx.state = IAP_STATE_SEND_DATA_PACKET;
                    }
                } else if (rx == YM_CAN) {
                	s_ctx.err_code = IAP_ERR_REMOTE_CAN; // <--- 0x07
                    xil_printf("[IAP_ERR] CAN received from joystick!\r\n");
                    s_ctx.state = IAP_STATE_FAILED;
                }
            } else if (s_ctx.timer_ms > ACK_TIMEOUT_MS) {
                if (++s_ctx.retry_count >= MAX_RETRIES) {
                	s_ctx.err_code = IAP_ERR_DATA_TIMEOUT; // <--- 0x06
                    xil_printf("[IAP_ERR] Timeout on pkt %u!\r\n", s_ctx.packet_num);
                    s_ctx.state = IAP_STATE_FAILED;
                } else {
                    s_ctx.state = IAP_STATE_SEND_DATA_PACKET;
                }
            }
            break;

        case IAP_STATE_PACKET_DELAY:
            if (s_ctx.timer_ms >= 200) { // 200ms 包间保护延时
                s_ctx.state = IAP_STATE_SEND_DATA_PACKET;
            }
            break;

        case IAP_STATE_WAIT_EOT_ACK:
            if (has_new_rx && (rx == YM_ACK || rx == YM_C)) {
                xil_printf("[IAP] 6. EOT Confirmed. Sending End Packet 0...\r\n");
                s_ctx.state = IAP_STATE_SEND_END_PACKET;
            } else if (s_ctx.timer_ms > ACK_TIMEOUT_MS) {
                xil_printf("[IAP_ERR] EOT ACK timeout!\r\n");
                s_ctx.err_code = IAP_ERR_EOT_TIMEOUT; // <--- 0x08
                s_ctx.state = IAP_STATE_FAILED;
            }
            break;

        case IAP_STATE_SEND_END_PACKET: {
            uint8_t payload[PACKET_PAYLOAD_SIZE];
            memset(payload, 0x00, PACKET_PAYLOAD_SIZE);
            send_stx_frame(0, payload);

            s_ctx.state = IAP_STATE_WAIT_END_ACK;
            s_ctx.timer_ms = 0;
            break;
        }

        case IAP_STATE_WAIT_END_ACK:
            if (has_new_rx && rx == YM_ACK) {
                xil_printf("\r\n=======================================================\r\n");
                xil_printf("[IAP SUCCESS] Bootloader COMPLETE! Joystick Resetting.\r\n");
                xil_printf("=======================================================\r\n");
                s_ctx.state = IAP_STATE_SUCCESS;
                g_is_joystick_iap_active = 0; // 退出门禁，恢复日常摇杆解析
            } else if (s_ctx.timer_ms > ACK_TIMEOUT_MS) {
            	s_ctx.err_code = IAP_ERR_END_TIMEOUT; // <--- 0x09
                xil_printf("[IAP_ERR] End Packet ACK timeout!\r\n");
                s_ctx.state = IAP_STATE_FAILED;
            }
            break;

        case IAP_STATE_SUCCESS:
            break;

        case IAP_STATE_FAILED:
            xil_printf("[IAP FATAL] Upgrade aborted! Releasing UART gate.\r\n");
            g_is_joystick_iap_active = 0; // 释放门禁，避免串口死锁
            break;

        default:
            break;
    }
}

uint8_t joystick_iap_get_progress(void)
{
    if (s_ctx.fw_size == 0) return 0;
    return (uint8_t)((s_ctx.bytes_sent * 100ULL) / s_ctx.fw_size);
}

iap_state_t joystick_iap_get_state(void)
{
    return s_ctx.state;
}

// =========================================================================
// 4. 终端 Shell 命令实现与注册
// =========================================================================
static void cmd_iap_help(void)
{
    xil_printf("\r\n--- [Joystick IAP Upgrade Command Menu] ---\r\n");
    xil_printf("  iap start <hex_addr> <size> : Start IAP with firmware at DDR address\r\n");
    xil_printf("  iap abort                   : Force abort upgrade & unlock UART gate\r\n");
    xil_printf("  iap status                  : Query current IAP state & progress\r\n");
    xil_printf("  iap help                    : Show this help menu\r\n");
    xil_printf("-------------------------------------------\r\n\r\n");
}

static int cmd_iap(int argc, char *argv[])
{
    if (argc < 2 || strcmp(argv[1], "help") == 0) {
        cmd_iap_help();
        return 1;
    }

    if (strcmp(argv[1], "status") == 0) {
        xil_printf("[IAP] Active: %d | State: %d | Progress: %u%%\r\n",
                   g_is_joystick_iap_active, s_ctx.state, joystick_iap_get_progress());
        return 1;
    }

    if (strcmp(argv[1], "abort") == 0) {
        g_is_joystick_iap_active = 0;
        s_ctx.state = IAP_STATE_IDLE;
        xil_printf("[IAP] Forcibly aborted. Normal UART communication resumed.\r\n");
        return 1;
    }

    if (strcmp(argv[1], "start") == 0) {
        if (argc >= 4) {
            uint32_t addr = (uint32_t)strtoul(argv[2], NULL, 0);
            uint32_t size = (uint32_t)strtoul(argv[3], NULL, 0);

            if (size == 0 || size > JOYSTICK_FLASH_APP_MAX_SIZE) {
                xil_printf("[IAP_CMD] Invalid size %u! Must be 1 ~ %d B.\r\n", size, JOYSTICK_FLASH_APP_MAX_SIZE);
                return 1;
            }

            int ret = joystick_iap_start((const uint8_t*)addr, size, "joystick_app.bin");
            if (ret != 0) {
                xil_printf("[IAP_CMD] Start failed: %d\r\n", ret);
            }
            return 1;
        }
        xil_printf("Usage: iap start <hex_addr> <size>\r\n");
        return 1;
    }

    xil_printf("Unknown argument. Type 'iap help'.\r\n");
    return 1;
}

void joystick_iap_cmd_register(void)
{
    cmd_shell_register_ext("IAP", cmd_iap, cmd_iap_help);
}
uint8_t joystick_iap_get_err_code(void)
{
    return s_ctx.err_code;
}
