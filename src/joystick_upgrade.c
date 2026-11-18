#include "joystick_upgrade.h"
#include "joystick_iap.h"
#include "hal_motor.h"
#include "protocol.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include <string.h>

extern uint8_t m_joystick_ctrl;
extern volatile uint8_t g_motor_owner;
extern struct tcp_pcb *g_current_tpcb;

static uint32_t s_fw_total_size = 0;
static uint32_t s_fw_recv_bytes = 0;
static uint8_t  s_host_addr     = 0x0A;
static uint8_t  s_host_cmd      = 0x73;
static uint8_t  s_monitoring    = 0;    // 1: 正在监控烧录状态并上报TCP
static uint8_t *s_fw_buf        = (uint8_t *)DDR_JOYSTICK_IAP_BUFFER;

// 向上位机 TCP 发送状态与错误码帧: [0x04, status, err_code, progress]
// status: 0x01=烧录中, 0x02=成功, 0x03=失败
static void report_to_host(uint8_t status, uint8_t err_code, uint8_t progress)
{
    if (g_current_tpcb != NULL && g_current_tpcb->state == ESTABLISHED) {
        uint8_t payload[4] = { JOY_IAP_SUB_CMD_REPORT, status, err_code, progress };
        protocol_send(s_host_addr, g_current_tpcb, s_host_cmd, payload, 4);
        tcp_output(g_current_tpcb);
    }
}

void Joystick_Upgrade_Init(void)
{
    s_fw_total_size = 0;
    s_fw_recv_bytes = 0;
    s_monitoring    = 0;
}

// 1. 处理上位机 TCP 传包，收完后直接启动底层烧录
void handle_joystick_iap_cmd(struct tcp_pcb *tpc, uint8_t *data, uint32_t raw_tcp_len)
{
    uint8_t sub_cmd = data[5];
    uint8_t ack_payload[2] = { sub_cmd, IAP_ERR_NONE };

    s_host_addr = data[2];
    s_host_cmd  = data[4];

    switch (sub_cmd)
    {
        case JOY_IAP_SUB_CMD_START:
            s_fw_total_size = ((uint32_t)data[6] << 24) | ((uint32_t)data[7] << 16) |
                              ((uint32_t)data[8] << 8)  | ((uint32_t)data[9]);

            if (s_fw_total_size == 0 || s_fw_total_size > JOYSTICK_MAX_FW_SIZE) {
                ack_payload[1] = IAP_ERR_SIZE; // 0x01: 大小非法
            } else if (g_is_joystick_iap_active) {
                ack_payload[1] = 0x02;         // 0x02: 设备正忙
            } else {
                s_fw_recv_bytes = 0;
                m_joystick_ctrl = 0;
                g_motor_owner   = 2;
                motor_stop(TMC2660_M1);
                motor_stop(TMC2660_M2);
                motor_stop(TMC2660_M3);
            }
            protocol_send(data[2], tpc, data[4], ack_payload, 2);
            tcp_output(tpc);
            break;

        case JOY_IAP_SUB_CMD_DATA: {
            uint32_t offset = ((uint32_t)data[6] << 24) | ((uint32_t)data[7] << 16) |
                              ((uint32_t)data[8] << 8)  | ((uint32_t)data[9]);
            uint16_t chunk_len = data[3] - 5;

            if (offset + chunk_len <= JOYSTICK_MAX_FW_SIZE) {
                memcpy(&s_fw_buf[offset], &data[10], chunk_len);
                s_fw_recv_bytes += chunk_len;
            }
            if (((offset + chunk_len) % 1024 == 0) || ((offset + chunk_len) >= s_fw_total_size)) {
                protocol_send(data[2], tpc, data[4], ack_payload, 2);
                tcp_output(tpc);
            }
            break;
        }

        case JOY_IAP_SUB_CMD_FINISH:
            if (s_fw_recv_bytes != s_fw_total_size || s_fw_total_size == 0) {
                ack_payload[1] = IAP_ERR_INCOMPLETE; // 0x03: 数据不完整
                protocol_send(data[2], tpc, data[4], ack_payload, 2);
                tcp_output(tpc);
            } else {
                // 先回 FINISH 确认包
                protocol_send(data[2], tpc, data[4], ack_payload, 2);
                tcp_output(tpc);

                Xil_DCacheFlushRange((UINTPTR)DDR_JOYSTICK_IAP_BUFFER, s_fw_total_size);

                // 启动串口 YModem 烧录并开启监控上报
                if (joystick_iap_start(s_fw_buf, s_fw_total_size, "joystick_app.bin") == 0) {
                    s_monitoring = 1;
                    report_to_host(0x01, IAP_ERR_NONE, 0);
                }
            }
            break;
    }
}

// 2. 在主循环 while(1) 中轮询，实时同步进度与错误码给上位机
void joystick_upgrade_process(void)
{
    static uint8_t last_progress = 0xFF;

    if (!s_monitoring) return;

    iap_state_t st   = joystick_iap_get_state();
    uint8_t     prog = joystick_iap_get_progress();

    if (st == IAP_STATE_SUCCESS) {
        report_to_host(0x02, IAP_ERR_NONE, 100); // 0x02: 升级成功
        s_monitoring    = 0;
        last_progress   = 0xFF;
        m_joystick_ctrl = 1; // 恢复摇杆日常控制
        g_motor_owner   = 0;
    } else if (st == IAP_STATE_FAILED) {
        report_to_host(0x03, joystick_iap_get_err_code(), prog); // 0x03: 失败 + 具体错误码
        s_monitoring    = 0;
        last_progress   = 0xFF;
        m_joystick_ctrl = 1; // 恢复摇杆日常控制
        g_motor_owner   = 0;
    } else if (prog != last_progress) {
        last_progress = prog;
        report_to_host(0x01, IAP_ERR_NONE, prog); // 0x01: 烧录中 + 当前百分比
    }
}
