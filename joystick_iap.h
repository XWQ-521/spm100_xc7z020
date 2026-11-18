#ifndef JOYSTICK_IAP_H
#define JOYSTICK_IAP_H

#include <stdint.h>
#include <stdbool.h>

#define JOYSTICK_FLASH_APP_MAX_SIZE   (44 * 1024)   // STM32F030C8T6 APP 分区最大 44KB

typedef enum {
    IAP_STATE_IDLE = 0,
    IAP_STATE_SEND_RESET_CMD,     // 发送进入 Bootloader 复位指令 (JSON)
    IAP_STATE_WAIT_RESET_STABLE,  // 等待复位完成 (600ms)
    IAP_STATE_SEND_PACKET_0,      // 发送包0 (文件名 + 大小)
    IAP_STATE_WAIT_ACK_AND_C,     // 等待从机回复 ACK + 'C'
    IAP_STATE_FLASH_ERASE_DELAY,  // 等待 Flash 44KB 擦除完成 (1000ms)
    IAP_STATE_SEND_DATA_PACKET,   // 发送 1024B 固件分包 (切片为 64B HID 报文)
    IAP_STATE_WAIT_DATA_ACK,      // 等待分包 ACK
    IAP_STATE_PACKET_DELAY,       // 包间延时保护 (200ms)
    IAP_STATE_SEND_EOT,           // 发送 EOT
    IAP_STATE_WAIT_EOT_ACK,       // 等待 EOT 确认
    IAP_STATE_SEND_END_PACKET,    // 发送空包0结束升级
    IAP_STATE_WAIT_END_ACK,       // 等待最终 ACK
    IAP_STATE_SUCCESS,            // 升级完成
    IAP_STATE_FAILED              // 升级失败
} iap_state_t;
#define IAP_ERR_NONE            0x00
#define IAP_ERR_SIZE            0x01
#define IAP_ERR_INCOMPLETE      0x03
#define IAP_ERR_PKT0_TIMEOUT    0x04  // 未进入Bootloader / Packet 0超时
#define IAP_ERR_DATA_NAK        0x05  // 数据包连续NAK校验失败
#define IAP_ERR_DATA_TIMEOUT    0x06  // 数据包ACK超时
#define IAP_ERR_REMOTE_CAN      0x07  // 摇杆主动取消(CAN)
#define IAP_ERR_EOT_TIMEOUT     0x08  // EOT确认超时
#define IAP_ERR_END_TIMEOUT     0x09  // 结束包确认超时

// 全局 IAP 门禁激活标志 (1: 拦截摇杆日常解析，0: 恢复日常推杆)
extern volatile uint8_t g_is_joystick_iap_active;

// 对外业务接口
int  joystick_iap_start(const uint8_t *fw_bin, uint32_t fw_size, const char *file_name);
void joystick_iap_poll(void);
void joystick_iap_feed_rx_byte(uint8_t byte);
uint8_t joystick_iap_get_progress(void);
iap_state_t joystick_iap_get_state(void);

// Shell 命令行交互注册
void joystick_iap_cmd_register(void);
uint8_t joystick_iap_get_err_code(void);
#endif /* JOYSTICK_IAP_H */
