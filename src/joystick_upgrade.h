#ifndef SRC_JOYSTICK_UPGRADE_H_
#define SRC_JOYSTICK_UPGRADE_H_

#include <stdint.h>
#include "lwip/tcp.h"

#define JOYSTICK_MAX_FW_SIZE        (44 * 1024)
#define DDR_JOYSTICK_IAP_BUFFER     0x31000000U

// TCP 子命令定义
#define JOY_IAP_SUB_CMD_START       0x01
#define JOY_IAP_SUB_CMD_DATA        0x02
#define JOY_IAP_SUB_CMD_FINISH      0x03
#define JOY_IAP_SUB_CMD_REPORT      0x04  // PS主动上报烧录进度与错误码

void Joystick_Upgrade_Init(void);
void handle_joystick_iap_cmd(struct tcp_pcb *tpc, uint8_t *data, uint32_t raw_tcp_len);
void joystick_upgrade_process(void);

#endif /* SRC_JOYSTICK_UPGRADE_H_ */
