/*
 * drv_flash.h
 *
 *  Created on: 2025年12月6日
 *      Author: dulc
 */

#ifndef SRC_DRV_FLASH_H_
#define SRC_DRV_FLASH_H_
#include "stdint.h"
#include "xparameters.h"
#include "xqspips.h"


// QSPI 设备 ID（根据你的 system_wrapper.h 确认）
#define QSPI_DEVICE_ID      XPAR_XQSPIPS_0_DEVICE_ID

// 用户参数存储地址（16MB Flash 的 15MB 处）
#define APP_FLASH_ADDR        0x00800000
#define USER_PARAM_ADDR     0x01F00000  // 必须按扇区对齐（64KB）

// Flash 命令定义（N25Q128A）
#define WRITE_ENABLE_CMD    0x06
#define WRITE_DISABLE_CMD   0x04
#define READ_STATUS_CMD     0x05
#define READ_CMD            0x03
#define PAGE_PROG_CMD       0x02
#define SECTOR_ERASE_CMD    0xD8   // 64KB 扇区擦除
#define READ_FLAG_STATUS    0x70


// 读取 Flash 状态寄存器（检查 WIP 位）
uint8_t FlashReadStatus(XQspiPs *QspiPtr) ;
// 等待 Flash 空闲（WIP=0）
void FlashWaitReady(XQspiPs *QspiPtr);
// 写使能
void FlashWriteEnable(XQspiPs *QspiPtr);
// 擦除 64KB 扇区
int FlashSectorErase(XQspiPs *QspiPtr, uint32_t Address) ;
// 页编程（最多 256 字节）
int FlashPageProgram(XQspiPs *QspiPtr, uint32_t Address, uint8_t *Data, uint32_t Length);
// 从 Flash 读取数据
int FlashRead(XQspiPs *QspiPtr, uint32_t Address, uint8_t *Buffer, uint32_t Length);

int Flash_init( void );

XQspiPs * FlashGetQspiInstance( void );

#endif /* SRC_DRV_FLASH_H_ */
