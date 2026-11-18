/*
 * drv_flash.c
 *
 *  Created on: 2025年12月6日
 *      Author: dulc
 */

#include "drv_flash.h"
#include "xil_printf.h"
#include "sleep.h"



XQspiPs QspiInstance;



// 读取 Flash 状态寄存器（检查 WIP 位）
u8 FlashReadStatus(XQspiPs *QspiPtr) {
    u8 status_cmd = READ_STATUS_CMD;
    u8 status_reg = 0;
    XQspiPs_PolledTransfer(QspiPtr, &status_cmd, &status_reg, 2);
    return status_reg;
}

//// 等待 Flash 空闲（WIP=0）
//void FlashWaitReady(XQspiPs *QspiPtr) {
//    while ((FlashReadStatus(QspiPtr) & 0x01) == 0x01) {
//        usleep(1000); // 等待 1ms
//    }
//}



void FlashWaitReady(XQspiPs *QspiPtr) {
    u8 status;
    do {
        u8 cmd = 0x05; // Read Status Register-1
        XQspiPs_PolledTransfer(QspiPtr, &cmd, &status, 1);
    } while (status & 0x01); // WIP bit = bit0
}


// 写使能
void FlashWriteEnable(XQspiPs *QspiPtr) {
    u8 write_en_cmd = WRITE_ENABLE_CMD;
    XQspiPs_PolledTransfer(QspiPtr, &write_en_cmd, NULL, 1);
}

// 擦除 64KB 扇区
int FlashSectorErase(XQspiPs *QspiPtr, u32 Address) {
    u8 erase_cmd[4] = {SECTOR_ERASE_CMD,
                       (u8)((Address >> 16) & 0xFF),
                       (u8)((Address >> 8)  & 0xFF),
                       (u8)( Address        & 0xFF)};
    FlashWriteEnable(QspiPtr);
    int status = XQspiPs_PolledTransfer(QspiPtr, erase_cmd, NULL, 4);
    FlashWaitReady(QspiPtr);
    return status;
}

// 页编程（最多 256 字节）
int FlashPageProgram(XQspiPs *QspiPtr, u32 Address, u8 *Data, u32 Length) {
    if (Length > 256) return XST_FAILURE;

    u8 write_buf[260]; // 1字节命令 + 3字节地址 + 数据
    write_buf[0] = PAGE_PROG_CMD;
    write_buf[1] = (u8)((Address >> 16) & 0xFF);
    write_buf[2] = (u8)((Address >> 8)  & 0xFF);
    write_buf[3] = (u8)( Address        & 0xFF);
    memcpy(&write_buf[4], Data, Length);

    FlashWriteEnable(QspiPtr);
    int status = XQspiPs_PolledTransfer(QspiPtr, write_buf, NULL, 4 + Length);
    FlashWaitReady(QspiPtr);
    return status;
}

// 从 Flash 读取数据
int FlashRead(XQspiPs *QspiPtr, u32 Address, u8 *Buffer, u32 Length) {
    u8 read_cmd[4] = {READ_CMD,
                      (u8)((Address >> 16) & 0xFF),
                      (u8)((Address >> 8)  & 0xFF),
                      (u8)( Address        & 0xFF)};
    uint8_t read_buf[512];
    int ret;

    ret = XQspiPs_PolledTransfer(QspiPtr, read_cmd, read_buf, 4 + Length);

    memcpy( Buffer, read_buf+4, Length );

    return ret;
}



// 复位 Winbond Flash（退出 Quad 模式）
void FlashReset(XQspiPs *QspiPtr) {
    u8 cmd;

    // Reset Enable
    cmd = 0x66;
    XQspiPs_PolledTransfer(QspiPtr, &cmd, NULL, 1);

    // Reset
    cmd = 0x99;
    XQspiPs_PolledTransfer(QspiPtr, &cmd, NULL, 1);

    usleep(100); // 等待 100us（>30us 即可）
}


// 手动禁用 Linear QSPI 模式（适用于所有 Xilinx SDK 版本）
//#define XQSPIPS_LQSPI_CR_OFFSET    0x000000A0  // Linear QSPI Config Register
#define XQSPIPS_LQSPI_CR_EN_MASK   0x00000001  // LQ Mode Enable bit

void DisableLqspiMode(XQspiPs *InstancePtr) {
    u32 RegValue;
    RegValue = XQspiPs_ReadReg(InstancePtr->Config.BaseAddress, XQSPIPS_LQSPI_CR_OFFSET);
    RegValue &= ~XQSPIPS_LQSPI_CR_EN_MASK; // Clear EN bit
    XQspiPs_WriteReg(InstancePtr->Config.BaseAddress, XQSPIPS_LQSPI_CR_OFFSET, RegValue);
}


// 读取状态寄存器1
u8 FlashReadStatusReg1(XQspiPs *QspiPtr) {
    u8 cmd = 0x05; // Read Status Register-1
    u8 status;
    XQspiPs_PolledTransfer(QspiPtr, &cmd, &status, 2); // 发送1字节，接收1字节
    return status;
}

// 写状态寄存器1（需先 WREN）
void FlashWriteStatusReg1(XQspiPs *QspiPtr, u8 value) {
    FlashWriteEnable(QspiPtr);
    u8 cmd[2] = {0x01, value}; // Write Status Register-1
    XQspiPs_PolledTransfer(QspiPtr, cmd, NULL, 2);
    FlashWaitReady(QspiPtr);
}


// 清除 LQSPI 配置中的 Dummy Cycles（关键！）
void ClearLqspiDummyCycles(XQspiPs *InstancePtr) {
    u32 RegVal;
    // 读取 LQSPI 配置寄存器 (offset 0xA0)
    RegVal = XQspiPs_ReadReg(InstancePtr->Config.BaseAddress, 0xA0);
    // 清除 Dummy Cycles 位 (bit 8~13, 共6位)
    RegVal &= ~0x3F00; // 0x3F00 = 0b0011111100000000
    // 写回
    XQspiPs_WriteReg(InstancePtr->Config.BaseAddress, 0xA0, RegVal);
}


int Flash_init( void )
{
    int Status;
    XQspiPs_Config *QspiConfig;

    // 初始化 QSPI
    QspiConfig = XQspiPs_LookupConfig(QSPI_DEVICE_ID);
    if (QspiConfig == NULL) {
        xil_printf("QSPI config lookup failed.\r\n");
        return XST_FAILURE;
    }

    Status = XQspiPs_CfgInitialize(&QspiInstance, QspiConfig, QspiConfig->BaseAddress);
    if (Status != XST_SUCCESS) {
        xil_printf("QSPI init failed.\r\n");
        return XST_FAILURE;
    }

    // 显式关闭 LQSPI
    DisableLqspiMode(&QspiInstance);

    ClearLqspiDummyCycles(&QspiInstance);

    // 设置标准 SPI 模式
     XQspiPs_SetOptions(&QspiInstance,
         XQSPIPS_FORCE_SSELECT_OPTION |
         XQSPIPS_MANUAL_START_OPTION);

    XQspiPs_SetClkPrescaler(&QspiInstance, XQSPIPS_CLK_PRESCALE_128); // ~100MHz/8=12.5MHz

    FlashReset(&QspiInstance);

    // 解除写保护（清除 BP2-BP0）
    u8 status1 = FlashReadStatusReg1(&QspiInstance);
    if ((status1 & 0x1C) != 0) { // 检查 BP2-BP0 (bit4~bit2)
        xil_printf("Clearing Block Protect bits (SR1=0x%02X)...\r\n", status1);
        FlashWriteStatusReg1(&QspiInstance, status1 & ~0x1C); // 清除 BP 位
    }

    xil_printf("QSPI initialized.\r\n");

    return XST_SUCCESS;
}


XQspiPs * FlashGetQspiInstance( void )
{
	return &QspiInstance;
}

