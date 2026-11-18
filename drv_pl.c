/*
 * drv_pl.c
 *
 *  Created on: 2025年10月24日
 *      Author: dulc
 */

#include "drv_pl.h"
#include "xil_cache.h"


// 地址检查结果枚举 (Address Check Result Enumeration)
typedef enum {
    ADDR_ERROR   = 0,                       // 地址无效
    ADDR_CORRECT,                           // 地址有效
} addr_check_e;

/**
 * @brief 检查写入地址是否合法
 * 
 * @param addr 待检查的地址
 * @return addr_check_e 返回地址合法性结果
 */
addr_check_e pl_write_addr_check(uint32_t addr)
{
    if ((addr >= WRITE_ADDR_START1) &&
        (addr <= WRITE_ADDR_END1)) {
        return ADDR_CORRECT;
    } else {
        return ADDR_ERROR;
    }
}

/**
 * @brief 检查读取地址是否合法
 * 
 * @param addr 待检查的地址
 * @return addr_check_e 返回地址合法性结果
 */
addr_check_e pl_read_addr_check(uint32_t addr)
{
    if (((addr >= READ_ADDR_START1) && (addr <= READ_ADDR_END1)) ||
        ((addr >= READ_ADDR_START2) && (addr <= READ_ADDR_END2))) {
        return ADDR_CORRECT;
    } else {
        return ADDR_ERROR;
    }
}

/**
 * @brief 向指定地址写入一个字节的数据
 * 
 * @param addr 目标地址
 * @param data 要写入的数据
 */
void pl_write_u8(uint32_t addr, uint8_t data)
{
    if (!pl_write_addr_check(addr))
        return;

    volatile uint8_t* write_addr = (volatile uint8_t*)(addr);
    *write_addr = data;
    __sync_synchronize();  // 内存屏障，确保操作顺序
    // 显式刷新缓存到物理内存
    Xil_DCacheFlushRange((UINTPTR)write_addr, sizeof(data));
}

/**
 * @brief 向指定地址写入两个字节的数据
 * 
 * @param addr 目标地址
 * @param data 要写入的数据
 */
void pl_write_u16(uint32_t addr, uint16_t data)
{
    if (!pl_write_addr_check(addr) || (addr & 0x1) )
        return;

    volatile uint16_t* write_addr = (volatile uint16_t*)(addr);
    *write_addr = data;
    __sync_synchronize();
    // 显式刷新缓存到物理内存
    Xil_DCacheFlushRange((UINTPTR)write_addr, sizeof(data));
}

/**
 * @brief 向指定地址写入四个字节的数据
 * 
 * @param addr 目标地址
 * @param data 要写入的数据
 */
void pl_write_u32(uint32_t addr, uint32_t data)
{
    if (!pl_write_addr_check(addr) || (addr & 0x3) )
        return;

    volatile uint32_t* write_addr = (volatile uint32_t*)(addr);
    *write_addr = data;
//    printf("addr1: %d,   addr2: %d,  addr3: %d,   addr:%d \r\n",
//			((uint8_t*)write_addr)[0],
//			((uint8_t*)write_addr)[1],
//			((uint8_t*)write_addr)[2],
//			((uint8_t*)write_addr)[3] );
    __sync_synchronize();
    // 显式刷新缓存到物理内存
    Xil_DCacheFlushRange((UINTPTR)write_addr, sizeof(data));
}

/**
 * @brief 从指定地址读取一个字节的数据
 * 
 * @param addr 数据源地址
 * @return uint8_t 读取到的数据或错误码
 */
uint8_t pl_read_u8(uint32_t addr)
{
    if (!pl_read_addr_check(addr))
        return READ_ERROR_U8;

    __sync_synchronize();
    uint8_t *data_ptr = (uint8_t*)addr;
    return *data_ptr;
}

/**
 * @brief 从指定地址读取两个字节的数据
 * 
 * @param addr 数据源地址
 * @return uint16_t 读取到的数据或错误码
 */
uint16_t pl_read_u16(uint32_t addr)
{
    if ( !pl_read_addr_check(addr) || (addr & 0x1) )
        return READ_ERROR_U16;

    __sync_synchronize();
    uint16_t *data_ptr = (uint16_t*)addr;
    return *data_ptr;
}

/**
 * @brief 从指定地址读取四个字节的数据
 * 
 * @param addr 数据源地址
 * @return uint32_t 读取到的数据或错误码
 */
uint32_t pl_read_u32(uint32_t addr)
{
    if ( !pl_read_addr_check(addr) || (addr & 0x3) )
        return READ_ERROR_U32;

    __sync_synchronize();
    uint32_t *data_ptr = (uint32_t*)addr;
    return *data_ptr;
}


/**
 * @brief 从指定地址读取多个16位数据（半字）
 *
 * @param addr    数据源起始地址（必须 2 字节对齐）
 * @param buf     目标缓冲区
 * @param count   要读取的 uint16_t 数量
 * @return int    0 成功，-1 失败
 */
int pl_read_u16_array(uint32_t addr, uint16_t *buf, uint16_t count)
{
    // 检查地址是否 2 字节对齐
    if (!pl_read_addr_check(addr) || (!buf) || (count == 0) || (addr & 0x1) )
        return -1;

    __sync_synchronize();
    uint16_t *src = (uint16_t*)addr;
    for (size_t i = 0; i < count; i++) {
        buf[i] = src[i];
    }
    return 0;
}



/**
 * @brief 从指定地址读取多个16位数据（半字）
 *
 * @param addr    数据源起始地址（必须 2 字节对齐）
 * @param buf     目标缓冲区
 * @param count   要读取的 uint16_t 数量
 * @return int    0 成功，-1 失败
 */
int pl_read_u32_array(uint32_t addr, uint32_t *buf, uint16_t count)
{
    // 检查地址是否 2 字节对齐
    if (!pl_read_addr_check(addr) || (!buf) || (count == 0) || (addr & 0x3) )
        return -1;

    __sync_synchronize();
    uint32_t *src = (uint32_t*)addr;
    for (size_t i = 0; i < count; i++) {
        buf[i] = src[i];
    }
    return 0;
}
