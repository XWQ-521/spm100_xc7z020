/*
 * hal_cmos.c
 *
 *  Created on: 2025年10月25日
 *      Author: dulc
 */
#include "hal_cmos.h"
#include "drv_pl.h"
#include "COMMON.h"
/**
 * @brief 读取探测器数据
 * 
 * 从硬件寄存器中读取PSD探测器的数据，
 * 
 * @return detector_t 包含探测器当前状态的数据结构
 */
detector_t get_detector_data(void)
{
    detector_t tmp_det = { 0 };

    pl_read_u16_array(READ_ADDR_PSD, (uint16_t*)&tmp_det, sizeof(detector_t)/sizeof(uint16_t));

    return tmp_det;
}

/**
 * @brief 设置XYZ轴对焦参数
 * 
 * 根据传入的data数组设置X、Y、Z三个方向上的对焦位置。
 * 
 * @param data 指向包含对焦参数的缓冲区指针
 */
void xyz_param_set(uint8_t *data)
{
	uint16_t z_value = 0;
    // 设置X轴对焦位置
    pl_write_u8(WRITE_ADDR_FOCUS_X, data[0]);
    // 设置Y轴对焦位置
    pl_write_u8(WRITE_ADDR_FOCUS_Y, data[1]);

    z_value = 0;
    z_value = ((data[2]*256 + data[3]) & 0x0FFF) | 0xA000;
    // 设置Z轴对焦位置（由两个字节组合成一个16位值）
    pl_write_u16(WRITE_ADDR_FOCUS_Z, z_value);
    usleep(1000);
    z_value = 0;
    z_value = ((data[4]*256 + data[5]) & 0x0FFF) | 0x5000;
    pl_write_u16(WRITE_ADDR_FOCUS_Z, z_value);
}

/**
 * @brief 设置CMOS曝光时间
 * 
 * 将给定的时间值写入CMOS曝光控制寄存器。
 * 
 * @param time 曝光时间值
 */
void cmos_expose_set(uint16_t time)
{
    pl_write_u16(WRITE_ADDR_CMOS_EXPO, time);
}

/**
 * @brief 获取CMOS图像数据
 * 
 * 从CMOS传感器读取图像数据并存储到read_buf指向的缓冲区中。
 * 
 * @param read_buf 指向用于存放读取数据的缓冲区的指针
 * @return int 成功返回读取的字节数，失败返回-1
 */
int cmos_get_z_data(uint32_t* read_buf)
{
    int ret = -1;

    ret = pl_read_u32_array(READ_ADDR_CMOS_DATA, read_buf, 256);

    return ret;
}



int cmos_get_xy_data( uint32_t* read_buf )
{
    int ret = -1;

    ret = pl_read_u32_array(READ_ADDR_XY_DATA, read_buf, 256);

    return ret;
}



