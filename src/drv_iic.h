/*
 * drv_iic.h
 *
 *  Created on: 2025Äê11ÔÂ15ÈÕ
 *      Author: dulc
 */

#ifndef SRC_DRV_IIC_H_
#define SRC_DRV_IIC_H_
#include "xiicps.h"
#include "COMMON.h"



int IIC_Init(XIicPs *InstPtr, uint16_t DeviceId, uint32_t FsclHz);

void IIC_Write_Reg(XIicPs *InstPtr,uint8_t SlaveID, uint8_t RegBIT, uint16_t reg, uint8_t dat);

uint8_t IIC_Read_Reg(XIicPs *InstPtr,uint8_t SlaveID, uint8_t RegBIT, uint16_t reg);

int IIC_SeqWrite_Reg(XIicPs *InstPtr, uint8_t SlaveID, uint8_t RegBIT,
                        uint16_t reg, uint8_t *send_dat, uint32_t send_len);

void IIC_SeqRead_Reg(XIicPs *InstPtr,uint8_t SlaveID, uint8_t RegBIT,
		uint16_t reg,uint8_t *recv_data, uint32_t recv_len);

#endif /* SRC_DRV_IIC_H_ */
