/*
 * drv_iic.c
 *
 *  Created on: 2025年11月15日
 *      Author: dulc
 */
#include "drv_iic.h"


#define EEPROM_PAGE_SIZE         128     // 文档第1页：页写缓冲区为128字节
#define EEPROM_WRITE_DELAY_MS    5       // 文档表1-2：写周期时间最大5ms







/************************************************************************************************************
**  @brief	初始化IIC设备
**  @param	InstPtr IIC设备实例化对象
**  @param	DeviceId	设备ID
**  @param	FsclHz		IIC时钟频率
**	Sample:	PS_IIC_Init(&IIC0,XPAR_XIICPS_0_DEVICE_ID,100000);	//初始化设备IIC0，时钟频率为100kHz
************************************************************************************************************/
int IIC_Init(XIicPs *InstPtr, uint16_t DeviceId, uint32_t FsclHz)
{
    XIicPs_Config *Config;
    int Status;

    // 1. 获取设备配置
    Config = XIicPs_LookupConfig(DeviceId);
    if (Config == NULL)
    {
    	xil_printf("错误: 找不到I2C设备配置 (DeviceId=0x%X)\r\n", DeviceId);
        return XST_FAILURE; // 设备ID无效
    }

    // 2. 初始化控制器
    Status = XIicPs_CfgInitialize(InstPtr, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS)
    {
    	xil_printf("错误: I2C控制器初始化失败 (Status=%d)\r\n", Status);
        return Status; // 初始化失败
    }

    // 3. 自检
    Status = XIicPs_SelfTest(InstPtr);
    if (Status != XST_SUCCESS)
    {
    	xil_printf("警告: I2C硬件自检失败，但继续操作 (Status=%d)\r\n", Status);
        //return Status; // 硬件自检失败
    }

    // 4. 设置时钟频率
    Status = XIicPs_SetSClk(InstPtr, FsclHz);
    if (Status != XST_SUCCESS)
    {
    	xil_printf("错误: I2C时钟设置失败 (Status=%d)\r\n", Status);
        return Status; // 时钟设置失败
    }

    xil_printf("I2C初始化成功，时钟频率: %d Hz\r\n", FsclHz);
    return XST_SUCCESS; // 初始化成功
}


/************************************************************************************************************
**  @brief	写入一个字节数据到I2C设备指定寄存器地址
**  @param	SlaveID	从机地址(7位地址)
**  @param	RegBIT	从机寄存器位数：REG8（8位）REG16（16位）
**  @param	reg		从机寄存器地址
**  @param	Data	要写入的数据
**	Sample:	PS_IIC_Write_Reg(&IIC0,0x12,REG8, 0x34,6);	//设备IIC0写入数据6到0x34地址，从机地址0x12
************************************************************************************************************/
void IIC_Write_Reg(XIicPs *InstPtr,uint8_t SlaveID, uint8_t RegBIT, uint16_t reg, uint8_t dat)
{
	uint8_t WR_Buffer[3];
	if(RegBIT == REG8)
	{
		WR_Buffer[0] = reg;
		WR_Buffer[1] = dat;

		//传输数据
		XIicPs_MasterSendPolled(InstPtr, WR_Buffer,2, SlaveID);
	}
	else if(RegBIT == REG16)
	{
		WR_Buffer[0] = reg>>8;		//取高八位
		WR_Buffer[1] = reg&0x00FF;	//取低八位
		WR_Buffer[2] = dat;

	    uint16_t page_boundary = (reg / EEPROM_PAGE_SIZE + 1) * EEPROM_PAGE_SIZE;
		if (reg + 3 > page_boundary)
		{
			xil_printf("警告: 写入操作可能跨越页边界，地址: 0x%04X\r\n", reg);
		}

		//传输数据
		XIicPs_MasterSendPolled(InstPtr, WR_Buffer,3, SlaveID);
	}

	//等待数据传输完成
	while (XIicPs_BusIsBusy(InstPtr));
}

/************************************************************************************************************
**  @brief	读取I2C设备指定地址寄存器的数据
**  @param	InstPtr	IIC实例化对象
**  @param	SlaveID	从机地址(7位地址)
**  @param	RegBIT	从机寄存器位数：REG8（8位）REG16（16位）
**  @param	reg		从机寄存器地址
**  @return			要读取的数据
**  Sample:	uint8_t value = PS_IIC_Read_Reg(&IIC0, 0x12,REG8, 0x34);//从机设备地址为0x12，读取从机寄存器地址为0x34的数据
************************************************************************************************************/
uint8_t IIC_Read_Reg(XIicPs *InstPtr,uint8_t SlaveID, uint8_t RegBIT, uint16_t reg)
{
	uint8_t RD_Buffer[1],WR_Buffer[2];
	static int send_count = 0;

	if(RegBIT == REG8)
	{
		WR_Buffer[0] = reg;

		//发送数据
		XIicPs_MasterSendPolled(InstPtr, WR_Buffer,1, SlaveID);
	}
	else if(RegBIT == REG16)
	{
		WR_Buffer[0] = reg>>8;		//取高八位
		WR_Buffer[1] = reg&0x00FF;	//取低八位
		//发送数据
		XIicPs_MasterSendPolled(InstPtr, WR_Buffer,2, SlaveID);
	}
	send_count = 0;
	//等待数据传输完成
	while (XIicPs_BusIsBusy(InstPtr) && (send_count < 500) )
	{
		send_count++;
	}

	XIicPs_MasterRecvPolled(InstPtr, RD_Buffer,1, SlaveID);

	send_count = 0;
	//等待，直到所有的数据被接收
	while (XIicPs_BusIsBusy(InstPtr) && (send_count < 500) )
	{
		send_count++;
	}
	return RD_Buffer[0];
}

/************************************************************************************************************
**  @brief	连续写入多个字节数据到I2C设备指定寄存器地址
**  @param	InstPtr	IIC实例化对象
**  @param	SlaveID	从机地址(7位地址)
**  @param	RegBIT	从机寄存器位数：REG8（8位）REG16（16位）
**  @param	reg		从机寄存器地址
**  @param	send_dat	要写入的数据
**  @param	send_len	要写入的数据长度
**	Sample:	PS_IIC_SeqWrite_Reg(&IIC0,0x12,REG8, 0x34,senddata,8);
			//设备IIC0写入数据senddata[0]~senddata[7]到0x34地址，从机地址0x12
************************************************************************************************************/
int IIC_SeqWrite_Reg(XIicPs *InstPtr, uint8_t SlaveID, uint8_t RegBIT,
                        uint16_t reg, uint8_t *send_dat, uint32_t send_len)
{
	static int send_count = 0;
    uint32_t bytes_written = 0;
    uint16_t current_addr = reg;

    while(bytes_written < send_len)
	{
        // 计算当前页剩余空间（重要：防止跨页写入）
        uint16_t page_offset = current_addr % EEPROM_PAGE_SIZE;
        uint32_t chunk_size = EEPROM_PAGE_SIZE - page_offset;

        if(chunk_size > (send_len - bytes_written))
		{
            chunk_size = send_len - bytes_written;
		}

        uint8_t WR_Buffer[chunk_size + 2];

//        xil_printf("IIC -1 \r\n");
        if(RegBIT == REG8)
		{
            WR_Buffer[0] = current_addr;
            memcpy(WR_Buffer + 1, send_dat + bytes_written, chunk_size);
            XIicPs_MasterSendPolled(InstPtr, WR_Buffer, chunk_size + 1, SlaveID);
        }
		else if(RegBIT == REG16)
		 {
            WR_Buffer[0] = current_addr >> 8;
            WR_Buffer[1] = current_addr & 0x00FF;
            memcpy(WR_Buffer + 2, send_dat + bytes_written, chunk_size);
            XIicPs_MasterSendPolled(InstPtr, WR_Buffer, chunk_size + 2, SlaveID);
        }
//        xil_printf("IIC -2\r\n");

        send_count = 0;
        // 等待传输完成  添加超时时间判断
        while (XIicPs_BusIsBusy(InstPtr) && (send_count < 100) )
        {
        	send_count++;
        }
//        xil_printf("IIC -3 \r\n");

        // 遵守EEPROM写周期时间要求
        usleep(EEPROM_WRITE_DELAY_MS * 1000);

        bytes_written += chunk_size;
        current_addr += chunk_size;
    }
    // 判断是否超时，超时返回-1
    if ( send_count > 100 )
    	return -1;
    else
    	return 0;
}


/************************************************************************************************************
**  @brief	连续读取多个I2C设备指定地址寄存器的数据
**  @param	InstPtr	IIC实例化对象
**  @param	SlaveID	从机地址(7位地址)
**  @param	RegBIT	从机寄存器位数：REG8（8位）REG16（16位）
**  @param	reg		从机寄存器地址
**  @param	recv_data	将读取的数据存放在数组recv_data里
**  @param	recv_len	读取数据的长度
**  Sample:	PS_IIC_SeqRead_Reg(&IIC0, 0x12,REG8, 0x34,recv_data,8);
**  		//从机设备地址为0x12，读取从机寄存器地址为0x34的数据，连读8个，存到recv_data[0]~recv_data[7]中
************************************************************************************************************/
void IIC_SeqRead_Reg(XIicPs *InstPtr,uint8_t SlaveID, uint8_t RegBIT,
		uint16_t reg,uint8_t *recv_data, uint32_t recv_len)
{
	uint8_t WR_Buffer[2];
	static int send_count = 0;

	if(RegBIT == REG8)
	{
		WR_Buffer[0] = reg;

		//发送数据
		XIicPs_MasterSendPolled(InstPtr, WR_Buffer,1, SlaveID);
	}
	else if(RegBIT == REG16)
	{
		WR_Buffer[0] = reg>>8;		//取高八位
		WR_Buffer[1] = reg&0x00FF;	//取低八位
		//发送数据
		XIicPs_MasterSendPolled(InstPtr, WR_Buffer,2, SlaveID);
	}
	send_count = 0;
	//等待数据传输完成
	while (XIicPs_BusIsBusy(InstPtr) && (send_count < 500) )
	{
		send_count++;
	}
	XIicPs_MasterRecvPolled(InstPtr, recv_data,recv_len, SlaveID);

	send_count = 0;
	//等待，直到所有的数据被接收
	while (XIicPs_BusIsBusy(InstPtr) && (send_count < 500) )
	{
		send_count++;
	}
}

