/*
 * hal_tcp.h
 *
 *  Created on: 2025年10月26日
 *      Author: dulc
 */

#ifndef SRC_HAL_HAL_TCP_H_
#define SRC_HAL_HAL_TCP_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "lwip/err.h"
#include "lwip/tcp.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_mmu.h"



#define RECV_QUEUE_SIZE 	10


extern volatile uint8_t *ddr_base;

// DDR相关配置
#define DDR_BASE_ADDR   	0x10000014
#define DDR_BUF_SIZE    	4096   //0x10001000          //DDR_SIZE  0x40000000  // 1GB (十六进制)
#define RECV_QUEUE_SIZE 	10


// 命令解析结构                  ---->       那12字节  一一相对应的标志位  （待修改）
typedef struct {
    uint8_t start_flag; //起始标志位（0XAA）       1个字节   索引0
    uint8_t cmd_type;   //命令类型	   1个字节   索引1
    uint8_t sub_cmd;   //子命令			1个字节   索引2
    uint8_t data_len;	//数据长度		1个字节   索引3
    uint8_t payload[8];  //负载数据		8个字节   索引4 到 11
    //uint16_t checksum;   //校验和     （不知道是否需要）
} CommandFrame; //命令框架



// 全局DDR接收队列
typedef struct {
    CommandFrame frames[RECV_QUEUE_SIZE];  //命令解析结构的数组
    int head; //头
    int tail;  //尾
    int count;  //计数
} RecvQueue;


// 连接状态结构
typedef struct {
    uint8_t recv_buffer[20];  // 接收缓冲区，略大于最大命令长度       -->   固定12字节  （接收数据为  --> 上位机发送的命令）
    int recv_count;           // 当前已接收字节数
    uint8_t *ddr_ptr;         // DDR存储位置指针
    RecvQueue *recv_queue;    // 接收队列指针
} conn_state_t;




// 定义回调函数类型
typedef void (*tcp_packet_handler_t)(struct tcp_pcb *tpcb, uint8_t *data, uint32_t len);





// global 或 static 全局变量
extern struct tcp_pcb *g_current_tpcb;


int start_application();     // 启动应用程序函数声明

void register_packet_handler(tcp_packet_handler_t handler);




#endif /* SRC_HAL_HAL_TCP_H_ */
