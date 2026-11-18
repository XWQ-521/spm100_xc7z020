/*
 * hal_tcp.c
 *
 *  Created on: 2025年10月26日
 *      Author: dulc
 */

#include "hal_tcp.h"
#include "com_func.h"


// DDR基地址，标记为volatile表示可能被外部修改
volatile uint8_t *ddr_base = (uint8_t *)DDR_BASE_ADDR;

// 接收队列，32字节对齐以提高访问效率
RecvQueue recv_queue __attribute__((aligned(32)));

// 全局TCP连接控制块指针
struct tcp_pcb *g_current_tpcb = NULL;

// 全局数据包处理回调函数指针，初始化为NULL
static tcp_packet_handler_t g_packet_handler = NULL;

// 前向声明回调函数
err_t accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err);
err_t recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

/**
 * @brief 注册数据包处理回调函数
 * 
 * @param handler 回调函数指针
 */
void register_packet_handler(tcp_packet_handler_t handler)
{
    g_packet_handler = handler;
}

/**
 * @brief 初始化接收队列
 * 
 * @param queue 指向接收队列结构体的指针
 */
void init_recv_queue(RecvQueue *queue)
{
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

/**
 * @brief 启动TCP应用服务
 * 
 * @return int 成功返回0，失败返回负数错误码
 */
int start_application()
{
    struct tcp_pcb *pcb;
    err_t err;
    unsigned port = 6001;

    // 初始化接收队列
    init_recv_queue(&recv_queue);

    // 清理缓存并设置DDR内存属性为非缓存可共享
    Xil_DCacheFlushRange((UINTPTR)&recv_queue, sizeof(RecvQueue));
    Xil_SetTlbAttributes(DDR_BASE_ADDR, NORM_NONCACHE | SHAREABLE);
    Xil_DCacheFlushRange((UINTPTR)ddr_base, DDR_BUF_SIZE);

    // 创建新的TCP控制块
    pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        xil_printf("Error creating PCB. Out of Memory\n\r");
        return -1;
    }

    // 绑定端口
    err = tcp_bind(pcb, IP_ANY_TYPE, port);
    if (err != ERR_OK) {
        xil_printf("Unable to bind to port %d: err = %d\n\r", port, err);
        mem_free(pcb);
        return -2;
    }

    // 设置连接参数并开始监听
    tcp_arg(pcb, NULL);
    pcb = tcp_listen(pcb);
    if (!pcb) {
        xil_printf("Out of memory while tcp_listen\n\r");
        return -3;
    }

    // 注册连接接受回调函数
    tcp_accept(pcb, accept_callback);
    xil_printf("TCP server started on port %d\r\n", port);

    return 0;
}

/**
 * @brief TCP连接接受回调函数
 * 
 * @param arg 用户自定义参数
 * @param newpcb 新建立的连接控制块
 * @param err 错误代码
 * @return err_t 错误状态
 */
err_t accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    // 分配连接状态结构体内存
    conn_state_t *state = (conn_state_t *)mem_malloc(sizeof(conn_state_t));
    if (!state) {
        xil_printf("Failed to allocate connection state\r\n");
        tcp_close(newpcb);
        return ERR_MEM;
    }

    // 保存当前连接控制块
    g_current_tpcb = newpcb;

    // 初始化连接状态
    memset(state, 0, sizeof(conn_state_t));
    state->recv_queue = &recv_queue;
    state->ddr_ptr = (uint8_t *)ddr_base;

    // 设置连接参数和数据接收回调函数
    tcp_arg(newpcb, state);
    tcp_recv(newpcb, recv_callback);

    return ERR_OK;
}

/**
 * @brief TCP数据接收回调函数
 * 
 * @param arg 用户自定义参数（连接状态）
 * @param tpcb 当前连接控制块
 * @param p 接收到的数据缓冲区
 * @param err 错误代码
 * @return err_t 错误状态
 */
err_t recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    conn_state_t *state = (conn_state_t *)arg;
    
    // 如果没有数据，关闭连接并清理资源
    if (!p) {
        if (state) mem_free(state);
        tcp_close(tpcb);
        tcp_recv(tpcb, NULL);
        return ERR_OK;
    }

    // 整理数据缓冲区
    p = pbuf_coalesce(p, PBUF_TRANSPORT);
    uint8_t *data = (uint8_t *)p->payload;
    uint32_t len = p->tot_len;
    uint32_t offset = 0;

    // 解析数据中的数据包帧
    while (offset < len) {
        // 查找帧头标识 0xAA 0x55
        while (offset + 2 <= len && !(data[offset] == 0xAA && data[offset + 1] == 0x55))
            offset++;
        
        // 检查是否有足够的数据读取长度字段
        if (offset + 4 >= len) break;

        // 获取数据长度并计算总帧长度
        uint8_t data_len = data[offset + 3];
        uint32_t total_len = data_len + 8; // AA 55 Addr Len + Data + CS + BC

        // 检查剩余数据是否足够组成完整帧
        if (offset + total_len > len) break;

        // 调用注册的数据包处理回调函数
        if (g_packet_handler != NULL) {
            g_packet_handler(tpcb, &data[offset], total_len);
        } else {
            xil_printf("No packet handler registered!\r\n");
        }

        // 移动到下一帧
        offset += total_len;
    }

    // 通知TCP协议栈已处理完数据
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

