/*
 * motorctrl.c
 *
 *  Created on: 2025年7月22日
 *      Author: jczxy
 */

#include "motorctrl.h"
// 优化添加
#include "xgpiops.h"
#include "xil_cache.h"
#include "hal_timer.h"
#include "stdio.h"
#include "drv_motor.h"
#include "pid.h"
#include "protocol.h"
#include "hal_motor.h"
#include "param.h"
#include "motor_curve.h"
#include "version.h"
#include "joystick.h"
// 电机加减速控制
//#define MOTOR_CTRL_CURVE


#define MOTOR_HOLL_ON           0
#define MOTOR_HOLL_OFF          1

#define MOTOR_STEP_DIRECT_RUN   2000







motor_ctrl_t m_motorctrl[MOTOR_COUNT] = {
		{ 32, 20,  7000,  -6500,     0 },
		{ 32, 20, 19000, -19000,     0 },
		{ 32, 20,  8000,  -2000, -5000 },
		{ 32, 20, 10000, -10000,     0 },
		{ 32, 20, 10000, -10000,     0 }
};


center_t m_center[MOTOR_COUNT] = {
		{ 0, 0, 0, 0,  MOTOR_DIR_BACKWARD ,MOTOR_DIR_FORWARD },
		{ 0, 0, 0, 0,  MOTOR_DIR_BACKWARD ,MOTOR_DIR_FORWARD },
#ifdef THREE_AXIS_V20
		{ 0, 0, 0, 0,  MOTOR_DIR_FORWARD ,MOTOR_DIR_BACKWARD },
#else
		{ 0, 0, 0, 0,  MOTOR_DIR_BACKWARD,MOTOR_DIR_FORWARD  },
#endif
		{ 0, 0, 0, 0,  MOTOR_DIR_BACKWARD,MOTOR_DIR_FORWARD  },
		{ 0, 0, 0, 0,  MOTOR_DIR_BACKWARD,MOTOR_DIR_FORWARD  }
};


// 待机模式管理
motor_standby_t m_motor_standby[MOTOR_COUNT] = {
		{ MOTOR_MODE_STANDBY, 28, 0, 1 },
		{ MOTOR_MODE_STANDBY, 28, 0, 1 },
		{ MOTOR_MODE_STANDBY, 28, 0, 1 },
		{ MOTOR_MODE_STANDBY, 28, 0, 1 },
		{ MOTOR_MODE_STANDBY, 28, 0, 1 }
};

static uint64_t m_last_standby_check_time = 0;





void motor_center_check(MOTOR_ADDR addr_s);
void motor_focus_centering(MOTOR_ADDR addr);
void motor_limit_check( MOTOR_ADDR addr );




// motorctrlsd
void motorctrl_init( void )
{
//	param_init();

	for ( int addr = TMC2660_M1; addr <= TMC2660_M5; addr++ )
	{
		param_update_to_motor( addr, (motor_ctrl_t*)&m_motorctrl[addr-1] );
		xil_printf("M%d: 参数已经初始化，参数为： %d, %d, %d, %d, %d \r\n", addr,
				(int)(m_motorctrl[addr-1].forward_limit),
				(int)(m_motorctrl[addr-1].backward_limit), (int)(m_motorctrl[addr-1].center_step),
				(int)(m_motorctrl[addr-1].div), (int)(m_motorctrl[addr-1].current) );

		// 电机控制初始化
		motor_device_init( addr, m_motorctrl[addr-1].div, m_motorctrl[addr-1].current );
		//motor_device_init( addr, 32, 20 );
		motor_center_check( addr );
	}
}

// motorctrl
// 电机后台处理函数
void motorctrl_process( void )
{
	// 电机归中处理
    for( int i = 0; i < MOTOR_COUNT; i++ ){
        motor_focus_centering( i+1 );
        //xil_printf("center_flag = %d\r\n",m_center[i].center_flag);
        if(m_center[i].center_flag == 1){
        	motor_limit_check( i+1 );
            //xil_printf("center_flag = %d\r\n",m_center[i].center_flag);
        	}
    }
    //xil_printf("motoctrl_prosess\r\n");
    ReadAllSteps();
}


void motorctrl_limit_process( void )
{
	for ( int i = 0; i < MOTOR_COUNT; i++ ){
        //if(m_center[i].center_flag == 1)
        	motor_limit_check( i+1 );
	}
}
/**
 * @brief 电机限位触发与坐标实时同步函数
 * @note  该函数独立于运动控制逻辑，可由定时器或主循环高频调用
 * @param addr 电机地址
 */
#if 0
void motor_limit_sync_coords(MOTOR_ADDR addr)
{
	int idx = addr - 1;
	    uint8_t current_holl = motor_get_limit_center(addr);

	    // 1. 检测限位开关边缘触发（从非限位状态进入限位状态的一瞬间）
	    if (current_holl != m_center[idx].start_holl)
	    {
	        // 2. 如果之前是 0，说明这是“刚碰上”的一瞬间
	        if (m_center[idx].center_flag == 0)
	        {
	            // 执行坐标纠偏：强制同步当前物理位置为预设的绝对参考坐标
	            motor_center_step_init(addr, m_motorctrl[idx].center_step);

	            // 标记已进入限位区，锁定校准，防止在限位块上反复触发
	            m_center[idx].center_flag = 1;

	            xil_printf("M%d: 动态纠偏成功！已消除累计误差。当前坐标: %ld\r\n",
	                        addr, motor_get_cur_step(addr));
	        }
	    }
	    else
	    {
	        // 3. 重要：当电机离开限位区后，自动将标志位恢复为 0
	        // 这样下次电机再次经过此限位点时，可以再次触发纠偏
	        if (m_center[idx].center_flag == 1)
	        {
	            m_center[idx].center_flag = 0;
	            // xil_printf("M%d: 已离开限位区，纠偏功能重新就绪。\r\n", addr);
	        }
	    }
}
#endif
#if 1
/**
 * @brief 优化版双行程动态坐标纠偏函数
 * * 支持：正向、反向单次穿过或撞击限位块时，均能精准触发独立坐标校准。
 * * 解决：消除了限位块物理宽度带来的双向对准误差，并平滑同步了位置闭环。
 */
void motor_limit_sync_coords(MOTOR_ADDR addr)
{
    int idx = addr - 1;
    uint8_t current_holl = motor_get_limit_center(addr);

    extern volatile uint8_t motor_move_flag[];
    extern PIDController posPids[];
    extern float targetAngles[];
    extern float currentSteps[];
    extern struct tcp_pcb *g_current_tpcb;

    static uint8_t last_holl[5] = {0, 0, 0, 0, 0};
    static uint8_t is_first_run[5] = {1, 1, 1, 1, 1};

    if (is_first_run[idx])
    {
        last_holl[idx] = current_holl;
        is_first_run[idx] = 0;
        return;
    }

    // 状态发生跳变（穿过光电边缘）
    if (current_holl != last_holl[idx])
    {
        int32_t old_step = motor_get_cur_step(addr);

        // 1. 光电边缘在当前坐标系下的物理理论坐标为负值 (-1500)
        int32_t expected_coord = -m_motorctrl[idx].center_step;

        // 2. 正确计算真实物理漂移误差
        int32_t delta_error = expected_coord - old_step;

        int8_t move_dir = (targetAngles[idx] >= currentSteps[idx]) ? 1 : -1;
        if (move_dir == 1) {
            xil_printf("M%d: [正转] 亮(0)->灭(1) 沿触发！", addr);
        } else {
            xil_printf("M%d: [反转] 灭(1)->亮(0) 沿触发！", addr);
        }

        // 3. 安全保护门限：真正的机械漂移一般在 500 步以内（~0.5mm）
//        if (abs(delta_error) < 500)
//        {
            // 纠偏重新同步，传入底层的偏移参数依然是正数 center_step (+1500)
            motor_center_step_init(addr, m_motorctrl[idx].center_step);
            xil_printf("纠偏成功！真实偏差: %ld, 新坐标: %ld\r\n", (long)delta_error, (long)motor_get_cur_step(addr));
       // }
//        else
//        {
//            xil_printf("纠偏忽略 (真实偏差过大异常: %ld)\r\n", (long)delta_error);
//        }

        // 4. 电机 2 上报 0x6B
        if (addr == TMC2660_M2)
                {
                    // 确保 TCP 控制块存在且连接处于建立状态 (ESTABLISHED = 4)
                    if (g_current_tpcb != NULL && g_current_tpcb->state == ESTABLISHED)
                    {
                        uint8_t payload[1];
                        payload[0] = current_holl; // 0 或 1 代表当前左右状态

                        // 发送并立即刷新
                        protocol_send(0x0A, g_current_tpcb, 0x6B, payload, 1);
                        err_t out_err = tcp_output(g_current_tpcb);

                        xil_printf("[0x6B SEND] State: %d, tcp_output err: %d\r\n", current_holl, out_err);
                    }
                    else
                    {
                        xil_printf("[0x6B DROP] TCP not connected! pcb: %p\r\n", g_current_tpcb);
                    }
                }

        last_holl[idx] = current_holl;
    }
}
#endif

// motorctrl
void motor_center_check(MOTOR_ADDR addr_s)
{
    // 在头文件或全局定义
	#define MOTOR_BASE_FREQ       10000

    if (!motor_addr_check(addr_s))
        return;

    // 初始化中心控制标志
    m_center[addr_s-1].center_flag = 0;
    m_center[addr_s-1].start_holl = motor_get_limit_center(addr_s);

    if (m_center[addr_s-1].start_holl == MOTOR_HOLL_ON) {
    	 motor_control( addr_s, m_center[addr_s-1].hollon_dir, MOTOR_BASE_FREQ, 0x7FFF );
    } else {
    	 motor_control( addr_s, m_center[addr_s-1].holloff_dir, MOTOR_BASE_FREQ, 0x7FFF );
    }
    xil_printf( "初始化归中：MOTOR_ID: %d!  启动限位： %d  当前步数：step: %d \n\r",
    		addr_s,
			m_center[addr_s-1].start_holl,
			(int)motor_get_cur_step( addr_s ) );
}




// motor_ctrl
/**
 * @brief 聚焦电机回零处理函数
 * 
 * 该函数用于处理指定电机的回零操作。通过检测限位开关状态的变化来判断是否需要执行回零操作，
 * 如果需要回零且尚未执行过，则设置相应的回零参数并触发回零动作。
 * 
 * @param addr 电机地址/编号 (1-5)，对应M1-M5电机
 * @return 无返回值
 */
void motor_focus_centering(MOTOR_ADDR addr)
{
    // 检查当前限位开关状态是否与初始状态不同（表示检测到限位信号变化）
    if(motor_get_limit_center(addr) != m_center[addr-1].start_holl)
    {
        // 检查该电机是否已经执行过回零操作（flag为1表示已执行）
        if(m_center[addr-1].center_flag != 1)
        {
        	motor_center_step_init( addr, m_motorctrl[addr-1].center_step );
//        	xil_printf("=====限位开关状态改变： 初始限位： %d,  当前限位： %d \r\n",  m_center[addr-1].start_holl, motor_get_limit_center(addr) );
        	xil_printf("=============初始化归中步数: 地址： %d, %ld !   step: %ld============ \n\r",
        	        			addr,
        						motor_get_real_encode(addr),
        						motor_get_cur_step( addr ) );
            // 设置回零标志位，表示即将执行回零操作
        	m_center[addr-1].center_flag = 1;

        	motor_stop( addr );
            
            if ( m_motorctrl[addr-1].center_step == 0 )
            {
            	m_center[addr-1].stop_done = 1;
            	motor_stop( addr );
            }else
            // 根据预设的回零步数正负值来设置回零方向和步数
            if(m_motorctrl[addr-1].center_step > 0)
            {
            	m_center[addr-1].stop_check = 1;
                motor_control( addr, MOTOR_DIR_FORWARD, MOTOR_BASE_FREQ, fabsf(m_motorctrl[addr-1].center_step)+15 );

            	xil_printf("正转!  ---%d---  step: %ld \n\r",
                		(int)m_motorctrl[addr-1].center_step,
						motor_get_cur_step( addr ) );
            }
            else
            {
            	m_center[addr-1].stop_check = 1;
            	motor_control( addr, MOTOR_DIR_BACKWARD, MOTOR_BASE_FREQ, fabsf(m_motorctrl[addr-1].center_step)+15 );
            	xil_printf("反转!  ---%d---  step: %ld  timer: %d \n\r",
                		(int)m_motorctrl[addr-1].center_step ,
						motor_get_cur_step( addr ),
						(int)timer_get_1us() );
            }
        }
    }
    else
    {
    	// 额拖电机行程较大，持续给控制，直到检测到限位变化
    	if ( (addr == TMC2660_M4) && (m_center[addr-1].center_flag != 1) )
    	{
    		static uint32_t step = 110;
    		step ++;
    		// 该判断用于电机反转后，限位状态还没有变，等待限位状态改变再进行判断，否则方向走不对
    		if ( step > 100 )
    		{
    			if ( step > 150 )
    			{
    				step = 110;
    			}

				if (m_center[addr-1].start_holl == MOTOR_HOLL_ON) {
//					xil_printf("MOTOR_FORWARD\r\n");
					 motor_control( addr, m_center[addr-1].hollon_dir, MOTOR_BASE_FREQ, 500+step );
				} else {
//					xil_printf("MOTOR_BACKWARD\r\n");
					 motor_control( addr, m_center[addr-1].holloff_dir, MOTOR_BASE_FREQ, 500+step );
				}
    		}

//            xil_printf("启动时限位状态： %d,   当前限位状态: %d\n",  m_center[addr-1].start_holl, motor_get_limit_center(addr) );
    	}
    }

    if (m_center[addr-1].stop_check == 1 )
    {



		if ( fabs(motor_get_cur_step( addr )) <= 50 )
		{
			m_center[addr-1].stop_check = 0;
			m_center[addr-1].stop_done = 1;
			motor_stop( addr );
			xil_printf("M%d: 归中完成！电机停止, timer: %d\r\n",addr, (int)timer_get_1us());
		} else {
			// 额拖电机行程有限，持续给动作
			if ( addr == TMC2660_M4 )
			{
				static uint8_t random_step = 10;
				random_step++;
	            if(m_motorctrl[addr-1].center_step > 0)
	            {
	            	if ( motor_get_cur_step( addr ) > 0 )
	            	{
	        			m_center[addr-1].stop_check = 0;
	        			m_center[addr-1].stop_done = 1;
	        			motor_stop( addr );
	        			xil_printf("M%d: 归中完成！归中超过零点停止，电机停止, timer: %d\r\n",addr, (int)timer_get_1us());
	            	}
	            	else
	            	{
	            		motor_control( addr, MOTOR_DIR_FORWARD, MOTOR_BASE_FREQ, 500+random_step );
	            	}
	            }
	            else
	            {
	            	if ( motor_get_cur_step( addr ) < 0 )
	            	{
	        			m_center[addr-1].stop_check = 0;
	        			m_center[addr-1].stop_done = 1;
	        			motor_stop( addr );
	        			xil_printf("M%d: 归中完成！归中超过零点停止，电机停止, timer: %d\r\n",addr, (int)timer_get_1us());
	            	}
	            	else
	            	{
	            		motor_control( addr, MOTOR_DIR_BACKWARD, MOTOR_BASE_FREQ, 500+random_step );
	            	}
	            }

			}
		}
    }
}




uint8_t motor_center_done_check( void )
{
#define CURRENT_MOTOR_COUNT		4
	int i = 0;
	int count = 0;

	for ( i = 0; i < CURRENT_MOTOR_COUNT; i++ )
	{
		if (m_center[i].stop_done == 1 )
			count++;

	}
	//xil_printf("motor_center_done_check\r\n");
	if (count >= CURRENT_MOTOR_COUNT )
		return 1;
	else
		return 0;
}





// motorctrl
#if 0
void motor_limit_check( MOTOR_ADDR addr )
{
	switch ( m_motor[addr-1].dir )
	{
	case 1:
		if ( (int32_t)motor_get_cur_step(addr) >= (int32_t)m_motorctrl[addr-1].forward_limit-10 ) // 预留10步空间
		{
			xil_printf( "M%d: 达到前限位状态！cur_step: %d,   limit: %d,  time: %lld \n\r", addr,
					(int)motor_get_cur_step(addr),
					(int)m_motorctrl[addr-1].forward_limit,
					timer_get_1us() );
			m_motor[addr-1].dir = 0;
			// 到达目标位置，停止电机并清除移动标志
			PID_Stop( addr );
//			motor_stop(addr);
//			motor_move_flag[addr-1] = 0;   // 清除移动标志
//			is_pid_flag[addr-1] = 0;
			protocol_report_limit( addr, 1, 0 );
			xil_printf("电机停止！---4\r\n");
			PID_Init(&posPids[addr-1],currentSteps[addr-1]);
		}
		break;
	case 2:
		if ( (int32_t)motor_get_cur_step(addr) <= (int32_t)m_motorctrl[addr-1].backward_limit+10 )
		{
			xil_printf( "M%d: 达到后限位状态！cur_step: %d,   limit: %d,  time: %lld \n\r", addr,
					(int)motor_get_cur_step(addr),
					(int)m_motorctrl[addr-1].backward_limit,
					timer_get_1us() );
			m_motor[addr-1].dir = 0;
			// 到达目标位置，停止电机并清除移动标志
			PID_Stop( addr );
//			motor_stop(addr);
//			motor_move_flag[addr-1] = 0;   // 清除移动标志
//			is_pid_flag[addr-1] = 0;
			protocol_report_limit( addr, 0, 1 );
			xil_printf("电机停止！---5\r\n");
			PID_Init(&posPids[addr-1],currentSteps[addr-1]);
		}
		break;
	default:
		break;
	}

}
#endif
void motor_limit_check( MOTOR_ADDR addr )
{
	// 声明我们自己立起的电机2全局标志位
	extern uint8_t g_m2_low_speed_run_flag;

	switch ( m_motor[addr-1].dir )
	{
	case 1:
		if ( (int32_t)motor_get_cur_step(addr) >= (int32_t)m_motorctrl[addr-1].forward_limit-10 ) // 预留10步空间
		{
			xil_printf( "M%d: 达到前限位状态！cur_step: %d,   limit: %d,   time: %lld \n\r", addr,
					(int)motor_get_cur_step(addr),
					(int)m_motorctrl[addr-1].forward_limit,
					timer_get_1us() );
			m_motor[addr-1].dir = 0;

			//【电机2专属前限位急停】：如果是电机2触发了限位
			if (addr == 2)
			{
				g_m2_low_speed_run_flag = 0;   // 1. 瞬间关闭 1ms 定时器的后台续杯
				motor_low_speed_run(2, 0, 0);  // 2. 一键调用专属函数传0，让硬件物理寄存器彻底清零
			}

			// 到达目标位置，停止电机并清除移动标志
			PID_Stop( addr );
//			motor_stop(addr);
//			motor_move_flag[addr-1] = 0;   // 清除移动标志
//			is_pid_flag[addr-1] = 0;
			protocol_report_limit( addr, 1, 0 );
			xil_printf("电机停止！---4\r\n");
			PID_Init(&posPids[addr-1],currentSteps[addr-1]);
		}
		break;
	case 2:
		if ( (int32_t)motor_get_cur_step(addr) <= (int32_t)m_motorctrl[addr-1].backward_limit+10 )
		{
			xil_printf( "M%d: 达到后限位状态！cur_step: %d,   limit: %d,   time: %lld \n\r", addr,
					(int)motor_get_cur_step(addr),
					(int)m_motorctrl[addr-1].backward_limit,
					timer_get_1us() );
			m_motor[addr-1].dir = 0;

			// 【电机2专属后限位急停】：如果是电机2触发了限位
			if (addr == 2)
			{
				g_m2_low_speed_run_flag = 0;   // 1. 瞬间关闭 1ms 定时器的后台续杯
				motor_low_speed_run(2, 0, 0);  // 2. 一键调用专属函数传0，让硬件物理寄存器彻底清零
			}

			// 到达目标位置，停止电机并清除移动标志
			PID_Stop( addr );
//			motor_stop(addr);
//			motor_move_flag[addr-1] = 0;   // 清除移动标志
//			is_pid_flag[addr-1] = 0;
			protocol_report_limit( addr, 0, 1 );
			xil_printf("电机停止！---5\r\n");
			PID_Init(&posPids[addr-1],currentSteps[addr-1]);
		}
		break;
	default:
		break;
	}
}


/**
 * @brief 步数限位检查
 *
 * 根据当前电机位置和运动方向，检查是否超出限位范围
 * 如果超出限位则返回可移动的最大步数，否则返回原步数
 *
 * @param addr 电机地址
 * @param step 请求移动步数
 * @return uint32_t 实际可移动步数
 */
#if 0
uint32_t step_limit_check(MOTOR_ADDR addr, int32_t step)
{
    uint32_t move_steps = 0;

    switch (m_motor[addr-1].dir) {
        case 1:  // 正向运动
            // 如果已经到达前限位
            if ((int32_t)motor_get_cur_step(addr) >= (int32_t)m_motorctrl[addr-1].forward_limit) {
            	xil_printf("cmd达到前限位状态！time: %lld \n\r", timer_get_1us());
                return 0;
            }
            // 如果移动后会超出前限位
            else if ((int32_t)((int32_t)motor_get_cur_step(addr) + (int32_t)step) >= (int32_t)m_motorctrl[addr-1].forward_limit) {
                // 计算到限位点的距离
                move_steps = fabs((int32_t)m_motorctrl[addr-1].forward_limit - (int32_t)motor_get_cur_step(addr));
            } else {
                // 可以正常移动
                move_steps = step;
            }
            break;

        case 2:  // 反向运动
            // 如果已经到达后限位
            if ((int32_t)motor_get_cur_step(addr) <= (int32_t)m_motorctrl[addr-1].backward_limit) {
            	xil_printf("cmd达到后限位状态！time: %lld \n\r", timer_get_1us());
                return 0;
            }
            // 如果移动后会超出后限位
            else if ((int32_t)((int32_t)motor_get_cur_step(addr) - (int32_t)step) <= (int32_t)m_motorctrl[addr-1].backward_limit) {
                // 计算到限位点的距离
                move_steps = fabs((int32_t)m_motorctrl[addr-1].backward_limit - (int32_t)motor_get_cur_step(addr));
            } else {
                // 可以正常移动
                move_steps = step;
            }
            break;

        default:
            move_steps = 0;
            break;
    }

    return move_steps;
}
#endif
#if 1
uint32_t step_limit_check(MOTOR_ADDR addr, int32_t step)
{
    uint32_t move_steps = 0;
    int32_t cur_pos = (int32_t)motor_get_cur_step(addr);
    int32_t f_limit = (int32_t)m_motorctrl[addr-1].forward_limit;
    int32_t b_limit = (int32_t)m_motorctrl[addr-1].backward_limit;

    switch (m_motor[addr-1].dir) {
        case 1:  // 正向运动
            // 如果已经到达或超出了前限位
            if (cur_pos >= f_limit) {
                xil_printf("cmd达到前限位状态！time: %lld \n\r", timer_get_1us());
                return 0;
            }
            // 如果移动后会超出前限位
            else if (cur_pos + step >= f_limit) {
                // 【安全修复】：纯有符号整数减法，绝不用 fabs()
                move_steps = (uint32_t)(f_limit - cur_pos);
            } else {
                // 可以正常移动
                move_steps = (uint32_t)step;
            }
            break;

        case 2:  // 反向运动
            // 如果已经到达或超出了后限位
            if (cur_pos <= b_limit) {
                xil_printf("cmd达到后限位状态！time: %lld \n\r", timer_get_1us());
                return 0;
            }
            // 如果移动后会超出后限位（注意反向是当前坐标减去步数）
            else if (cur_pos - step <= b_limit) {
                // 【安全修复】：用大数减小数，直接得出正数距离，彻底断绝负数和隐式转换！
                // 当前坐标(例如19000) 减去 后限位(例如-19000)，结果就是绝对安全的 38000
                move_steps = (uint32_t)(cur_pos - b_limit);
            } else {
                // 可以正常移动
                move_steps = (uint32_t)step;
            }
            break;

        default:
            move_steps = 0;
            break;
    }

    return move_steps;
}
#endif


uint32_t move_limit_check(MOTOR_ADDR addr, motor_dir_e dir, uint32_t step)
{
    uint32_t move_steps = 0;

    switch (dir) {
        case 1:  // 正向运动
            // 如果已经到达前限位
            if ((int32_t)motor_get_cur_step(addr) >= (int32_t)m_motorctrl[addr-1].forward_limit) {
            	xil_printf("cmd达到前限位状态！time: %lld \n\r", timer_get_1us());
                return 0;
            }
            // 如果移动后会超出前限位
            else if ((int32_t)((int32_t)motor_get_cur_step(addr) + step) >= (int32_t)m_motorctrl[addr-1].forward_limit) {
                // 计算到限位点的距离
                move_steps = fabs((int32_t)m_motorctrl[addr-1].forward_limit - (int32_t)motor_get_cur_step(addr));
            } else {
                // 可以正常移动
                move_steps = step;
            }
            break;

        case 2:  // 反向运动
            // 如果已经到达后限位
            if ((int32_t)motor_get_cur_step(addr) <= (int32_t)m_motorctrl[addr-1].backward_limit) {
            	xil_printf("cmd达到后限位状态！time: %lld \n\r", timer_get_1us());
                return 0;
            }
            // 如果移动后会超出后限位
            else if ((int32_t)((int32_t)motor_get_cur_step(addr) - step) <= (int32_t)m_motorctrl[addr-1].backward_limit) {
                // 计算到限位点的距离
                move_steps = fabs((int32_t)m_motorctrl[addr-1].backward_limit - (int32_t)motor_get_cur_step(addr));
            } else {
                // 可以正常移动
                move_steps = step;
            }
            break;

        default:
            move_steps = 0;
            break;
    }

    return move_steps;
}




/**
 * @brief 坐标限位检查
 *
 * 根据目标坐标位置，计算实际可移动的步数并进行限位检查
 *
 * @param addr 电机地址
 * @param coord 目标坐标位置
 * @return uint32_t 实际可移动步数
 */
uint32_t coord_limit_check(MOTOR_ADDR addr, int32_t coord)
{
    uint32_t move_steps = 0;

    // 根据目标坐标与当前位置的关系确定运动方向
    if ((int32_t)(coord) > (int32_t)motor_get_cur_step(addr)) {
        // 记录运动方向为正向
        m_motor[addr-1].dir = MOTOR_DIR_FORWARD;
    } else {
        // 记录运动方向为反向
        m_motor[addr-1].dir = MOTOR_DIR_BACKWARD;
    }

    switch (m_motor[addr-1].dir) {
        case 1:  // 正向运动
            // 如果已经到达前限位
            if (motor_get_cur_step(addr) >= m_motorctrl[addr-1].forward_limit) {
            	xil_printf("cmd达到前限位状态！time: %lld \n\r", timer_get_1us());
                return 0;
            }
            // 如果目标坐标超出前限位
            else if (coord >= m_motorctrl[addr-1].forward_limit) {
                // 计算到前限位的距离
                move_steps = fabs(m_motorctrl[addr-1].forward_limit - motor_get_cur_step(addr));
            } else {
                // 计算到目标位置的距离
                move_steps = fabs(coord - motor_get_cur_step(addr));
            }
            break;

        case 2:  // 反向运动
            // 如果已经到达后限位
            if (motor_get_cur_step(addr) <= m_motorctrl[addr-1].backward_limit) {
            	xil_printf("cmd达到后限位状态！time: %lld \n\r", timer_get_1us());
                return 0;
            }
            // 如果目标坐标超出后限位
            else if (coord <= m_motorctrl[addr-1].backward_limit) {
                // 计算到后限位的距离
                move_steps = fabs(m_motorctrl[addr-1].backward_limit - motor_get_cur_step(addr));
            } else {
                // 计算到目标位置的距离
                move_steps = fabs(coord - motor_get_cur_step(addr));
            }
            break;

        default:
            break;
    }

    return move_steps;
}







/**
 * @brief 按步数移动电机
 *
 * 根据指定步数和方向移动电机，并进行限位检查
 *
 * @param addr 电机地址
 * @param dir 运动方向(1:正向, 2:反向)
 * @param step 移动步数(正值)
 */
void motor_move_step(MOTOR_ADDR addr, uint8_t dir, uint32_t step)
{
    int32_t move_steps = 0;
    int32_t move_speed = 2000;
    motor_dir_e tmp_dir = MOTOR_DIR_NONE;

    if (!motor_addr_check(addr))
        return;

//    // 记录运动时间并切换到活动模式
    //motor_record_move_time(addr);
   // motor_set_active_mode(addr);

    tmp_dir = m_motor[addr - 1].dir;
    // 记录运动方向
    m_motor[addr - 1].dir = dir;

    // 进行限位检查，获取实际可移动步数
    move_steps = step_limit_check(addr, step);

    // 如果不能移动则直接返回
    if ((move_steps == 0) || (dir == 0)) {
        motor_stop(addr);
        return;
    }

#ifdef LOW_SPEED_TEST
    if(1)
#else
    // 根据步数大小选择控制模式
    if (fabsf(move_steps) <= MOTOR_STEP_DIRECT_RUN)
#endif
    {
        move_speed = move_steps*4;

        switch(m_motor_speed_level[addr-1].cur_level) {
			case 0: move_speed = 1000; break;
			case 1: move_speed = 3000; break;
			case 2: move_speed = 6000; break;
			case 3: move_speed = 9000; break;
			case 4: move_speed = 12000; break;
			default:move_speed = 5000; break;
        }


        if (move_speed < 1000)
            move_speed = 1000;

        motor_move_flag[addr - 1] = 0;
        is_pid_flag[addr - 1] = 0;

#ifdef MOTOR_CTRL_CURVE
        motor_motion_start( addr, m_motor[addr - 1].dir, move_steps, move_speed );
#else
        // 小步数模式：直接控制
        motor_control(addr, m_motor[addr - 1].dir, 0, 0);
        motor_control(addr, m_motor[addr - 1].dir, move_speed, move_steps);
        xil_printf("test_1\r\n");
#endif

        xil_printf("电机移动步数: %d,  当前步数： %d,  目标步数: %d \n",

        		(int)move_steps,
				(int)motor_get_cur_step(addr),
				( dir == 1 )? ((int)(motor_get_cur_step(addr)+move_steps)):((int)(motor_get_cur_step(addr)-move_steps)) );
    } else {
#if 1
        if (motor_move_flag[addr - 1] == 1) {
            // 判断是否反转
            if (tmp_dir != m_motor[addr - 1].dir) {
                motionProfiles[addr-1].reverse = 1;
            }

            // 大步数模式：PID控制
            targetAngles[addr - 1] = (m_motor[addr - 1].dir == 1)
                          ? (motor_get_cur_step(addr) + (float)move_steps)
                          : (motor_get_cur_step(addr) - (float)move_steps);
        } else {
        	PID_Start( addr, m_motor[addr - 1].dir, move_steps, 1 );
        	xil_printf("test_1\r\n");
        	xil_printf("test大步数模式 - 电机%d: 速度等级=%d, 步数=%d\r\n",
        	    addr-1, m_motor_speed_level[addr-1].cur_level, move_steps);
        	xil_printf("电机移动步数: %ld,  当前步数： %d,  目标步数: %d \n", move_steps, (int)motor_get_cur_step(addr), (int)targetAngles[addr - 1]);
        }

#ifdef PID_CONTROL_NEW
        MotionProfile_Start(&motionProfiles[addr - 1], targetAngles[addr - 1]);
#endif
#else
        motor_motion_start( addr, m_motor[addr - 1].dir, move_steps, 30000 );
#endif
    }


}

/**
 * @brief 按坐标位置移动电机
 *
 * 移动电机到指定坐标位置，并进行限位检查
 *
 * @param addr 电机地址
 * @param coord 目标坐标位置
 */
void motor_move_coord(MOTOR_ADDR addr, int32_t coord)
{
    int32_t move_steps = 0;
    int32_t move_speed = 2000;
    motor_dir_e tmp_dir = MOTOR_DIR_NONE;

    if (!motor_addr_check(addr))
        return;

//    // 记录运动时间并切换到活动模式
    //motor_record_move_time(addr);
    //motor_set_active_mode(addr);

    tmp_dir = m_motor[addr - 1].dir;
    // 进行坐标限位检查，获取实际移动步数 确定方向
    move_steps = coord_limit_check(addr, coord);

#ifdef LOW_SPEED_TEST
    if(1)
#else
    // 根据步数大小选择控制模式
    if (fabsf(move_steps) <= MOTOR_STEP_DIRECT_RUN)
#endif
    {
        move_speed = move_steps*4;
        switch(m_motor_speed_level[addr-1].cur_level) {
        case 0: move_speed = 1000; break;
        case 1: move_speed = 3000; break;
        case 2: move_speed = 6000; break;
        case 3: move_speed = 9000; break;
        case 4: move_speed = 12000; break;
        default:move_speed = 5000; break;
        }

        if (move_speed < 1000)
            move_speed = 1000;

        motor_move_flag[addr - 1] = 0;
        is_pid_flag[addr - 1] = 0;
#ifdef MOTOR_CTRL_CURVE
        motor_motion_start( addr, m_motor[addr - 1].dir, move_steps, move_speed );
#else
        // 小步数模式：直接控制
        motor_control(addr, m_motor[addr - 1].dir, 0, 0);
        motor_control(addr, m_motor[addr - 1].dir, move_speed, move_steps);
#endif
    } else {
#if 1
        if (motor_move_flag[addr - 1] == 1) {
            // 判断是否反转
            if (tmp_dir != m_motor[addr - 1].dir) {
                motionProfiles[addr-1].reverse = 1;
            }
            // 大步数模式：PID控制
            targetAngles[addr - 1] = (m_motor[addr - 1].dir == 1)
                          ? (motor_get_cur_step(addr) + (float)move_steps)
                          : (motor_get_cur_step(addr) - (float)move_steps);
        } else {
        	PID_Start( addr, m_motor[addr - 1].dir, move_steps, 1 );
        }
        xil_printf("testb大步数模式 - 电机%d: 速度等级=%d, 实际max_vel=%d, 步数=%d\r\n",
            addr-1, m_motor_speed_level[addr-1].cur_level,
            (int)motionProfiles[addr-1].max_vel, move_steps);
#ifdef PID_CONTROL_NEW
        MotionProfile_Start(&motionProfiles[addr - 1], targetAngles[addr - 1]);
#endif
#else
        motor_motion_start( addr, m_motor[addr - 1].dir, move_steps, 30000 );
#endif
    }
}

/**
 * @brief 按指定速度和步数移动电机
 *
 * 根据指定速度和步数移动电机，并进行限位检查
 *
 * @param addr 电机地址
 * @param dir 运动方向(1:正向, 2:反向)
 * @param speed 运动速度
 * @param step 移动步数
 */
#if 1
void motor_move_speed_step(MOTOR_ADDR addr, uint8_t dir, uint32_t speed, int32_t step)
{
    int32_t move_steps = 0;

    if (!motor_addr_check(addr))
        return;

//    // 记录运动时间并切换到活动模式
    //motor_record_move_time(addr);
    //motor_set_active_mode(addr);
    PID_Stop(addr);
    // 记录运动方向
    m_motor[addr - 1].dir = dir;

    // 进行限位检查，获取实际可移动步数
    move_steps = step_limit_check(addr, step);

    // 如果不能移动则直接返回
    if ((move_steps == 0) || (dir == 0)) {
        motor_stop(addr);
        return;
    }

    xil_printf("电机移动速度:%ld,电机移动步数: %ld\n",speed, move_steps);

#ifdef MOTOR_CTRL_CURVE
        motor_motion_start( addr, m_motor[addr - 1].dir, move_steps, speed );
#else
    motor_control(addr, m_motor[addr - 1].dir, 0, 0);
    motor_control(addr, m_motor[addr - 1].dir, speed, move_steps);
#endif
}

#endif
#if 0
void motor_move_speed_step(MOTOR_ADDR addr, uint8_t dir, uint32_t speed, int32_t step)
{
    int32_t move_steps = 0;

    if (!motor_addr_check(addr))
        return;

    // 1. 必须恢复唤醒逻辑：确保电机从待机电流切换回正常工作电流
    motor_record_move_time(addr);
    motor_set_active_mode(addr);

    // 2. 记录运动方向
    m_motor[addr - 1].dir = dir;

    // 3. 安全检查：获取经过限位修正后的实际步数[cite: 11, 12]
    move_steps = step_limit_check(addr, step);

    if ((move_steps == 0) || (dir == 0)) {
        motor_stop(addr);
        return;
    }

    // 4. 解决异响的关键：强制使用曲线控制
    // 即使上位机下发的是固定速度值，软件层也应将其作为“目标速度”
    // 在 motor_motion_start 内部通过定时器分步更新频率，消除阶跃冲击
#define MOTOR_CTRL_CURVE // 确保此宏开启
#ifdef MOTOR_CTRL_CURVE
    // 使用 S 型或梯形算法启动，内部会自动处理加速过程
    motor_motion_start(addr, m_motor[addr - 1].dir, move_steps, speed);
#else
    // 如果不使用曲线，至少先给一个起跳频率（底频），再切到目标速度
    // 但这种做法效果不如曲线控制平滑
    motor_control(addr, dir, 1000, 0); // 1kHz 起跳
    motor_control(addr, dir, speed, move_steps);
#endif

    xil_printf("M%d 启动：目标速度=%ld, 步数=%ld\r\n", addr, speed, move_steps);
}


#endif

/**
 * @brief 初始化待机模式
 */
void motor_standby_init( void )
{
    for ( int addr = TMC2660_M1; addr <= TMC2660_M5; addr++ )
    {
        // 保存每个电机的正常电流值
        m_motor_standby[addr-1].normal_current = m_motorctrl[addr-1].current;
        m_motor_standby[addr-1].mode = MOTOR_MODE_STANDBY;
        m_motor_standby[addr-1].standby_enabled = 1;
        m_motor_standby[addr-1].last_move_time = 0;

        // 初始时设置待机电流以降低发热
        motor_set_current( addr, STANDBY_CURRENT );
        xil_printf("M%d: 待机模式初始化, 正常电流=%d, 待机电流=%d\r\n",
                   addr, m_motor_standby[addr-1].normal_current, STANDBY_CURRENT);
    }
}

/**
 * @brief 记录电机运动时间
 */
void motor_record_move_time( MOTOR_ADDR addr )
{
    if (!motor_addr_check(addr))
        return;

    m_motor_standby[addr-1].last_move_time = timer_get_1us();
}

/**
 * @brief 设置电机为活动模式(正常电流)
 */
/*
void motor_set_active_mode( MOTOR_ADDR addr )
{
    if (!motor_addr_check(addr))
        return;

    if (m_motor_standby[addr-1].mode == MOTOR_MODE_STANDBY)
    {
        m_motor_standby[addr-1].mode = MOTOR_MODE_ACTIVE;
        motor_set_current( addr, m_motor_standby[addr-1].normal_current );
        xil_printf("M%d: 切换到活动模式, 恢复电流=%d\r\n",
                   addr, m_motor_standby[addr-1].normal_current);
    }
}
*/
/**
 * @brief 设置电机为待机模式(低电流)
 */
/*
void motor_set_standby_mode( MOTOR_ADDR addr )
{
    if (!motor_addr_check(addr))
        return;

    if (m_motor_standby[addr-1].mode == MOTOR_MODE_ACTIVE)
    {
        m_motor_standby[addr-1].mode = MOTOR_MODE_STANDBY;
        motor_set_current( addr, STANDBY_CURRENT );
        xil_printf("M%d: 切换到待机模式, 降低电流=%d\r\n", addr, STANDBY_CURRENT);
    }
}
*/
void motor_set_standby_mode_all(void)
{
	//for(int addr = TMC2660_M1;addr <= TMC2660_M5; addr++)
		//motor_set_standby_mode( addr );

}

/**
 * @brief 待机模式处理函数
 * 定期检查电机运动状态,自动切换待机/活动模式
 */
#if 0
void motor_standby_process( void )
{
    uint64_t current_time = timer_get_1us();
    uint64_t time_diff_ms;

    // 每秒检查一次
    if ( (current_time - m_last_standby_check_time) < (STANDBY_CHECK_INTERVAL_MS * 1000) )
    {
        return;
    }
    m_last_standby_check_time = current_time;

    // 检查所有电机的状态
    for ( int addr = TMC2660_M1; addr <= TMC2660_M5; addr++ )
    {
        // 检查是否有运动任务在进行
        uint8_t has_move_task = 0;

        // 检查PID运动标志
        if ( motor_move_flag[addr-1] == 1 )
        {
            has_move_task = 1;
        }
        // 检查直接运动标志(步数计数)
        else if ( m_motor[addr-1].dir != MOTOR_DIR_NONE )
        {
            has_move_task = 1;
        }

        if ( has_move_task )
        {
            // 有运动任务,确保在活动模式
            if ( m_motor_standby[addr-1].mode == MOTOR_MODE_STANDBY )
            {
                motor_set_active_mode( addr );
            }
            // 更新最后运动时间
            m_motor_standby[addr-1].last_move_time = current_time;
        }
        else
        {
            // 无运动任务,检查超时
            if ( m_motor_standby[addr-1].mode == MOTOR_MODE_ACTIVE )
            {
                if ( m_motor_standby[addr-1].last_move_time == 0 )
                {
                    // 从未运动过,直接进入待机
                    motor_set_standby_mode( addr );
                }
                else
                {
                    // 计算距离上次运动的时间
                    time_diff_ms = (current_time - m_motor_standby[addr-1].last_move_time) / 1000;

                    // 超时则进入待机模式
                    if ( time_diff_ms >= STANDBY_TIMEOUT_MS )
                    {
                        motor_set_standby_mode( addr );
                    }
                }
            }
        }
    }
}
#endif
#if 0
void motor_standby_process( void )
{
    uint64_t current_time = timer_get_1us();

    // 每秒检查一次
    if ( (current_time - m_last_standby_check_time) < (STANDBY_CHECK_INTERVAL_MS * 1000) )
    {
        return;
    }
    m_last_standby_check_time = current_time;

    static uint32_t debug_print_cnt = 0;
    uint8_t need_debug_print = (debug_print_cnt++ % 3 == 0);

    // 【核心修复 1】：引入独立的私有闲置计数器，彻底斩断外部 last_move_time 的续杯污染
    // 用静态数组记录每个电机“真正停稳了多少秒”
    static uint32_t s_motor_idle_seconds[5] = {0};

//    if (need_debug_print) {
//        xil_printf("\r\n--- [STANDBY SECURITY CHECK] CurTime: %lld s ---\r\n", (unsigned long long)current_time);
//    }

    for ( int addr = TMC2660_M1; addr <= TMC2660_M5; addr++ )
    {
        int idx = addr - 1;
        uint8_t has_move_task = (motor_move_flag[idx] == 1); // 纯粹靠 PID flag 决定生死

        if ( has_move_task )
        {
            if ( m_motor_standby[idx].mode == MOTOR_MODE_STANDBY )
            {
                motor_set_active_mode( addr );
            }
            // 只要电机还在干活，私有计数器无脑归零，并同步刷新
            s_motor_idle_seconds[idx] = 0;
            m_motor_standby[idx].last_move_time = current_time;
        }
        else
        {
            // 电机停稳了 (flag == 0)
            if ( m_motor_standby[idx].mode == MOTOR_MODE_ACTIVE )
            {
                // 【核心修复 2】：每秒进来一次，私有秒数计数器雷打不动地自增 1 秒！
                // 不管外面的 last_move_time 怎么发疯续杯，我们只相信自己每秒加 1 的死数！
                s_motor_idle_seconds[idx]++;

                // 【核心修复 3】：防御性强制防内存污染。
                // 如果发现外部宏或变量被越界踩成 0 了，底线保底 3 秒！
                uint32_t timeout_sec = STANDBY_TIMEOUT_MS / 1000;
                if (timeout_sec == 0) {
                    timeout_sec = 3;
                }

//                if (need_debug_print) {
//                    xil_printf("  M%d: Tracking | RealIdleTime: %d s / SafeTarget: %d s\r\n",
//                               addr, s_motor_idle_seconds[idx], timeout_sec);
//                }

                // 严格比对私有秒数，满 3 秒直接切低功耗待机
                if ( s_motor_idle_seconds[idx] >= timeout_sec )
                {
                    motor_set_standby_mode( addr );
                    s_motor_idle_seconds[idx] = 0; // 进入睡眠，计数器交棒
                }
            }
            else
            {
//                if (need_debug_print) {
//                    xil_printf("  M%d: Sleeping | Quiet.\r\n", addr);
//                }
                s_motor_idle_seconds[idx] = 0; // 已经在睡眠，保持归零
            }
        }
    }
}
#endif


/**
 * @brief 专属低速运动函数 (motor_mode == 1 专用)
 * @param addr 电机物理地址 (1~5)
 * @param dir  运动方向 (1:正转, 2:反转)
 * @param step 移动总步数
 */
uint8_t g_m2_low_speed_run_flag = 0; // 0:停止, 1:正转一直走, 2:反转一直走
uint32_t g_m2_run_speed = 0;         // 记录运行速度

	void motor_low_speed_run(MOTOR_ADDR addr, uint8_t dir, uint32_t speed)
	{
	    uint8_t tmp_dir = 0;

	    if (addr != 2) return; // 本通道只特殊服务电机2

	    // 1. 如果收到的是 1A 下的 00 00 停止指令
	    if (dir == 0)
	    {
	        m_motor[addr - 1].dir = 0;
	        pl_write_u32(WRITE_ADDR_MOTOR1 + (addr - 1) * 4, 0); // 硬件直接清零急停
	        return;
	    }

	    // 2. 正常同步软件方向标志
	    m_motor[addr - 1].dir = dir;
	    if (dir == MOTOR_DIR_FORWARD)
	        tmp_dir = 0;
	    else
	        tmp_dir = 1;

	    // 3. 【核心创新】：获取电机2当前的绝对物理坐标
	    int32_t cur_pos = (int32_t)motor_get_cur_step(2);
	    uint32_t safe_step = 0;

	    if (dir == 1) // 正转脱困（目标是往正方向冲到接近 18000）
	    {
	        // 如果当前还在深坑里（比如坐标在 -18000 到 0 之间）
	        if (cur_pos < 0)
	        {
	            // 距离中点和终点还很远，我们直接下一发绝对安全的 30000 步让它拼命跑
	            // 完美规避 &0x7FFF 抹零
	            safe_step = 30000;
	        }
	        else
	        {
	            // 如果电机已经成功逃离负半周，越过了 0 点（比如到了 +2000）
	            // 此时距离终点 18000 只剩下不到 16000 步了！
	            // 我们直接动态计算出精确的剩余步数：
	            int32_t remain = 18000 - cur_pos;
	            if (remain < 0) remain = 0;

	            safe_step = (uint32_t)remain; // 剩下的步数完美小于 32767，直接一发收尾！
	        }
	    }
	    else // 反转脱困（如果从正限位往负向退）
	    {
	        if (cur_pos > 0)
	        {
	            safe_step = 30000; // 在正半周时，无脑下发安全的 30000 步倒车
	        }
	        else
	        {
	            // 越过 0 点进入负半周，计算距离负限位 -18000 还差多少步
	            int32_t remain = cur_pos - (-18000);
	            if (remain < 0) remain = 0;

	            safe_step = (uint32_t)remain; // 剩下的尾数安全下发
	        }
	    }

	    // 4. 最终物理下发（经过坐标动态裁剪后，safe_step 永远不可能超过 32000！）
	    pl_write_u32(WRITE_ADDR_MOTOR1 + (addr - 1) * 4, (speed << 16) | (tmp_dir << 15) | (safe_step & 0x7FFF));
	}

	// 仿照 hal 的加速度斜率限制器 (ClampFreq)
	/**
	 * @brief 防过冲 X 轴纯比例(P)坐标跟踪器
	 * 核心原理：严格限制 `steps` 绝不超过剩余步数 `abs_error`，从物理上断绝“冲过头又退回来”的现象。
	 */
