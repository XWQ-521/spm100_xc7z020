/*
 * motor_curve.c
 *
 *  Created on: 2025年11月23日
 *      Author: dulc
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "drv_motor.h"
#include "hal_motor.h"
#include "motor_curve.h"
#include "hal_timer.h"
#include "motorctrl.h"
#include "pid.h"
#include "version.h"
#include "xil_printf.h"
#include "cmd_shell.h"

#define MOTOR_START_SPEED		3000	// 启动停止速度
#define MTOOR_NORMAL_ACC		800     // 正常加速度
#define MOTOR_EMER_ACC			800     // 急停加速度

typedef enum {
	MOTOR_STATE_INIT = 0,
	MOTOR_STATE_START,
	MOTOR_STATE_SPEEDUP,
	MOTOR_STATE_RUN,
	MOTOR_STATE_SPEEDDOWN,
	MOTOR_STATE_STOP,
	MOTOR_STATE_KEEPSTOP,
	MOTOR_STATE_ROLLBACK_SPEEDDOWN,
	MOTOR_STATE_ROLLBACK_STOP,
	MOTOR_STATE_ROLLBACK_WAIT,
	MOTOR_STATE_ROLLBACK_SPEEDUP,
}	motor_state_e;

typedef struct {
	uint8_t run_state;		// 当前状态
	uint8_t cur_dir;		// 当前方向
	uint8_t set_dir;		// 设置方向
	uint32_t cur_speed;		// 当前速度
	uint32_t target_speed;	// 目标速度
	int32_t start_step;     // 启动的坐标
	int32_t cur_step;		// 当前坐标
	int32_t target_step;	// 目标坐标
	uint32_t run_step;		// 运动步数
	uint32_t run_count;		// 加速计数
	uint32_t inc_step;		// 加速步数
	uint64_t cur_time;
	uint8_t  stop_flag;
	uint64_t last_log_time; // 周期心跳打印时间
}	run_ctrl_t;

run_ctrl_t m_run_ctrl[MOTOR_COUNT] = {
		{ MOTOR_STATE_KEEPSTOP, MOTOR_DIR_NONE, MOTOR_DIR_NONE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ MOTOR_STATE_KEEPSTOP, MOTOR_DIR_NONE, MOTOR_DIR_NONE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ MOTOR_STATE_KEEPSTOP, MOTOR_DIR_NONE, MOTOR_DIR_NONE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ MOTOR_STATE_KEEPSTOP, MOTOR_DIR_NONE, MOTOR_DIR_NONE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ MOTOR_STATE_KEEPSTOP, MOTOR_DIR_NONE, MOTOR_DIR_NONE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

static uint8_t g_motor_curve_log_en = 0;

void motor_run_control( MOTOR_ADDR motor_id );

static void cmd_motor_curve_help( void )
{
	xil_printf("\r\n--- [Motor Curve Command Help] ---\r\n");
	xil_printf("  motor_curve log on|1     : 开启电机加减速与实时轨迹日志\r\n");
	xil_printf("  motor_curve log off|0    : 关闭日志 (静音模式)\r\n");
	xil_printf("  motor_curve dump <1~5>   : 打印指定电机状态与限位\r\n");
	xil_printf("----------------------------------\r\n\r\n");
}

static int cmd_motor_curve( int argc, char *argv[] )
{
	if ( argc < 2 || strcmp( argv[1], "help" ) == 0 )
	{
		cmd_motor_curve_help();
		return 1;
	}

	if ( strcmp( argv[1], "log" ) == 0 )
	{
		if ( argc >= 3 )
		{
			if ( strcmp( argv[2], "on" ) == 0 || strcmp( argv[2], "1" ) == 0 )
			{
				g_motor_curve_log_en = 1;
				xil_printf("[MOTOR_CMD] Motor curve log: ENABLED\r\n");
				return 1;
			}
			else if ( strcmp( argv[2], "off" ) == 0 || strcmp( argv[2], "0" ) == 0 )
			{
				g_motor_curve_log_en = 0;
				xil_printf("[MOTOR_CMD] Motor curve log: DISABLED\r\n");
				return 1;
			}
		}
		xil_printf("[MOTOR_CMD] Usage: motor_curve log <on|off|1|0>\r\n");
		return 1;
	}

	if ( strcmp( argv[1], "dump" ) == 0 )
	{
		if ( argc >= 3 )
		{
			int motor_id = atoi( argv[2] );
			if ( motor_id >= TMC2660_M1 && motor_id <= TMC2660_M5 )
			{
				int idx = motor_id - 1;
				int32_t cur = motor_get_cur_step( motor_id );
				xil_printf("\r\n========== [M%d STATUS DUMP] ==========\r\n", motor_id);
				xil_printf(" FSM State  : %d\r\n", m_run_ctrl[idx].run_state);
				xil_printf(" Cur Pos    : %d\r\n", cur);
				xil_printf(" Target Pos : %d\r\n", m_run_ctrl[idx].target_step);
				xil_printf(" Limits     : Back=%d | Fwd=%d\r\n", m_motorctrl[idx].backward_limit, m_motorctrl[idx].forward_limit);
				xil_printf(" Speed      : Cur=%u / Target=%u\r\n", m_run_ctrl[idx].cur_speed, m_run_ctrl[idx].target_speed);
				xil_printf(" Direction  : Set=%d, Cur=%d\r\n", m_run_ctrl[idx].set_dir, m_run_ctrl[idx].cur_dir);
				xil_printf("========================================\r\n\r\n");
				return 1;
			}
		}
		xil_printf("[MOTOR_CMD] Usage: motor_curve dump <1~5>\r\n");
		return 1;
	}

	xil_printf("[MOTOR_CMD] Unknown argument. Type 'motor_curve help'\r\n");
	return 1;
}

void motor_curve_cmd_register( void )
{
	cmd_shell_register_ext(
		"motor_curve",
		cmd_motor_curve,
		cmd_motor_curve_help
	);
}

void motor_curve_process( void )
{
	for( int motor_id = TMC2660_M1; motor_id <= TMC2660_M5; motor_id++ )
	{
		motor_run_control( motor_id );
	}
}

void motor_motion_start( MOTOR_ADDR motor_id, motor_dir_e dir, uint32_t step, uint32_t speed )
{
	int idx = motor_id - 1;
	int32_t current_pos = motor_get_cur_step(motor_id);

	// 限位边缘拦截
	if (dir == MOTOR_DIR_FORWARD && current_pos >= (m_motorctrl[idx].forward_limit - 5)) {
		xil_printf("M%d 拒绝启动：已处于正限位边缘！当前步数：%d\r\n", motor_id, current_pos);
		motor_stop(motor_id);
		m_run_ctrl[idx].run_state = MOTOR_STATE_KEEPSTOP;
		return;
	}
	if (dir == MOTOR_DIR_BACKWARD && current_pos <= (m_motorctrl[idx].backward_limit + 5)) {
		xil_printf("M%d 拒绝启动：已处于负限位边缘！当前步数：%d\r\n", motor_id, current_pos);
		motor_stop(motor_id);
		m_run_ctrl[idx].run_state = MOTOR_STATE_KEEPSTOP;
		return;
	}

	m_run_ctrl[idx].set_dir = dir;
	m_run_ctrl[idx].target_speed = speed;
	m_run_ctrl[idx].run_step = step;
	m_run_ctrl[idx].start_step = current_pos;
	m_run_ctrl[idx].cur_step = current_pos;

	PID_Stop( motor_id );

	// 还原你原本最熟悉的启动格式
	if ( dir == MOTOR_DIR_FORWARD )
	{
		m_run_ctrl[idx].target_step = current_pos + step;
		xil_printf( "正转： 启动步数： %d,  目标步数： %d", m_run_ctrl[idx].start_step, m_run_ctrl[idx].target_step );
	}
	else if ( dir == MOTOR_DIR_BACKWARD )
	{
		m_run_ctrl[idx].target_step = current_pos - step;
		xil_printf( "反转：  启动步数： %d,  目标步数： %d", m_run_ctrl[idx].start_step, m_run_ctrl[idx].target_step );
	}

	if ( (dir == MOTOR_DIR_NONE) || (speed == 0) || (step == 0) )
	{
		xil_printf("不处理，停止运动： %d   %d   %d\r\n", dir, speed, step );
		if ( m_run_ctrl[idx].run_state != MOTOR_STATE_KEEPSTOP )
		{
			m_run_ctrl[idx].stop_flag = 1;
			m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDDOWN;
		} else {
			motor_stop( motor_id );
			return;
		}
	}

	m_motor[idx].dir = dir;

	switch ( dir )
	{
	case MOTOR_DIR_NONE:
		break;

	case MOTOR_DIR_FORWARD:
	case MOTOR_DIR_BACKWARD:
		if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_NONE )
		{
			xil_printf("正常启动： %d   %d   %d\r\n", dir, speed, step );
			m_run_ctrl[idx].run_state = MOTOR_STATE_INIT;
		} else
		{
			if ( m_run_ctrl[idx].cur_dir == dir )
						{
							switch( m_run_ctrl[idx].run_state )
							{
							case MOTOR_STATE_SPEEDUP:
							case MOTOR_STATE_RUN:
								xil_printf("指令不做处理：\r\n");
								break;

							case MOTOR_STATE_SPEEDDOWN:
								// 核心防堵转：急停/减速中重启动，强制降回 3000 初速重新爬坡，禁止带着 24000 高速硬拔起步
								xil_printf("减速中重启动：重置为安全起步速度 3000，重新爬坡！\r\n");
								m_run_ctrl[idx].cur_speed = MOTOR_START_SPEED; // 强制降回 3000
								m_run_ctrl[idx].stop_flag = 0;                 // 清除停止标志
								m_run_ctrl[idx].run_count = 0;
								m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDUP;
								break;

							default:
								m_run_ctrl[idx].run_state = MOTOR_STATE_INIT;
								break;
							}
						}else {
				xil_printf("减速反转： %d   %d   %d\r\n", dir, speed, step );
				m_run_ctrl[idx].run_state = MOTOR_STATE_ROLLBACK_SPEEDDOWN;
			}
		}
		break;

	default:
		break;
	}
}

void motor_run_control( MOTOR_ADDR motor_id )
{
	static uint64_t run_time[MOTOR_COUNT] = {0};
	static uint32_t tmp_random_speed = 10;
	static uint32_t pre_state[MOTOR_COUNT] = {0};
	int idx = motor_id - 1;

	if ( timer_get_diff_1us( run_time[idx], timer_get_1us() ) < 1000 )
		return;

	run_time[idx] = timer_get_1us();

	// 1. 还原你最习惯的状态切换打印
	if ( pre_state[motor_id] != m_run_ctrl[idx].run_state )
	{
		pre_state[motor_id] = m_run_ctrl[idx].run_state;
		xil_printf("M%d 当前状态： =====STATE:%d===== time: %d \r\n", motor_id, m_run_ctrl[idx].run_state, (int)timer_get_1us() );
	}

	// 2. 核心诊断：开启日志后，运转阶段每 100ms 打印一次心跳（堵转、丢步一目了然）
	if ( g_motor_curve_log_en &&
	    (m_run_ctrl[idx].run_state == MOTOR_STATE_SPEEDUP ||
	     m_run_ctrl[idx].run_state == MOTOR_STATE_RUN ||
	     m_run_ctrl[idx].run_state == MOTOR_STATE_SPEEDDOWN) )
	{
		if ( timer_get_diff_1us( m_run_ctrl[idx].last_log_time, timer_get_1us() ) >= 100000 ) // 100ms
		{
			m_run_ctrl[idx].last_log_time = timer_get_1us();
			xil_printf("[M%d 动态] 状态:%d, 当前步数:%d, 目标步数:%d, 速度:%u\r\n",
			           motor_id, m_run_ctrl[idx].run_state, motor_get_cur_step(motor_id),
			           m_run_ctrl[idx].target_step, m_run_ctrl[idx].cur_speed);
		}
	}

	switch( m_run_ctrl[idx].run_state )
	{
	case MOTOR_STATE_INIT:
		m_run_ctrl[idx].cur_dir = m_run_ctrl[idx].set_dir;
		m_run_ctrl[idx].cur_speed = MOTOR_START_SPEED;
		m_run_ctrl[idx].start_step = motor_get_cur_step( motor_id );
		m_run_ctrl[idx].cur_step = motor_get_cur_step( motor_id );
		m_run_ctrl[idx].run_count = 0;

		motor_control( motor_id, m_run_ctrl[idx].cur_dir, m_run_ctrl[idx].cur_speed, m_run_ctrl[idx].run_step );
		m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDUP;
		break;

	case MOTOR_STATE_START:
		break;

	// 加速
	case MOTOR_STATE_SPEEDUP:
		m_run_ctrl[idx].run_count++;
		m_run_ctrl[idx].cur_speed += MTOOR_NORMAL_ACC;

		motor_control( motor_id, m_run_ctrl[idx].cur_dir, m_run_ctrl[idx].cur_speed, m_run_ctrl[idx].run_step );

		// 达到目标速度
		if ( m_run_ctrl[idx].cur_speed >= m_run_ctrl[idx].target_speed )
		{
			m_run_ctrl[idx].inc_step = m_run_ctrl[idx].run_step - fabsf( motor_get_cur_step( motor_id ) - m_run_ctrl[idx].target_step);
			m_run_ctrl[idx].run_state = MOTOR_STATE_RUN;
		}

		// 短距离提前减速（完全还原你的原版格式）
		if ( fabsf( motor_get_cur_step( motor_id ) - m_run_ctrl[idx].start_step) >= m_run_ctrl[idx].run_step/2 )
		{
			xil_printf("开始减速1：%d\r\n", m_run_ctrl[idx].run_state );
			m_run_ctrl[idx].inc_step = m_run_ctrl[idx].run_step - fabsf( motor_get_cur_step( motor_id ) - m_run_ctrl[idx].target_step);
			m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDDOWN;
		}

		// 正向限位减速（完全还原你的原版格式）
		if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_FORWARD )
		{
			if ( fabsf( motor_get_cur_step( motor_id ) - m_run_ctrl[idx].start_step) >= fabsf( m_run_ctrl[idx].start_step - m_motorctrl[idx].forward_limit)/2 )
			{
				xil_printf("开始减速2：%d 起始步数：%d 当前步数:%d   限位步数： %d,  运行步数：%d\r\n",
						        m_run_ctrl[idx].run_state,
								m_run_ctrl[idx].start_step,
								motor_get_cur_step( motor_id ),
								m_motorctrl[idx].forward_limit,
								m_run_ctrl[idx].run_step/2 );
				m_run_ctrl[idx].inc_step = m_run_ctrl[idx].run_step - fabsf( motor_get_cur_step( motor_id ) - m_run_ctrl[idx].target_step);
				m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDDOWN;
			}
		}
		// 反向限位减速（完全还原你的原版格式）
		else if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_BACKWARD )
		{
			if ( fabsf( motor_get_cur_step( motor_id ) - m_run_ctrl[idx].start_step) >= fabsf( m_run_ctrl[idx].start_step - m_motorctrl[idx].backward_limit)/2 )
			{
				xil_printf("开始减速3：%d  起始步数：%d  当前步数:%d   限位步数： %d,  运行步数：%d\r\n",
						        m_run_ctrl[idx].run_state,
								m_run_ctrl[idx].start_step,
								motor_get_cur_step( motor_id ),
								m_motorctrl[idx].backward_limit,
								m_run_ctrl[idx].run_step/2 );
				m_run_ctrl[idx].inc_step = m_run_ctrl[idx].run_step - fabsf( motor_get_cur_step( motor_id ) - m_run_ctrl[idx].target_step);
				m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDDOWN;
			}
		}
		break;

	// 匀速
	case MOTOR_STATE_RUN:
		tmp_random_speed++;
		if ( tmp_random_speed > 20 )
			tmp_random_speed = 0;
		motor_control( motor_id, m_run_ctrl[idx].cur_dir, m_run_ctrl[idx].cur_speed, m_run_ctrl[idx].run_step+tmp_random_speed );

		if ( fabsf( motor_get_cur_step( motor_id ) - m_run_ctrl[idx].target_step) <= m_run_ctrl[idx].inc_step )
		{
			m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDDOWN;
		}

		// 限位减速判定
		if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_FORWARD )
		{
			if ( fabsf( motor_get_cur_step( motor_id ) - m_motorctrl[idx].forward_limit ) <= m_run_ctrl[idx].inc_step )
			{
				m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDDOWN;
			}
		}
		else if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_BACKWARD )
		{
			if ( fabsf( motor_get_cur_step( motor_id ) - m_motorctrl[idx].backward_limit ) <= m_run_ctrl[idx].inc_step )
			{
				m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDDOWN;
			}
		}

		// 限位停机判定（完全还原原版）
		if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_FORWARD )
		{
			if ( motor_get_cur_step( motor_id ) >= (m_motorctrl[idx].forward_limit-10) )
			{
				xil_printf("M%d: 达到前限位状态！cur_step: %d, limit: %d\r\n", motor_id, motor_get_cur_step(motor_id), m_motorctrl[idx].forward_limit);
				motor_stop( motor_id );
				m_run_ctrl[idx].run_state = MOTOR_STATE_STOP;
			}
		}
		else if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_BACKWARD )
		{
			if ( motor_get_cur_step( motor_id ) <= (m_motorctrl[idx].backward_limit+10) )
			{
				xil_printf("M%d: 达到后限位状态！cur_step: %d, limit: %d\r\n", motor_id, motor_get_cur_step(motor_id), m_motorctrl[idx].backward_limit);
				motor_stop( motor_id );
				m_run_ctrl[idx].run_state = MOTOR_STATE_STOP;
			}
		}
		break;

	// 减速
	case MOTOR_STATE_SPEEDDOWN:
		m_run_ctrl[idx].cur_speed -= MTOOR_NORMAL_ACC;
		if ( m_run_ctrl[idx].cur_speed < MOTOR_START_SPEED )
		{
			m_run_ctrl[idx].cur_speed = MOTOR_START_SPEED;

			if ( m_run_ctrl[idx].stop_flag == 1 )
			{
				m_run_ctrl[idx].stop_flag = 0;
				motor_stop( motor_id );
				m_run_ctrl[idx].run_state = MOTOR_STATE_STOP;
			}
		}
		motor_control( motor_id, m_run_ctrl[idx].cur_dir, m_run_ctrl[idx].cur_speed, m_run_ctrl[idx].run_step );

		// 到位判断（还原原版 111 / 222）
		if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_FORWARD )
		{
			if ( motor_get_cur_step( motor_id ) >= (m_run_ctrl[idx].target_step-10) )
			{
				xil_printf("到位判断111： %d\r\n", m_run_ctrl[idx].target_step );
				motor_stop( motor_id );
				m_run_ctrl[idx].run_state = MOTOR_STATE_STOP;
			}
		}
		else if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_BACKWARD )
		{
			if ( motor_get_cur_step( motor_id ) <= (m_run_ctrl[idx].target_step+10) )
			{
				xil_printf("到位判断222： %d\r\n", m_run_ctrl[idx].target_step );
				motor_stop( motor_id );
				m_run_ctrl[idx].run_state = MOTOR_STATE_STOP;
			}
		}

		// 限位停机判定
		if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_FORWARD )
		{
			if ( motor_get_cur_step( motor_id ) >= (m_motorctrl[idx].forward_limit-10) )
			{
				xil_printf("M%d: 达到前限位状态！cur_step: %d, limit: %d\r\n", motor_id, motor_get_cur_step(motor_id), m_motorctrl[idx].forward_limit);
				motor_stop( motor_id );
				m_run_ctrl[idx].run_state = MOTOR_STATE_STOP;
			}
		}
		else if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_BACKWARD )
		{
			if ( motor_get_cur_step( motor_id ) <= (m_motorctrl[idx].backward_limit+10) )
			{
				xil_printf("M%d: 达到后限位状态！cur_step: %d, limit: %d\r\n", motor_id, motor_get_cur_step(motor_id), m_motorctrl[idx].backward_limit);
				motor_stop( motor_id );
				m_run_ctrl[idx].run_state = MOTOR_STATE_STOP;
			}
		}
		break;

	case MOTOR_STATE_STOP:
		motor_stop( motor_id );
		m_run_ctrl[idx].cur_dir = MOTOR_DIR_NONE;
		m_run_ctrl[idx].run_state = MOTOR_STATE_KEEPSTOP;
		break;

	case MOTOR_STATE_KEEPSTOP:
		break;

	case MOTOR_STATE_ROLLBACK_SPEEDDOWN:
		m_run_ctrl[idx].cur_speed -= MOTOR_EMER_ACC;
		motor_control( motor_id, m_run_ctrl[idx].cur_dir, m_run_ctrl[idx].cur_speed, m_run_ctrl[idx].run_step );

		if ( m_run_ctrl[idx].cur_speed < MOTOR_START_SPEED )
		{
			xil_printf("减速反转1：\r\n");
			m_run_ctrl[idx].run_state = MOTOR_STATE_ROLLBACK_STOP;
		}

		if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_FORWARD )
		{
			if ( motor_get_cur_step( motor_id ) >= (m_motorctrl[idx].forward_limit-10) )
			{
				xil_printf("减速反转4：\r\n");
				motor_stop( motor_id );
				m_run_ctrl[idx].run_state = MOTOR_STATE_STOP;
			}
		}
		else if ( m_run_ctrl[idx].cur_dir == MOTOR_DIR_BACKWARD )
		{
			if ( motor_get_cur_step( motor_id ) <= (m_motorctrl[idx].backward_limit+10) )
			{
				xil_printf("减速反转5：\r\n");
				motor_stop( motor_id );
				m_run_ctrl[idx].run_state = MOTOR_STATE_STOP;
			}
		}
		break;

	case MOTOR_STATE_ROLLBACK_STOP:
		m_run_ctrl[idx].cur_dir = m_run_ctrl[idx].set_dir;
		m_run_ctrl[idx].cur_speed = MOTOR_START_SPEED;
		m_run_ctrl[idx].start_step = motor_get_cur_step( motor_id );
		m_run_ctrl[idx].cur_step = motor_get_cur_step( motor_id );
		m_run_ctrl[idx].run_count = 0;
		motor_stop( motor_id );
		m_run_ctrl[idx].cur_time = timer_get_1us();
		m_run_ctrl[idx].run_state = MOTOR_STATE_ROLLBACK_WAIT;
		break;

	case MOTOR_STATE_ROLLBACK_WAIT:
		if ( timer_get_diff_1us( m_run_ctrl[idx].cur_time, timer_get_1us() ) >= 100000 )
		{
			xil_printf("开始反转：%d\r\n", m_run_ctrl[idx].run_state );
			m_run_ctrl[idx].run_state = MOTOR_STATE_SPEEDUP;
		}
		break;

	case MOTOR_STATE_ROLLBACK_SPEEDUP:
		break;

	default:
		break;
	}
}
void motor_curve_abort_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        m_run_ctrl[i].run_state = MOTOR_STATE_KEEPSTOP;
        m_run_ctrl[i].cur_dir   = MOTOR_DIR_NONE;
        m_run_ctrl[i].set_dir   = MOTOR_DIR_NONE;
        m_run_ctrl[i].cur_speed = 0;
        m_run_ctrl[i].stop_flag = 0;
    }
}
void motor_curve_abort_motor( MOTOR_ADDR motor_id )
{
    if ( motor_id < 1 || motor_id > MOTOR_COUNT ) return;
    int idx = motor_id - 1;
    m_run_ctrl[idx].run_state = MOTOR_STATE_KEEPSTOP;
    m_run_ctrl[idx].cur_dir   = MOTOR_DIR_NONE;
    m_run_ctrl[idx].set_dir   = MOTOR_DIR_NONE;
    m_run_ctrl[idx].cur_speed = 0;
    m_run_ctrl[idx].stop_flag = 0;
}
