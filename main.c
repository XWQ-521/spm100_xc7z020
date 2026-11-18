/*
 * Copyright (C) 2009 - 2018 Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */
/*
1. 锟斤拷始锟斤拷硬锟斤拷平台锟斤拷lwIP协锟斤拷栈锟斤拷
2. 锟斤拷锟斤拷锟斤拷锟斤拷涌冢锟斤拷锟斤拷锟組AC锟斤拷址锟斤拷IP锟斤拷址锟斤拷支锟斤拷DHCP锟斤拷态IP锟斤拷锟斤拷
3. 锟斤拷锟斤拷TCP锟斤拷锟斤拷锟斤拷应锟矫筹拷锟津（碉拷锟斤拷start_application锟斤拷锟斤拷
4. 锟斤拷锟斤拷锟斤拷循锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷帮拷锟酵讹拷时锟斤拷锟斤拷
*/
#include "COMMON.h"
#include <stdio.h>

#include "pid.h"
#include "protocol.h"
#include "platform.h"
#include "motorctrl.h"
#include "hal_timer.h"
#include "test.h"
#include "taskevent.h"
#include "tcp_server.h"
#include "hal_motor.h"
#include "hal_cmos.h"
#include "hal_param.h"
#include "joystick.h"
#include "motor_curve.h"
#include "param.h"
#include "UPgrade.h"
#include "version.h"
#include "drv_flash.h"
#include "motor_stall.h"
#include "x_axis_ctrl.h"
#include "cmd_shell.h"
#include "bglamp_drv.h"
#include "joystick_iap.h"
#include "joystick_upgrade.h"
//                          x    y   z1锟斤拷位/锟斤拷位  z2锟斤拷位/锟斤拷位
uint8_t m_xyz_param[6] = { 98, 160,     0, 200,       1, 15 };



void pid_control_init(void);    // PID锟斤拷锟狡筹拷始锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
void process_motor_control_loops(void);
void motor_center_while( void );
void xyz_report_event_process_1( void )
{


	static uint64_t pre_time = 0;

    // 锟斤拷锟斤拷欠锟斤到预锟斤拷锟绞憋拷锟斤拷锟斤拷锟斤拷锟斤拷未锟斤到锟斤拷直锟接凤拷锟斤拷
    if ( timer_get_diff_1us( pre_time, timer_get_1us() ) < 100*1000 )
        return;

    // 锟斤拷锟斤拷锟斤拷一锟斤拷状态锟侥憋拷锟绞憋拷锟轿拷锟角笆憋拷锟�
    pre_time = timer_get_1us();
    xil_printf("pre_time = %d\r\n",(int)pre_time);
    //xyz_report();
}
void joystick_check_print(void)
{
    if (g_joystick_data_updated == 1)
    {
        static uint64_t last_print_time = 0;
        uint64_t current_time = timer_get_1us(); // 获取当前时间[cite: 1, 3]

        // 每 200ms 最多打印一次，防止频繁打印卡死总线
        if (current_time - last_print_time >= 200000)
        {
            last_print_time = current_time;

            // 【修改后的打印】：加上了 X_DIR, Y_DIR 和 FLAG 状态
            xil_printf("[MAIN_JOY] X:%d | Y:%d | Z:%d | Buttons:0x%04X | X_DIR:%d | Y_DIR:%d | FLAG:%d\r\n",
                       g_current_joystick_data.x,
                       g_current_joystick_data.y,
                       g_current_joystick_data.z,
                       g_current_joystick_data.buttons,
                       g_current_joystick_data.x_dir,
                       g_current_joystick_data.y_dir,
                       g_joystick_active_flag);
        }

        g_joystick_data_updated = 0; // 清除更新标志，等待下一帧
    }
}


int main()
{
    //==========================================================================================//
    //PSD锟斤拷锟斤拷锟姐法锟斤拷始
    //==========================================================================================//



#if Mater_Plate
	Flash_init();
	check_and_boot_area2();
#endif
	//Xil_Out8(0x1000002C, 0x01);
	//Xil_Out8(WRITE_ADDR_SLIT_PARAM, 0xff);

    //sleep(3);
	//Xil_Out8(WRITE_ADDR_SLIT_PARAM, 0x1);
	Xil_Out8(0x1000002C, 0x00);
	Xil_Out8(0x1000002D, 0x00);
	Xil_Out32(0x10000030, 0x17D7840);
    Xil_DCacheDisable();

    motor_ctrlreg_init();

    timer_init();

    param_init();

    //==========================================================================================//
    // TCP锟斤拷始锟斤拷锟斤拷始
    //==========================================================================================//
    protocol_init();

//    tcp_server_init();


    //==========================================================================================//
    //PID锟斤拷始锟斤拷锟斤拷始
    //==========================================================================================//
    pid_control_init();

    // 锟斤拷时1锟斤拷却锟斤拷锟斤拷锟斤拷锟斤拷芯片锟饺讹拷
    timer_delay_1us(1000000);

    //==========================================================================================//
    //==========================================================================================//
    motorctrl_init();

#if 1
    motor_center_while();

    cmos_expose_set( 10 );
	
	param_psd_expose_set( 20 );
	
    
    param_xyz_focus_coord( PARAM_COORD_X, 5 );
    param_xyz_focus_coord( PARAM_COORD_Y, 5 );
    param_xyz_focus_coord( PARAM_COORD_Z, 8 );

    xyz_param_set(m_xyz_param);

//  param_focus_set( SPEED_D1, 100 );
//	param_focus_set( DISTANCE_D1, 1000 );
//	param_focus_set( SPEED_D2, 100 );
//	param_focus_set( DISTANCE_D2, 500 );
//	param_focus_set( SPEED_D3, 100 );
//	param_focus_set( DISTANCE_D3, 228 );
//	param_focus_set( SPEED_D4, 100 );
//	param_focus_set( DISTANCE_D4, 347 );
    param_slit_set( 99 );
#endif
//    xyz_param_set( m_xyz_param );
    //==========================================================================================//
    //PID锟斤拷始锟斤拷锟斤拷锟斤拷,锟斤拷位锟斤拷锟截的筹拷锟斤拷锟斤拷锟斤拷3
    //==========================================================================================//

    tcp_server_init();

    usleep(10000);
    Xil_Out8(0x1000002C, 0x01);
    joystick_init();
    Joystick_Filter_Flush();

    // 锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    //motor_standby_init();

    //xil_printf("WHILE1_锟斤拷始 TEST!  %lld...\n\r", timer_get_1us() );
    joystick_iap_cmd_register();
    bglamp_drv_init();
    motor_curve_cmd_register();
    cmd_shell_init();

    // 锟斤拷循锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷帮拷锟酵讹拷时锟斤拷
    while ( 1 )
    {
    	cmd_shell_process();
    	joystick_process();
    	Joy_Stress_Test_Poll();
    	Joy_Stall_Blackbox_Poll();
    	// 锟斤拷锟斤拷锟轿伙拷锟斤拷锟�
    	motorctrl_limit_process();
    	// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    	//motor_standby_process();
        // PID锟斤拷锟狡诧拷锟斤拷 ***********************************************************************************************************************************************************************
        process_motor_control_loops();

        // TCP锟斤拷锟界处锟斤拷锟斤拷锟斤拷 ***********************************************************************************************************************************************************************
        tcp_server_tasks();

        test_process();

        taskevent_process();

        motor_curve_process();


        for(int i = 1; i <= MOTOR_COUNT; i++) {
                    motor_limit_sync_coords(i);
                }
        check_joystick_hijack();
        Motor_Center_Main_Process();
        // 3. 锟斤拷锟斤拷锟侥讹拷锟斤拷锟斤拷位锟斤拷锟狡讹拷 500ms 锟斤拷台锟斤拷询
        Motor_Pack_Move_Main_Process();
        Motor_Seq_Move_Main_Process();
        check_and_report_motor_status();

            joystick_iap_poll();
            joystick_upgrade_process();

        //joystick_check_print();	//摇杆检查
        //motor_stall_task_process();//电机堵转上报

        //xyz_report_event_process_1();
        //==========================================================================================//
    }

    /* never reached */
        // 锟斤拷远锟斤拷锟斤拷执锟叫碉拷锟斤拷锟斤（锟斤拷循锟斤拷锟斤拷锟斤拷锟斤拷为锟斤拷锟斤拷锟皆帮拷锟斤拷锟斤拷锟斤拷
    cleanup_platform();
    return 0;
}



void motor_center_while( void )
{
#define MOTOR_CENTER_OVERTIME		20000000
    static uint64_t start_time;
    start_time = timer_get_1us();
    while( 1 )
    {
        // 锟斤拷锟斤拷锟斤拷写锟斤拷锟斤拷锟街恢达拷锟�1锟斤拷
        motorctrl_process();

        // 锟斤拷时15锟斤拷
        if ( timer_get_diff_1us( start_time, timer_get_1us() ) >= MOTOR_CENTER_OVERTIME )
        {
        	xil_printf("锟斤拷锟叫筹拷时锟斤拷锟斤拷\r\n");
        	break;
        }

        if ( motor_center_done_check() )
        {
        	xil_printf("锟斤拷锟斤拷锟斤拷桑锟斤拷锟絓r\n");
            break;
        }
    }
}

