/*
 * motorctrl.h
 *
 *  Created on: 2025脛锚7脭脗22脠脮
 *      Author: jczxy
 */

#ifndef SRC_MOTORCTRL_H_
#define SRC_MOTORCTRL_H_
#include "drv_motor.h"
#include "hal_motor.h"
#include <math.h>
#include "version.h"
#include "drv_pl.h"


typedef struct
{
  uint8_t flag;     // 鹿茅脰脨卤锚脰戮
  uint32_t count;   // 鹿茅脰脨录脝脢媒
} center_ctrl_t;



typedef struct {
	int32_t forward;
	int32_t backward;
}	limit_t;


typedef struct {
	uint16_t div;
	uint8_t current;
	int32_t forward_limit;
	int32_t backward_limit;
	int32_t center_step;
}	motor_ctrl_t;


extern motor_ctrl_t m_motorctrl[MOTOR_COUNT];

// 寰呮満鐢垫祦绠＄悊鐩稿叧瀹氫箟
#define STANDBY_CURRENT            20       // 寰呮満鐢垫祦鍊� (0-31)
#define STANDBY_TIMEOUT_MS         30000   // 鏃犺繍鍔ㄤ换鍔¤秴鏃舵椂闂� (30绉�)
#define STANDBY_CHECK_INTERVAL_MS  1000    // 寰呮満鐘舵�佹鏌ラ棿闅� (1绉�)

typedef enum {
    MOTOR_MODE_ACTIVE = 0,    // 杩愬姩妯″紡: 姝ｅ父鐢垫祦
    MOTOR_MODE_STANDBY        // 寰呮満妯″紡: 浣庣數娴�
} motor_mode_e;

typedef struct {
    motor_mode_e mode;                 // 褰撳墠妯″紡
    uint8_t normal_current;            // 姝ｅ父鐢垫祦鍊�
    uint64_t last_move_time;           // 鏈�鍚庤繍鍔ㄦ椂闂�(寰)
    uint8_t standby_enabled;           // 寰呮満鍔熻兘浣胯兘
} motor_standby_t;
typedef struct {
	uint8_t start_holl;
	uint8_t center_flag;
	uint8_t stop_check;
	uint8_t stop_done;
	motor_dir_e hollon_dir;
	motor_dir_e holloff_dir;
}	center_t;
/******************************************************************************************/
/******************************************************************************************/

void motorctrl_init( void );

void motorctrl_process( void );

uint8_t motor_center_done_check( void );

void motor_move_step( MOTOR_ADDR addr, uint8_t dir, uint32_t step );

void motor_move_coord( MOTOR_ADDR addr, int32_t coord );

void motor_move_speed_step( MOTOR_ADDR addr, uint8_t dir, uint32_t speed, int32_t step );

void motorctrl_limit_process( void );
void motor_limit_sync_coords(MOTOR_ADDR addr);
// 寰呮満鐢垫祦绠＄悊鍑芥暟
void motor_standby_init( void );
void motor_standby_process( void );
//void motor_set_active_mode( MOTOR_ADDR addr );
//void motor_set_standby_mode( MOTOR_ADDR addr );
void motor_record_move_time( MOTOR_ADDR addr );
void motor_low_speed_run(MOTOR_ADDR addr, uint8_t dir, uint32_t speed);
uint32_t step_limit_check(MOTOR_ADDR addr, int32_t step);
uint32_t coord_limit_check(MOTOR_ADDR addr, int32_t coord);
uint32_t move_limit_check(MOTOR_ADDR addr, motor_dir_e dir, uint32_t step);
void motor_set_standby_mode_all(void);
void Motor_Follow_Target_Process(MOTOR_ADDR addr);
extern center_t m_center[MOTOR_COUNT];
extern uint8_t g_m2_low_speed_run_flag;
extern uint32_t g_m2_run_speed;
#endif /* SRC_MOTORCTRL_H_ */
