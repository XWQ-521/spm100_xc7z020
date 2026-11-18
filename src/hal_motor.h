/*
 * hal_motor.h
 *
 *  Created on:
 *      Author: dulc
 */

#ifndef SRC_HAL_HAL_MOTOR_H_
#define SRC_HAL_HAL_MOTOR_H_
#include "stdio.h"
#include "drv_motor.h"
#include "version.h"


typedef enum {
    MOTOR_DIR_NONE = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD
}   motor_dir_e;




typedef struct {
    motor_dir_e dir;
    int32_t center_step;
}   motor_info_t;



extern motor_info_t m_motor[MOTOR_COUNT];




int motor_device_init( MOTOR_ADDR addr, uint16_t sub_div, uint8_t current );

int8_t motor_get_limit_center(MOTOR_ADDR addr);

void motor_ctrlreg_init( void );

void motor_control(MOTOR_ADDR addr, uint8_t dir, uint32_t speed, uint32_t step);

void motor_stop( MOTOR_ADDR addr );



void motor_center_step_init(MOTOR_ADDR addr, int32_t offset);

int32_t motor_get_cur_step(MOTOR_ADDR addr);

int32_t motor_get_real_encode(MOTOR_ADDR addr);

uint32_t motor_get_raw_encode(MOTOR_ADDR addr);

void  motor_limit_update_to_pl( MOTOR_ADDR addr, uint8_t forward, uint8_t backward );
void Motor_Read_DRVSTATUS(MOTOR_ADDR motor_id, uint16_t *out_sg_val, uint8_t *out_sg_flag, uint8_t *out_stst);


#endif /* SRC_HAL_HAL_MOTOR_H_ */
