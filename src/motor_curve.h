/*
 * motor_curve.h
 *
 *  Created on: 2025Äê11ÔÂ23ÈÕ
 *      Author: dulc
 */

#ifndef SRC_MOTOR_CURVE_H_
#define SRC_MOTOR_CURVE_H_


void motor_curve_process( void );

void motor_motion_start( MOTOR_ADDR motor_id, motor_dir_e dir, uint32_t step, uint32_t speed );
void motor_curve_cmd_register( void );
void motor_curve_abort_all(void);
void motor_curve_abort_motor( MOTOR_ADDR motor_id );
#endif /* SRC_MOTOR_CURVE_H_ */
