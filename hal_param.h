/*
 * hal_param.h
 *
 *  Created on: 2025年11月5日
 *      Author: dulc
 */

#ifndef SRC_HAL_PARAM_H_
#define SRC_HAL_PARAM_H_
#include "stdint.h"

typedef enum {
	SPEED_D1 = 0x01,
	DISTANCE_D1 = 0x02,
	SPEED_D2 = 0x04,
	DISTANCE_D2 = 0x08,
	SPEED_D3 = 0x10,
	DISTANCE_D3 = 0x20,
	SPEED_D4 = 0x40,
	DISTANCE_D4 = 0x80,
}	param_cmd_e;


typedef enum {
	PARAM_COORD_X = 1,
	PARAM_COORD_Y,
	PARAM_COORD_Z,
}	coord_xyz_e;



void param_focus_set( param_cmd_e cmd, uint32_t value );

void param_slit_set( uint8_t value );
// XYZ 允差范围测试
void param_xyz_focus_coord( coord_xyz_e xyz, uint8_t value );
// PSD 曝光时间配置
void param_psd_expose_set( uint16_t value );

void param_slit_pwm_set( uint8_t * data );

#endif /* SRC_HAL_PARAM_H_ */
