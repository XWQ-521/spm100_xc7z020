/*
 * hal_cmos.h
 *
 *  Created on: 2025Äê10ÔÂ25ÈÕ
 *      Author: dulc
 */

#ifndef SRC_HAL_HAL_CMOS_H_
#define SRC_HAL_HAL_CMOS_H_
#include <stdio.h>



typedef struct {
    uint16_t x1_val;
    uint16_t y1_val;
    uint16_t x2_val;
    uint16_t y2_val;
    uint16_t z1_pos;
    uint16_t z1_val;
    uint16_t z2_pos;
    uint16_t z2_val;
}   detector_t;



detector_t get_detector_data( void );

void xyz_param_set( uint8_t *data );

void cmos_expose_set( uint16_t time );

int cmos_get_z_data( uint32_t* read_buf );

int cmos_get_xy_data( uint32_t* read_buf );

#endif /* SRC_HAL_HAL_CMOS_H_ */
