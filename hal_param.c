/*
 * hal_param.c
 *
 *  Created on: 2025骞�11鏈�5鏃�
 *      Author: dulc
 */

#include "hal_param.h"
#include "drv_pl.h"
#include "xil_printf.h"
#include "version.h"








void param_focus_set( param_cmd_e cmd, uint32_t value )
{
	uint32_t tmp_value = value;
    uint32_t write_val = 0;

    switch( cmd )
    {
    case SPEED_D1:
    case SPEED_D2:
    case SPEED_D3:
    case SPEED_D4:
    	tmp_value = value * 1000 / 625;
    	break;
    case DISTANCE_D1:
    case DISTANCE_D2:
    case DISTANCE_D3:
    case DISTANCE_D4:
    	tmp_value = value * 10 / 25;
    	break;
    }

    // 鹿鹿脭矛32脦禄驴脴脰脝脢媒戮脻拢卢掳麓脦禄麓貌掳眉
    write_val += cmd;
    write_val = write_val << 8;
    write_val += (uint8_t)(tmp_value>>16);
    write_val = write_val << 8;
    write_val += (uint8_t)(tmp_value>>8);
    write_val = write_val << 8;
    write_val += (uint8_t)(tmp_value);

    xil_printf("write_val: 0x%08X   \r\n", write_val );
    pl_write_u32(WRITE_ADDR_FOCUS_PARAM, write_val);
}



void param_slit_set( uint8_t value )
{
    pl_write_u8(WRITE_ADDR_SLIT_PARAM, value );
}


// XYZ 脭脢虏卯路露脦搂虏芒脢脭
void param_xyz_focus_coord( coord_xyz_e xyz, uint8_t value )
{
	uint8_t write_value = 0;

	if ( xyz > PARAM_COORD_Z )
		return;

	write_value = ( 1 << (xyz+4)) + (value & 0x1F);

	pl_write_u8( WRITE_ADDR_COORD_XYZ, write_value );

}



void param_psd_expose_set( uint16_t value )
{
	pl_write_u16( WRITE_ADDR_PSD_EXPO, value );
}



void param_slit_pwm_set( uint8_t * data )
{
	uint8_t write_value = 0;

	// 脜盲脰脙脕脩脧露碌脝脮没脤氓脰脺脝脷
	write_value = 0x40 | ( data[0] & 0x3F);
	pl_write_u8( WRITE_ADDR_SLIT_PWM, write_value );
	// 脩脫脢卤碌脠麓媒1000赂枚脰脺脝脷
	for( int i = 0; i < 1000; i++ );
	// 脜盲脰脙脕脩脧露碌脝脕脕碌脛脢卤录盲
	write_value = 0x80 | ( data[1] & 0x3F);
	pl_write_u8( WRITE_ADDR_SLIT_PWM, write_value );
}


