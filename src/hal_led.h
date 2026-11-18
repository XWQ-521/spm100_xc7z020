/*
 * hal_led.h
 *
 *  Created on: 2025年10月25日
 *      Author: dulc
 */

#ifndef SRC_HAL_HAL_LED_H_
#define SRC_HAL_HAL_LED_H_

typedef enum{
    CMD_ALIGN_RIGHT = 0x21,     // 右对位
    CMD_ALIGN_LEFT,             // 左对位
    CMD_CENTER,                 // 中心灯
    CMD_FOCUS,                  // 对焦灯
    CMD_SLIT,                   // 裂隙灯
    CMD_FIXATION,               // 固视灯
    CMD_TRIGGER,                // 触发拍照
    CMD_GET_CCT,                // 获取CCT角膜厚度
    CMD_FILL_LIGHT,             // 补光灯
    CMD_MICRO_FOCUS,            // 精对焦
    CMD_XYZ_POS,                // XYZ位置差
    CMD_ALIGN_ALL,              // 全对位灯
    CMD_FILL_LIGHT_ALL,         // 所有补光灯
    CMD_XYZ_PARAM,              // XYZ对焦参数
	CMD_LOW_POWER_CTRL = 0x60,
	CMD_CAT_SET = 0x61, 		//白内障设置
	CMD_CENTER_BACK = 0x62,		//归中设置
	CMD_PACK = 0x63,			//打包设置
	CMD_SLR = 0x64,				//左右眼切换
	CMD_MOTOR2_SPEED = 0x66,	//左右电机2速度设置

	CMD_IAP_UPDATE = 0x70,
	CMD_Close_CMOS = 0x71,
	CMD_RL_Place = 0x72,
	CMD_Test = 0x73,
} CMD_T;

void led_control(CMD_T cmd, uint8_t id, uint8_t value);
void led_align_all_control(uint8_t value);
void led_fill_light_all_control(uint8_t value);

#endif /* SRC_HAL_HAL_LED_H_ */
