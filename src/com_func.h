/*
 * com_func.h
 *
 *  Created on: 2025Äê10ÔÂ26ÈÕ
 *      Author: dulc
 */

#ifndef SRC_COMMON_COM_FUNC_H_
#define SRC_COMMON_COM_FUNC_H_
#include "stdio.h"

uint16_t gen_crc(uint8_t *buf, uint16_t data_len);

int validate_packet(uint8_t *data, uint32_t len);

#endif /* SRC_COMMON_COM_FUNC_H_ */
