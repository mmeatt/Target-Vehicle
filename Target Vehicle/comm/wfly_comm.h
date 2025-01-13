#ifndef _WFLY_COMM_H_
#define _WFLY_COMM_H_
#include "stdint.h"

#define StartByte 0x0f
#define EndByte	  0x00

typedef enum{
	SBUS_UP = 353,
	SBUS_MI = 1024,
	SBUS_DN = 1694,
}sbus_sw_e;


typedef struct{
	uint8_t Start;
	int16_t Ch1;
	int16_t Ch2;
	int16_t Ch3;
	int16_t Ch4;
	uint16_t sw1;
	uint16_t sw2;
	uint16_t sw3;
	uint16_t sw4;
	uint16_t Ch9;
	uint16_t Ch10;
	uint16_t Ch11;
	uint16_t Ch12;
	uint16_t Ch13;
	uint16_t Ch14;
	uint16_t Ch15;
	uint16_t Ch16;
	uint8_t Flag;
	uint8_t End;
}SBUS_Buffer;

extern SBUS_Buffer SBUS;;
void sbus_callback_handler(SBUS_Buffer *SBUS, uint8_t *SBUS_RXBuffer);
#endif
