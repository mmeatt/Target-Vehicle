#include "wfly_comm.h"
SBUS_Buffer SBUS;


void sbus_callback_handler(SBUS_Buffer *SBUS, uint8_t *SBUS_RXBuffer)
{
		SBUS->Start=SBUS_RXBuffer[0];
		SBUS->Ch1=(((uint16_t)SBUS_RXBuffer[1])|((uint16_t)((SBUS_RXBuffer[2]&0x07)<<8)))-1024;
		SBUS->Ch2=1024-(((uint16_t)((SBUS_RXBuffer[2]&0xf8)>>3))|(((uint16_t)(SBUS_RXBuffer[3]&0x3f))<<6)/2);
		SBUS->Ch3=(((uint16_t)((SBUS_RXBuffer[3]&0xc0)>>6))|((((uint16_t)SBUS_RXBuffer[4])<<2))|(((uint16_t)(SBUS_RXBuffer[5]&0x01))<<10))-1024;
		SBUS->Ch4=(((uint16_t)((SBUS_RXBuffer[5]&0xfe)>>1))|(((uint16_t)(SBUS_RXBuffer[6]&0x0f))<<7))-1024;
		SBUS->sw1=((uint16_t)((SBUS_RXBuffer[6]&0xf0)>>4))|(((uint16_t)(SBUS_RXBuffer[7]&0x7f))<<4);
		SBUS->sw2=((uint16_t)((SBUS_RXBuffer[7]&0x80)>>7))|(((uint16_t)SBUS_RXBuffer[8])<<1)|(((uint16_t)(SBUS_RXBuffer[9]&0x03))<<9);
		SBUS->sw3=((uint16_t)((SBUS_RXBuffer[9]&0xfc)>>2))|(((uint16_t)(SBUS_RXBuffer[10]&0x1f))<<6);
		SBUS->sw4=((uint16_t)((SBUS_RXBuffer[10]&0xe0)>>5))|(((uint16_t)(SBUS_RXBuffer[11]))<<3);
		SBUS->Ch9=((uint16_t)SBUS_RXBuffer[12])|(((uint16_t)(SBUS_RXBuffer[13]&0x07))<<8);
		SBUS->Ch10=((uint16_t)((SBUS_RXBuffer[13]&0xf8)>>3))|(((uint16_t)(SBUS_RXBuffer[14]&0x3f))<<5);
		SBUS->Ch11=((uint16_t)((SBUS_RXBuffer[14]&0xc0)>>6))|(((uint16_t)SBUS_RXBuffer[15])<<2)|(((uint16_t)(SBUS_RXBuffer[16]&0x01))<<10);
		SBUS->Ch12=((uint16_t)((SBUS_RXBuffer[16]&0xfe)>>1))|(((uint16_t)(SBUS_RXBuffer[17]&0x0f))<<7);
		SBUS->Ch13=((uint16_t)((SBUS_RXBuffer[17]&0xf0)>>4))|(((uint16_t)(SBUS_RXBuffer[18]&0x7f))<<4);
		SBUS->Ch14=((uint16_t)((SBUS_RXBuffer[18]&0x80)>>7))|(((uint16_t)SBUS_RXBuffer[19])<<1)|(((uint16_t)(SBUS_RXBuffer[20]&0x03))<<9);
		SBUS->Ch15=((uint16_t)((SBUS_RXBuffer[20]&0xfc)>>2))|(((uint16_t)(SBUS_RXBuffer[21]&0x1f))<<6);
		SBUS->Ch16=((uint16_t)((SBUS_RXBuffer[21]&0xe0)>>5))|(((uint16_t)SBUS_RXBuffer[22])<<3);
		SBUS->Flag=SBUS_RXBuffer[23];
		SBUS->End=SBUS_RXBuffer[24];

		
	
	
}






