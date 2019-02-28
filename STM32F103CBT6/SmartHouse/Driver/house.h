#ifndef _HOUSE_H_
#define _HOUSE_H_

#include <stdint.h>

#define  BEE_OFF       GPIO_SetBits(GPIOA,GPIO_Pin_8)
#define  BEE_ON        GPIO_ResetBits(GPIOA,GPIO_Pin_8)

typedef struct
{
	uint8_t Safe_Mode;
	uint8_t Door_Status;
	uint8_t Light_Status;
	uint8_t Fan_Status;
	uint8_t Active_Alarm;
	uint8_t Guest_Alarm;
	uint8_t Fire_Alarm;
} HOUSE_INFO;

extern HOUSE_INFO houseInfo;

void House_Init(void);
void Status_Scan(void);

#endif
