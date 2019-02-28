#ifndef _LED_H_
#define _LED_H_





typedef struct
{

	unsigned char Led2Sta;
	unsigned char Led3Sta;
	unsigned char Led4Sta;
	unsigned char Led5Sta;

} LED_STATUS;

extern LED_STATUS led_status;


void Led_Init(void);

void Led2_Set(unsigned char pwmValue);

void Led3_Set(unsigned char pwmValue);

void Led4_Set(unsigned char pwmValue);

void Led5_Set(unsigned char pwmValue);


#endif
