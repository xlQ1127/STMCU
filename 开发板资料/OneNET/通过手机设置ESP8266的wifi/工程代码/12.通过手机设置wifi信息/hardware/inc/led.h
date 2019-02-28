#ifndef _LED_H_
#define _LED_H_


//RED_LED		PC7
//GREEN_LED		PC8
//YELLOW_LED	PC10
//BLUE_LED		PA12

typedef struct
{

	_Bool LedYellowSta;
	_Bool LedBlueSta;
	
	unsigned int LedRedSta;
	unsigned int LedGreenSta;

} LED_STATUS;

extern LED_STATUS ledStatus;

#define LED_ON		1
#define LED_OFF		0








void Led_Init(void);

void Led_Red_Set(_Bool status);

_Bool Led_Red_Get(void);

void Led_Green_Set(_Bool status);

_Bool Led_Green_Get(void);

void Led_Yellow_Set(_Bool status);

_Bool Led_Yellow_Get(void);

void Led_Blue_Set(_Bool status);

_Bool Led_Blue_Get(void);


#endif
