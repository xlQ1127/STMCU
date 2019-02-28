#ifndef _KEY_H_
#define _KEY_H_







#define KEY_INT		0 //是否使能外部中断

#define KEY0			GPIO_Pin_11
#define KEY1			GPIO_Pin_13
#define KEY2			GPIO_Pin_12
#define KEY3			GPIO_Pin_2

#define KEYPORTNUM1		(GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4)
#define KEYPORTNUM2		GPIO_Pin_0


/*******************************************
			按键按下与弹起
*******************************************/
#define KEYDOWN		1
#define KEYUP		0

#define KEY0DOWN	0
#define KEY0UP		1
#define KEY0DOWNREPEAT	10

#define KEY1DOWN	2
#define KEY1UP		3
#define KEY1DOWNREPEAT	11

#define KEY2DOWN	4
#define KEY2UP		5
#define KEY2DOWNREPEAT	12

#define KEY3DOWN	6
#define KEY3UP		7
#define KEY3DOWNREPEAT	13

#define KEYNONE		255


extern void Key_Init(void);

extern unsigned char Keyboard(void);

#endif
