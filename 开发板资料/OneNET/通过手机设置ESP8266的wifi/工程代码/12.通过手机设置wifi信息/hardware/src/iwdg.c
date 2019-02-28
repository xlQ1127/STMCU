#include "stm32f10x.h"

#include "iwdg.h"






//固定40KHz时钟
void Iwdg_Init(unsigned char psc, unsigned short arr)
{

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对寄存器IWDG_PR和IWDG_RLR的写操作
	
	IWDG_SetPrescaler(psc); //设置IWDG预分频值:设置IWDG预分频值为64
	//0		4分频
	//1		8分频
	//2		16分频
	//3		32分频
	//4		64分频
	//5		128分频
	//6		256分频
	//7		256分频
	
	IWDG_SetReload(arr); //设置IWDG重装载值		每次喂狗后从重载值开始递减
	
	IWDG_ReloadCounter(); //按照IWDG重装载寄存器的值重装载IWDG计数器
	
	IWDG_Enable(); //使能IWDG

}

//喂独立看门狗
void Iwdg_Feed(void)
{
	
 	IWDG_ReloadCounter();
	
}
