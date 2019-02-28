#ifndef __RTC_H__
#define __RTC_H__

#include "stm32f10x.h"
#include "SysTick.h"

//时间结构体
typedef struct 
{
	u8 hour;
	u8 min;
	u8 sec;			
	//公历日月年周
	u16 w_year;
	u8  w_month;
	u8  w_date;
	u8  week;		 
}tm;					 
extern tm timer; 
extern u8 const mon_table[12];//月份日期数据表
extern u16 CNTL;

u8 RTC_Init(u16 key);//初始化RTC,返回0,失败;1,成功;
u8 Is_Leap_Year(u16 year);//平年,闰年判断
u8 RTC_Get(void);//更新时间
u8 RTC_Get_Week(u16 year,u8 month,u8 day);
u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 shour,u8 smin,u8 ssec);//设置时间
void Auto_Time_Set(void);//设置时间为编译时间


#endif


