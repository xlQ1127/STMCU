#include "rtc.h"

tm timer;//时钟结构体
u16 CNTL;

//月份   1  2  3  4  5  6  7  8  9  10 11 12
//闰年   31 29 31 30 31 30 31 31 30 31 30 31
//非闰年 31 28 31 30 31 30 31 31 30 31 30 31


/*
 * 函数名：RTC_NVIC_Config
 * 描述  ：RTC中断配置 抢占3 响应3
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
void RTC_NVIC_Config(void)
{	
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;		//RTC全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//使能该通道中断
	NVIC_Init(&NVIC_InitStructure);		//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}

/*
 * 函数名：Is_Leap_Year
 * 描述  ：判断是否是闰年函数
 * 输入  ：year 年份
 * 输出  ：该年份是不是闰年 1,是 0,不是
 * 调用  ：内部调用
 */
u8 Is_Leap_Year(u16 year)//(能被4整除却不能被100整除)或者(能被400整除)的年份
{
	if((year%4==0 && year%100!=0)||(year%400==0))
		return 1;
	else
		return 0;	
}

/*
 * 函数名：RTC_Set
 * 描述  ：设置时钟 以1970年1月1日为基准,1970~2099年为合法年份
 * 输入  ：年 月 日 时 分 秒
 * 输出  ：0,成功;其他:错误代码
 * 调用  ：外部调用
 */
u8 const mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};//平年的月份日期表
u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 shour,u8 smin,u8 ssec)
{
	u16 i;
	u32 seccount=0;
	if(syear<1970||syear>2099) return 1;
	for(i=1970;i<syear;i++)
	{
		if(Is_Leap_Year(i)) seccount += 31622400; //闰年的秒钟数
		else seccount += 31536000;  //平年的秒钟数
	}
	smon--;
	for(i=0;i<smon;i++)
	{
		if(Is_Leap_Year(syear)&&i == 1) seccount += 86400;//闰年2月份增加一天的秒钟数
		seccount += (u32)mon_table[i]*86400;//月份秒钟数相加		
	}
	seccount += (u32)(sday-1)*86400;//日期秒钟数相加
	seccount += (u32)(shour)*3600;//小时秒钟数相加	
	seccount += (u32)(smin)*60; //分钟秒钟数
	seccount += ssec;//最后的秒钟加上去

	//设置时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP,ENABLE);//使能电源时钟 使能备份时钟
	PWR_BackupAccessCmd(ENABLE);//使能RTC和后备寄存器访问
	
	RTC_WaitForLastTask();//等待最近一次对RTC寄存器的写操作完成	
	RTC_SetCounter(seccount);//设置RTC计数器的值
	RTC_WaitForLastTask();//等待最近一次对RTC寄存器的写操作完成
	return 0;
	
}

/*
 * 函数名：RTC_Get
 * 描述  ：得到当前的时间
 * 输入  ：无
 * 输出  ：0,成功;其他:错误代码.
 * 调用  ：外部调用
 */
u8 RTC_Get(void)
{
	static u16 daycnt=0;//只第赋值一次
	u32 timecount=0;
	u32 temp=0;
	u16 temp1=0;
	
	RTC_WaitForSynchro();//等待RTC_XXX寄存器同步
	timecount = RTC_GetCounter();//得到计数器中的值(秒钟数)
   	
	temp = timecount/86400;	 //得到天数(秒钟数对应的)
	
	if(daycnt!=temp)//超过一天了
	{
		daycnt = temp;
		temp1 = 1970; 	//从1970年开始
		while(temp>=365)
		{
			if(Is_Leap_Year(temp1))//是闰年
			{
				if(temp >=366) temp-=366;
				else break;
			}
			else temp -= 365;
			temp1++;
		}
		timer.w_year = temp1;//得到年份

		temp1 = 0;
		while(temp>=28)
		{
			if(Is_Leap_Year(timer.w_year)&&temp1 == 1)//闰年二月份
			{
				if(temp>=29) temp -= 29;
				else break;
			}
			else
			{
				if(temp>=mon_table[temp1]) temp -= mon_table[temp1];//平年
				else break;
			}
			temp1++;
		}
		timer.w_month = temp1+1;//得到月份
		timer.w_date = temp+1;//得到星期
	}
	temp = timecount%86400;//得到剩余一天秒钟数
	timer.hour = temp/3600;//小时
	timer.min = temp%3600/60;//分钟
	timer.sec = temp%3600%60;//秒钟
	timer.week=RTC_Get_Week(timer.w_year,timer.w_month,timer.w_date);//获取星期  
	return 0;
}



/*
 * 函数名：RTC_Get_Week
 * 描述  ：获得现在是星期几 1970~2099年为合法年份 利用“基姆拉尔森计算公式”
 * 输入  ：年 月 日
 * 输出  ：星期号
 * 调用  ：外部调用
 */																						 
u8 RTC_Get_Week(u16 year,u8 month,u8 day)
{
	int week;
	if (month == 1 || month == 2)
    {
    	month += 12;
        year--;
    }	
	week =(day+2*month+3*(month+1)/5+year+year/4-year/100+year/400)%7;
	return week;
}


/*
 * 函数名：RTC_Init
 * 描述  ：初始化RTC时钟,同时检测时钟是否工作正常
 * 输入  ：key 保存识别钥匙
 * 输出  ：0正常 1代码错误
 * 调用  ：外部调用
 */
u8 RTC_Init(u16 key)
{
	u8 temp = 0;
	RTC_NVIC_Config();
	if(BKP_ReadBackupRegister(BKP_DR1)!=key)//第一次配置
	{	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP,ENABLE);//使能电源时钟 使能备份时钟
		PWR_BackupAccessCmd(ENABLE);//使能RTC和后备寄存器访问
		
		BKP_DeInit();//将外设BKP的全部寄存器重设为缺省值
		RCC_LSEConfig(RCC_LSE_ON);	//设置外部低速晶振(LSE),使用外设低速晶振

		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) //等待外部时钟就绪	 
		{
			temp++;
			delay_ms(10);
			if(temp>=250) return 1;
		}

		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//设置LSE作为RTC时钟
		RCC_RTCCLKCmd(ENABLE);//开启RTC时钟
		
		//RTC_WaitForSynchro();//等待RTC_XXX寄存器同步
		RTC_WaitForLastTask();//等待最近一次对RTC寄存器的写操作完成
		RTC_ITConfig(RTC_IT_SEC,ENABLE);//允许秒中断
		RTC_WaitForLastTask();//等待最近一次对RTC寄存器的写操作完成
		RTC_SetPrescaler(32767);//设置RTC预分频的值
		RTC_WaitForLastTask();//等待最近一次对RTC寄存器的写操作完成
		
		//RTC_Set(2012,12,9,21,47,0);
		Auto_Time_Set();
		BKP_WriteBackupRegister(BKP_DR1,0x6060);//向指定的后备寄存器中写入用户程序数据		
	}
	else
	{
		RTC_ITConfig(RTC_IT_SEC,ENABLE);//允许秒中断
		RTC_WaitForLastTask();//等待最近一次对RTC寄存器的写操作完成	
	}
	RTC_Get();//更新时间
	return 0; //ok				
}



//RTC中断服务函数//
void RTC_IRQHandler(void)
{
	if(RTC->CRL&(1<<0))//秒中断
	{
		while(!(RTC->CRL&(1<<3)));//等待RTC_XXX寄存器同步
		CNTL = RTC->CNTL;
		RTC_Get();//更新时间		
	}
	if(RTC->CRL&(1<<1))//闹钟中断
	{
		/***闹钟中断代码***/			
	}
	if(RTC->CRL&(1<<2))//溢出中断
	{
		/***溢出中断代码***/		
	}
	RTC->CRL &= 0xFFF8;         //清除所有中断标标志 [2]溢出 [1]闹钟 [0]秒
	RTC_WaitForLastTask();//等待最近一次对RTC寄存器的写操作完成
}


/*？？？？以下——未弄懂函数？？？？*/
//比较两个字符串指定长度的内容是否相等
//参数:s1,s2要比较的两个字符串;len,比较长度
//返回值:1,相等;0,不相等
u8 str_cmpx(u8*s1,u8*s2,u8 len)
{
	u8 i;
	for(i=0;i<len;i++)if((*s1++)!=*s2++)return 0;
	return 1;	   
}

extern const u8 *COMPILED_DATE;//获得编译日期
extern const u8 *COMPILED_TIME;//获得编译时间
const u8 Month_Tab[12][3]={"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"}; 
//自动设置时间为编译器时间   
void Auto_Time_Set(void)
{
	u8 temp[3];
	u8 i;
	u8 mon,date;
	u16 year;
	u8 sec,min,hour;
	for(i=0;i<3;i++)temp[i]=COMPILED_DATE[i];   
	for(i=0;i<12;i++)if(str_cmpx((u8*)Month_Tab[i],temp,3))break;	
	mon=i+1;//得到月份
	if(COMPILED_DATE[4]==' ')date=COMPILED_DATE[5]-'0'; 
	else date=10*(COMPILED_DATE[4]-'0')+COMPILED_DATE[5]-'0';  
	year=1000*(COMPILED_DATE[7]-'0')+100*(COMPILED_DATE[8]-'0')+10*(COMPILED_DATE[9]-'0')+COMPILED_DATE[10]-'0';	   
	hour=10*(COMPILED_TIME[0]-'0')+COMPILED_TIME[1]-'0';  
	min=10*(COMPILED_TIME[3]-'0')+COMPILED_TIME[4]-'0';  
	sec=10*(COMPILED_TIME[6]-'0')+COMPILED_TIME[7]-'0';  
	RTC_Set(year,mon,date,hour,min,sec);
	//printf("%d-%d-%d  %d:%d:%d\n",year,mon,date,hour,min,sec);
} 



