#include "touch.h"

Pen_Holder Pen_Point;//定义笔实体 //默认为touchtype=0的数据.

u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;

//SPI写数据
//写入1byte数据
void ADS_Write_Byte(u8 num)
{
	u8 count;
	TCLK_L;
	for(count=0;count<8;count++)
	{
		if(num&0x80) TDIN_H;
		else TDIN_L;
		num <<= 1;
		TCLK_L;
		TCLK_H;//上升沿有效 
	}	
}

//SPI读数据 
//读取adc值
u16 ADS_Read_AD(u8 CMD)	
{
	u8 count;
	u16 Num=0;
	TCLK_L;//拉低时钟 
	TCS_L;//选中XXXXX
	ADS_Write_Byte(CMD);//发送命令字
	delay_us(6);//XXXXX转换时间最长为6us
	TCLK_H;//给1个时钟，清除BUSY   	    
	TCLK_L;
	for(count=0;count<15;count++)
	{
		Num <<= 1;
		TCLK_H;  	    
		TCLK_L;
		if(DOUT_read) Num++;	
	}
	Num >>= 3;//高12位有效
	TCS_H;//释放XXXXX
	return(Num);	
}

//读取一个坐标值
//连续读取READ_TIMES次数据,对这些数据升序排列,
//然后去掉最低和最高LOST_VAL个数,取平均值 
#define READ_TIMES 15 //读取次数
#define LOST_VAL 5	  //丢弃值

u16 ADS_Read_XY(u8 xy,u8 delay)
{
	u16 i,j;
	u16 buf[READ_TIMES];
	u16 sum;
	u16 temp;
	if(delay!=0)
	{
		delay_ms(delay);
	}
	for(i=0;i<READ_TIMES;i++)
	{
		buf[i] = ADS_Read_AD(xy);
	}
	//冒泡排序
	for(i=0;i<READ_TIMES-1;i++)
	{
		for(j=0;j<READ_TIMES-1-i;j++)
		{
			if(buf[j]>buf[j+1])
			{
				temp = buf[j+1];
				buf[j+1] = buf[j];
				buf[j] = temp;	
			}
		}
	}
	////////////
	

	sum = 0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)
	{
		sum += buf[i];
	}
	temp = sum/(READ_TIMES-LOST_VAL*2);
	
	return (temp);
}

//带滤波的坐标读取
//最小值不能少于100.
u8 Read_ADS(u16* x,u16* y,u8 delay)
{	
	u16 xtemp,ytemp;
	xtemp = ADS_Read_XY(CMD_RDX,delay);
	ytemp = ADS_Read_XY(CMD_RDY,delay);
	if(xtemp<100||ytemp<100) return 0;//读数失败
	*x = xtemp;
	*y = ytemp;
	return 1;//读数成功
}

//2次读取ADS7846,连续读取2次有效的AD值,且这两次的偏差不能超过
//50,满足条件,则认为读数正确,否则读数错误.	   
//该函数能大大提高准确度
#define ERR_RANGE 50 //误差范围 
u8 Read_ADS2(u16 *x,u16 *y,u8 delay)
{
	u16 x1,x2,y1,y2;
	if(Read_ADS(&x1,&y1,delay)==0) return 0;
	if(Read_ADS(&x2,&y2,delay)==0) return 0;
	if(((s16)x1-(s16)x2<=50&&(s16)x1-(s16)x2>=-50)&&((s16)y1-(s16)y2<=50&&(s16)y1-(s16)y2>=-50))
	{
		*x = (x1+x2)/2;
		*y = (y1+y2)/2;	
		return 1;
	}
	else return 0;	
}

//读取一次坐标值	
//仅仅读取一次,直到PEN松开才返回!
u8 Read_TP_Once(void)
{
	u8 t = 0;
	Pen_Int_Set(0);//关闭中断
	Pen_Point.Key_Sta=Key_Up;
	Read_ADS2(&Pen_Point.X,&Pen_Point.Y,5);
	while(PEN_read==0&&t<=250)
	{
		t++;
		delay_ms(10);
	}
	Pen_Int_Set(1);//开启中断
	if(t>250) return 0;//按下2.5s 认为无效
	else return 1;
}


//////////////////////////////////////////////
//与LCD部分有关的函数

//画一个触摸点
//用来校准用的
void Drow_Touch_Point(u8 x,u16 y)
{
	LCD_DrawLine(x-12,y,x+13,y);//横线
	LCD_DrawLine(x,y-12,x,y+13);//竖线
	LCD_DrawPoint(x+1,y+1);
	LCD_DrawPoint(x-1,y+1);
	LCD_DrawPoint(x+1,y-1);
	LCD_DrawPoint(x-1,y-1);
	Draw_Circle(x,y,6);//画中心圈	
}

//画一个大点
//2*2的点			   
void Draw_Big_Point(u8 x,u16 y)
{	    
	LCD_DrawPoint(x,y);//中心点 
	LCD_DrawPoint(x+1,y);
	LCD_DrawPoint(x,y+1);
	LCD_DrawPoint(x+1,y+1);	 	  	
}

void ADJ_INFO_SHOW(u8*str)
{
	LCD_ShowString(40,40,"x1:       y1:       ",0);
	LCD_ShowString(40,60,"x2:       y2:       ",0);
	LCD_ShowString(40,80,"x3:       y3:       ",0);
	LCD_ShowString(40,100,"x4:       y4:       ",0);
 	LCD_ShowString(40,100,"x4:       y4:       ",0);
 	LCD_ShowString(40,120,str,0);					   
}
///////////////////////////////////////////////////////////


//转换结果
//根据触摸屏的校准参数来决定转换后的结果,保存在X0,Y0中
void Convert_Pos(void)
{
	if(Read_ADS2(&Pen_Point.X,&Pen_Point.Y,0))
	{
		Pen_Point.X0 = Pen_Point.xfac*Pen_Point.X + Pen_Point.xoff;
		Pen_Point.Y0 = Pen_Point.yfac*Pen_Point.Y + Pen_Point.yoff;	
	}	
}

//.中断,检测到PEN脚的一个下降沿
//置位Pen_Point.Key_Sta为按下状态
//中断线0线上的中断检测
void EXTI1_IRQHandler(void)
{
	Pen_Point.Key_Sta = Key_Down;//按键按下
	EXTI_ClearITPendingBit(EXTI_Line1);//清除LINE1上的中断标志位  	
}

//PEN中断设置
void Pen_Int_Set(u8 en)
{ 
	EXTI_InitTypeDef EXTI_InitStructure;
	if(en)//开启line1上的中断
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;//外部线路EXIT1 
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//设置 EXTI线路为中断请求 
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//设置输入线路下降沿为中断请求  
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能外部中断新状态
		EXTI_Init(&EXTI_InitStructure);//配置外部中断
	}
	else//关闭line1上的中断
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;//外部线路EXIT1 
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//设置 EXTI线路为中断请求 
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//设置输入线路下降沿为中断请求  
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;//关闭外部中断新状态
		EXTI_Init(&EXTI_InitStructure);//配置外部中断	
	}	   
}


////////////////////////////////////////////////////////////////////////////////////
//此部分涉及到使用外部EEPROM,如果没有外部EEPROM,屏蔽此部分即可
#ifdef ADJ_SAVE_ENABLE
//保存在EEPROM里面的地址区间基址,占用13个字节(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+12)
#define SAVE_ADDR_BASE 40

//保存校准参数
void Save_Adjdata(void)
{
	s32 temp;
	temp = Pen_Point.xfac*100000000;//保存x校正因素 
	AT24C02_WriteLenByte(SAVE_ADDR_BASE,temp,4);
	temp = Pen_Point.yfac*100000000;//保存y校正因素
	AT24C02_WriteLenByte(SAVE_ADDR_BASE+4,temp,4);

	AT24C02_WriteLenByte(SAVE_ADDR_BASE+8,Pen_Point.xoff,2);//保存x偏移量
	AT24C02_WriteLenByte(SAVE_ADDR_BASE+10,Pen_Point.yoff,2);//保存y偏移量
	AT24C02_WriteOneByte(SAVE_ADDR_BASE+12,Pen_Point.touchtype);//保存触屏类型
	
	temp=0xFF;//标记校准过了
	AT24C02_WriteOneByte(SAVE_ADDR_BASE+13,temp); 	
}

//得到保存在EEPROM里面的校准值
//返回值：1，成功获取数据
//        0，获取失败，要重新校准
u8 Get_Adjdata(void)
{
	s32 tempfac;
	tempfac = AT24C02_ReadOneByte(SAVE_ADDR_BASE+13);//读取标记字,看是否校准过！
	if(tempfac==0xFF)
	{
		tempfac = AT24C02_ReadLenByte(SAVE_ADDR_BASE,4);//取得x校正因素 
		Pen_Point.xfac = (float)tempfac/100000000;
		tempfac = AT24C02_ReadLenByte(SAVE_ADDR_BASE+4,4);//取得y校正因素
		Pen_Point.yfac = (float)tempfac/100000000;
		
		Pen_Point.xoff = (s16)AT24C02_ReadLenByte(SAVE_ADDR_BASE+8,2);//得到x偏移量
		Pen_Point.yoff = (s16)AT24C02_ReadLenByte(SAVE_ADDR_BASE+10,2);//得到y偏移量
		Pen_Point.touchtype	= AT24C02_ReadOneByte(SAVE_ADDR_BASE+12);//读取触屏类型标记
		if(Pen_Point.touchtype)
		{
			CMD_RDX=0X90;
			CMD_RDY=0XD0;	
		}
		else
		{
			CMD_RDX=0XD0;
			CMD_RDY=0X90;		
		}
		return 1;	
	}
	else return 0;	
}

#endif
////////////////////////////////////////////////////////////////////////////////


//触摸屏校准代码
//得到四个校准参数
void Touch_Adjust(void)
{								 
	u16 XY[4][2];
	u8 i=0;
	u16 d1,d2;
	u32 temp1,temp2;
	float fac;
	float xfac_1,xfac_2,yfac_1,yfac_2;
	s16	xoff_1,xoff_2,yoff_1,yoff_2;

	Drow_Touch_Point(20,20);//画点1
	while(1)
	{
		
		if(Pen_Point.Key_Sta == Key_Down)
		{
			if(Read_TP_Once())
			{
				XY[i][0] = Pen_Point.X;
				XY[i][1] = Pen_Point.Y;
				LCD_Clear(BLACK);//清屏
				i++; 
			}
			
			switch(i)
			{
				
				case 1:
				{
					Drow_Touch_Point(220,20);//画点1
					break;
				}
				case 2:
				{
					Drow_Touch_Point(20,300);//画点1
					break;
				}
				case 3:
				{
					Drow_Touch_Point(220,300);//画点1
					break;
				}
				default: break;
			}
		}
		if(i == 4) //全部四个点已经得到
		{	
			i=0;

			temp1 = fabsf(XY[0][0]-XY[1][0]);//x1-x2
			temp2 = fabsf(XY[0][1]-XY[1][1]);//y1-y2
			temp1 *= temp1;
			temp2 *= temp2;
			d1 = sqrtf(temp1+temp2);//得到1,2的距离

			temp1 = fabsf(XY[2][0]-XY[3][0]);//x3-x4
			temp2 = fabsf(XY[2][1]-XY[3][1]);//y3-y4
			temp1 *= temp1;
			temp2 *= temp2;
			d2 = sqrtf(temp1+temp2);//得到3,4的距离

			fac = (float)d1/(float)d2;
			if(fac<0.95||fac>1.05||d1==0||d2==0)//不合格的情况
			{
				Drow_Touch_Point(20,20);//画点1
				LCD_ShowNum(40,140,fac*100,3,16,0);//该数值必须在95~105范围之内.
				continue;//结束本次循环		
			}

			temp1 = fabsf(XY[0][0]-XY[2][0]);//x1-x3
			temp2 = fabsf(XY[0][1]-XY[2][1]);//y1-y3
			temp1 *= temp1;
			temp2 *= temp2;
			d1 = sqrtf(temp1+temp2);//得到1,3的距离

			temp1 = fabsf(XY[1][0]-XY[3][0]);//x2-x4
			temp2 = fabsf(XY[1][1]-XY[3][1]);//y2-y4
			temp1 *= temp1;
			temp2 *= temp2;
			d2 = sqrtf(temp1+temp2);//得到2,4的距离

			fac = (float)d1/(float)d2;
			if(fac<0.95||fac>1.05||d1==0||d2==0)
			{
				Drow_Touch_Point(20,20);//画点1
				LCD_ShowNum(40,140,fac*100,3,16,0);//该数值必须在95~105范围之内.
				continue;//结束本次循环		
			}

			temp1 = fabsf(XY[0][0]-XY[3][0]);//x1-x4
			temp2 = fabsf(XY[0][1]-XY[3][1]);//y1-y4
			temp1 *= temp1;
			temp2 *= temp2;
			d1 = sqrtf(temp1+temp2);//得到1,4的距离

			temp1 = fabsf(XY[1][0]-XY[2][0]);//x2-x3
			temp2 = fabsf(XY[1][1]-XY[2][1]);//y2-y3
			temp1 *= temp1;
			temp2 *= temp2;
			d2 = sqrtf(temp1+temp2);//得到2,3的距离

			fac = (float)d1/(float)d2;
			if(fac<0.95||fac>1.05||d1==0||d2==0)
			{
				Drow_Touch_Point(20,20);//画点1
				LCD_ShowNum(40,140,fac*100,3,16,0);//该数值必须在95~105范围之内.
				continue;//结束本次循环		
			}

			/*******程序运行到这里 说明合格*********/
			xfac_1 = (220-20)/(float)((s16)XY[3][0]-(s16)XY[0][0]);
			xoff_1 = (20*(s16)XY[3][0]-220*(s16)XY[0][0])/((s16)XY[3][0]-(s16)XY[0][0]);
			yfac_1 = (300-20)/(float)((s16)XY[3][1]-(s16)XY[0][1]);
			yoff_1 = (20*(s16)XY[3][1]-300*(s16)XY[0][1])/((s16)XY[3][1]-(s16)XY[0][1]);

			xfac_2 = (220-20)/(float)((s16)XY[1][0]-(s16)XY[2][0]);
			xoff_2 = (20*(s16)XY[1][0]-220*(s16)XY[2][0])/((s16)XY[1][0]-(s16)XY[2][0]);
			yfac_2 = (300-20)/(float)((s16)XY[2][1]-(s16)XY[1][1]);
			yoff_2 = (20*(s16)XY[2][1]-300*(s16)XY[1][1])/((s16)XY[2][1]-(s16)XY[1][1]);

			Pen_Point.xfac = (xfac_1 + xfac_2)/2;
			Pen_Point.yfac = (yfac_1 + yfac_2)/2;
			Pen_Point.xoff = (xoff_1 + xoff_2)/2;
			Pen_Point.yoff = (yoff_1 + yoff_2)/2;
			LCD_Clear(BLACK);//清屏
			break;											
		}	
	}

}

//配置触摸屏IO口
//PC0:TCLK SPI时钟
//PC1:PEN  触摸中断
//PC2:TDIN SPI输入 
//PC3:DOUT SPI输出
//PC13:TCS SPI片选 
void Touch_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);//使能IO口时钟	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_3|GPIO_Pin_13;//0 3 13推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;//1 2 上拉输入  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		 

	//配置外部中断通道及优先级
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//开启中断   
	NVIC_Init(&NVIC_InitStructure);//配置中断优先级

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);//选择 GPIO管脚用作外部中断线路  
	//配置外部中断
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;//外部线路EXIT1 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//设置 EXTI线路为中断请求 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//设置输入线路下降沿为中断请求  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能外部中断新状态
	EXTI_Init(&EXTI_InitStructure);//配置外部中断


////////////////////触摸屏校准/////////////////////////
#ifdef ADJ_SAVE_ENABLE
	AT24C02_Init();
	if(Get_Adjdata()) return;//已经校准
	else
	{
		delay_ms(1000);
		LCD_Clear(BLACK);//清屏
		Touch_Adjust();  //屏幕校准	
		Save_Adjdata();
	}
	Get_Adjdata();
#else
	LCD_Clear(BLACK);//清屏
    Touch_Adjust();  //屏幕校准,带自动保存
#endif
//////////////////////////////////////////////////
	 		
}

