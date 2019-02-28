#include "TFT.h"
#include "font.h"

u16  POINT_COLOR;
u16  BACK_COLOR;

/*
 * 函数名：LCD_WR_DATA
 * 描述  ：TFT_LCD写“数据”函数
 * 输入  ：data
 * 输出  ：无
 * 调用  ：TFT.c内部调用
 */
__inline void LCD_WR_DATA(u16 data)
{
	LCD_RS_SET;
	LCD_CS_CLR;
	DATAOUT(data);
	LCD_WR_CLR;
	LCD_WR_SET;
	LCD_CS_SET;
}

/*
 * 函数名：LCD_WR_REG
 * 描述  ：TFT_LCD写“寄存器”函数
 * 输入  ：data
 * 输出  ：无
 * 调用  ：TFT.c内部调用
 */
__inline void LCD_WR_REG(u8 data)
{ 
	LCD_RS_CLR;//写地址  
 	LCD_CS_CLR; 
	DATAOUT(data); 
	LCD_WR_CLR; 
	LCD_WR_SET; 
 	LCD_CS_SET;   
}

/*
 * 函数名：LCD_WriteReg
 * 描述  ：写寄存器
 * 输入  ：LCD_Reg寄存器地址 LCD_RegValue写入寄存器的值
 * 输出  ：无
 * 调用  ：TFT.c内部调用
 */
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);
}

/*
 * 函数名：LCD_ReadReg
 * 描述  ：读寄存器
 * 输入  ：LCD_Reg寄存器地址
 * 输出  ：寄存器的值
 * 调用  ：TFT.c内部调用
 */
u16 LCD_ReadReg(u8 LCD_Reg)
{
	u16 t;
	LCD_WR_REG(LCD_Reg);//写入要读取的寄存器的值
	GPIOB->CRL=0X88888888; //PB0-7  上拉输入
	GPIOB->CRH=0X88888888; //PB8-15 上拉输入
	GPIOB->ODR=0XFFFF;    //全部输出高	

	LCD_RS_SET;
	LCD_CS_CLR;
	//读取数据(读寄存器时,并不需要读2次)
	LCD_RD_CLR;
	delay_us(5);//FOR 8989,延时5us
	LCD_RD_SET;
	t=DATAIN;  
	LCD_CS_SET;

	GPIOB->CRL=0X33333333; //PB0-7  上拉输出
	GPIOB->CRH=0X33333333; //PB8-15 上拉输出
	GPIOB->ODR=0XFFFF;    //全部输出高
	return t;
		
}

/*
 * 函数名：LCD_WriteRAM_Prepare
 * 描述  ：准备开始写GRAM(地址计数器自动+1)
 * 输入  ：无
 * 输出  ：无
 * 调用  ：TFT.c内部调用
 */
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(R34);
}

/*
 * 函数名：LCD_WriteRAM
 * 描述  ：LCD写GRAM
 * 输入  ：RGB_Code
 * 输出  ：无
 * 调用  ：TFT.c内部调用
 */
void LCD_WriteRAM(u16 RGB_Code)
{
	LCD_WR_DATA(RGB_Code);//写十六位GRAM	
}

/*
 * 函数名：LCD_BGR2RGB
 * 描述  ：GBR 转 RGB
 * 输入  ：GBR格式的数据
 * 输出  ：无
 * 调用  ：TFT.c内部调用
 */
u16 LCD_BGR2RGB(u16 c)
{
	u16 r,g,b,rgb;
	b = (c>>0) & 0x1F;
	g = (c>>5) & 0x3F;
	r = (c>>11) & 0x1F;
	rgb = (b<<11)+(g<<5)+(r<<0);
	return(rgb);		
}





/*
 * 函数名：LCD_Init
 * 描述  ：LCD初始化
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void LCD_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;//定义IO口配置 结构体
	u16 DeviceCode;//ID号

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//使能GPIOC时钟
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开启辅助时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//JTAG-DP失能 + SW-DP使能(PA13 PA14)

	//选中引脚6-10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO输出速度50Hz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure);//配置GPIOC 6-10引脚
	GPIO_SetBits(GPIOC,GPIO_Pin_10|GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_6);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;//PB所有引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);//配置GPIOB 所有引脚
	GPIO_SetBits(GPIOB,GPIO_Pin_All);
	
	
	
	delay_ms(50); // delay 50 ms
	LCD_WriteReg(0x00,0x0001);//启动震荡
	delay_ms(50); // delay 50 ms
	DeviceCode = LCD_ReadReg(0x00);//读取ID号
	printf("LCD ID:%x\n",DeviceCode); //打印LCD ID
	
	/* ---ID 9325启动程序--- */
	LCD_WriteReg(0xE5,0x78F0); 
	LCD_WriteReg(0x01,0x0100); 
	LCD_WriteReg(0x02,0x0700); 
	LCD_WriteReg(0x03,0x1030); 
	LCD_WriteReg(0x04,0x0000); 
	LCD_WriteReg(0x08,0x0202);  
	LCD_WriteReg(0x09,0x0000);
	LCD_WriteReg(0x0A,0x0000); 
	LCD_WriteReg(0x0C,0x0000); 
	LCD_WriteReg(0x0D,0x0000);
	LCD_WriteReg(0x0F,0x0000);
	//power on sequence VGHVGL
	LCD_WriteReg(0x10,0x0000);   
	LCD_WriteReg(0x0011,0x0007);  
	LCD_WriteReg(0x0012,0x0000);  
	LCD_WriteReg(0x0013,0x0000); 
	LCD_WriteReg(0x0007,0x0000); 
	//vgh 
	LCD_WriteReg(0x0010,0x1690);   
	LCD_WriteReg(0x0011,0x0227);
	//delayms(100);
	//vregiout 
	LCD_WriteReg(0x0012,0x009D); //0x001b
	//delayms(100); 
	//vom amplitude
	LCD_WriteReg(0x0013,0x1900);
	//delayms(100); 
	//vom H
	LCD_WriteReg(0x0029,0x0025); 
	LCD_WriteReg(0x002B,0x000D); 
	//gamma
	LCD_WriteReg(0x0030,0x0007);
	LCD_WriteReg(0x0031,0x0303);
	LCD_WriteReg(0x0032,0x0003);// 0006
	LCD_WriteReg(0x0035,0x0206);
	LCD_WriteReg(0x0036,0x0008);
	LCD_WriteReg(0x0037,0x0406); 
	LCD_WriteReg(0x0038,0x0304);//0200
	LCD_WriteReg(0x0039,0x0007); 
	LCD_WriteReg(0x003C,0x0602);// 0504
	LCD_WriteReg(0x003D,0x0008); 
	//ram
	LCD_WriteReg(0x0050,0x0000); 
	LCD_WriteReg(0x0051,0x00EF);
	LCD_WriteReg(0x0052,0x0000); 
	LCD_WriteReg(0x0053,0x013F);  
	LCD_WriteReg(0x0060,0xA700); 
	LCD_WriteReg(0x0061,0x0001); 
	LCD_WriteReg(0x006A,0x0000); 
	//
	LCD_WriteReg(0x0080,0x0000); 
	LCD_WriteReg(0x0081,0x0000); 
	LCD_WriteReg(0x0082,0x0000); 
	LCD_WriteReg(0x0083,0x0000); 
	LCD_WriteReg(0x0084,0x0000); 
	LCD_WriteReg(0x0085,0x0000); 
	//
	LCD_WriteReg(0x0090,0x0010); 
	LCD_WriteReg(0x0092,0x0600); 
		
	LCD_WriteReg(0x0007,0x0133);
	LCD_WriteReg(0x00,0x0022);//
	/* ----------------------- */
		
	POINT_COLOR = WHITE;
	BACK_COLOR = BLACK;	 
	LCD_Clear(BACK_COLOR);
	LCD_LED(1);//点亮背光
}

/*
 * 函数名：LCD_DisplayOn
 * 描述  ：LCD开启显示
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内/外部调用
 */
void LCD_DisplayOn(void)
{
	LCD_WriteReg(R7, 0x0173); //26万色显示开启	
}

/*
 * 函数名：LCD_DisplayOff
 * 描述  ：LCD关闭显示
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内/外部调用
 */
void LCD_DisplayOff(void)
{
	LCD_WriteReg(R7, 0x0);//关闭显示	
} 

/*
 * 函数名：LCD_Clear
 * 描述  ：LCD清屏函数
 * 输入  ：无
 * 输出  ：Color 要清屏的填充色
 * 调用  ：内/外部调用
 */
void LCD_Clear(u16 Color)
{
	u32 index=0;      
	LCD_SetCursor(0x00,0x0000); //设置光标位置 
	LCD_WriteRAM_Prepare(); //开始写入GRAM	 	  
	for(index=76800;index>0;index--)
	{
		LCD_WR_DATA(Color);   
	}	
}

/*
 * 函数名：LCD_SetCursor
 * 描述  ：设置光标位置(__inline与宏定义基本相同,加快运行速度)
 * 输入  ：Xpos X轴 Ypos Y轴
 * 输出  ：无
 * 调用  ：内部调用
 */
__inline void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
	LCD_WriteReg(R32, Xpos);
	LCD_WriteReg(R33, Ypos);		
}

/*
 * 函数名：LCD_DrawPoint
 * 描述  ：画1点
 * 输入  ：x X轴坐标 y Y轴坐标
 * 输出  ：无
 * 调用  ：外部调用
 */
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);//设置光标位置 
	LCD_WR_REG(R34);//开始写入GRAM
	LCD_WR_DATA(POINT_COLOR); 	
}

/*
 * 函数名：LCD_ReadPoint
 * 描述  ：读1点
 * 输入  ：x X轴坐标 y Y轴坐标
 * 输出  ：此点的颜色RGB
 * 调用  ：外部调用
 */
u16 LCD_ReadPoint(u16 x,u16 y)
{
	u16 t;	
	if(x>=LCD_W||y>=LCD_H)return 0;//超过了范围,直接返回	
	LCD_SetCursor(x,y);
	LCD_WR_REG(R34);//选择GRAM地址

	GPIOB->CRL=0X88888888; //PB0-7  上拉输入
	GPIOB->CRH=0X88888888; //PB8-15 上拉输入
	GPIOB->ODR=0XFFFF;     //全部输出高

	LCD_RS_SET;
	LCD_CS_CLR;
	//读取数据(读GRAM时,需要读2次)
	LCD_RD_CLR;					   
	LCD_RD_SET;
	delay_us(2);//FOR 9320,延时2us
	//dummy READ
	LCD_RD_CLR;					   
	delay_us(2);//FOR 8989,延时2us					   
	LCD_RD_SET;
	t=DATAIN;  
	LCD_CS_SET;

	GPIOB->CRL=0X33333333; //PB0-7  上拉输出
	GPIOB->CRH=0X33333333; //PB8-15 上拉输出
	GPIOB->ODR=0XFFFF;    //全部输出高
	return t;
	//return LCD_BGR2RGB(t);		
}

/*
 * 函数名：LCD_Fill
 * 描述  ：在指定区域内填充指定颜色 
 *         区域大小:(xend-xsta)*(yend-ysta)
 * 输入  ：xsta	ysta xend yend color
 * 输出  ：无
 * 调用  ：外部调用
 */
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{
	u16 i;
	u32 point_num;

	//设置写入数据区域
	LCD_WriteReg(R80,xsta);
	LCD_WriteReg(R81,xend);
	LCD_WriteReg(R82,ysta);
	LCD_WriteReg(R83,yend);
	point_num = (xend-xsta+1)*(yend-ysta+1);
	
	LCD_SetCursor(xsta,ysta); //设置光标位置 
	LCD_WriteRAM_Prepare(); //开始写入GRAM
	for(i=point_num;i>0;i--)
	{
		LCD_WR_DATA(color);	
	}
	LCD_WriteReg(R80,0);
	LCD_WriteReg(R81,240);
	LCD_WriteReg(R82,0);
	LCD_WriteReg(R83,320);	
}


/*
 * 函数名：LCD_ShowChar
 * 描述  ：在指定位置显示一个字符
 *		   x,y:区域的坐标
 *		   num:要显示的字符 
 *         size:字体大小 12/16
 *		   mode:叠加方式(1)还是非叠加方式(0)
 * 输入  ：x y num size mode
 * 输出  ：无
 * 调用  ：外部调用
 */
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{
#define MAX_CHAR_POSX 231
#define MAX_CHAR_POSY 303

	u8 temp;//存取字符串
    u8 i,j;//循环次数
	if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;//判断是否出界	
	num=num-' ';//得到偏移后的值
	
	if(mode == 0)//非叠加模式
	{
		//设置写入数据区域
		LCD_WriteReg(R80,x);
		LCD_WriteReg(R82,y);
		if(size == 16)
		{
			LCD_WriteReg(R81,x+8-1);
			LCD_WriteReg(R83,y+16-1);
		}
		else if(size == 12)
		{
			LCD_WriteReg(R81,x+6-1);
			LCD_WriteReg(R83,y+12-1);
		}
		LCD_SetCursor(x,y);//设置光标位置 
		LCD_WriteRAM_Prepare();//开始写入GRAM

		for(i=0;i<size;i++)
		{
			if(size == 16) temp=asc2_1608[num][i];//调用1608字体
			else if (size == 12) temp=asc2_1206[num][i];//调用1206字体 
			for(j=0;j<size/2;j++)
			{
				if(temp&(0x01<<j)) LCD_WR_DATA(POINT_COLOR);
				else LCD_WR_DATA(BACK_COLOR);		
			}
		}
		//区域重设
		LCD_WriteReg(R80,0);
		LCD_WriteReg(R81,240);
		LCD_WriteReg(R82,0);
		LCD_WriteReg(R83,320);
	 }
	 else//叠加模式
	 {
	 	for(i=0;i<size;i++)
		{
			if(size == 16) temp=asc2_1608[num][i];//调用1608字体
			else if (size == 12) temp=asc2_1206[num][i];//调用1206字体 
			for(j=0;j<size/2;j++)
			{
				if(temp&(0x01<<j)) LCD_DrawPoint(x+j,y+i);//画一个点		
			}
		}	
	 }					
}

/*
 * 函数名：LCD_ShowNum
 * 描述  ：显示正整
 *		   x,y:区域的坐标
 *		   num:要显示的数字 
 *		   len:显示数字的长度
 *         size:字体大小 12/16
 *		   mode:叠加方式(1)还是非叠加方式(0)
 * 输入  ：x y num len size mode
 * 输出  ：无
 * 调用  ：外部调用
 */
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode)
{
	u8 i;
	u32 temp = 1;
	u32 n;
	temp = pow(10,len-1);//显示数位的数量级	
	n = num%(temp*10);//去掉超出的数位

	for(i=0;i<len;i++)
	{
		LCD_ShowChar(x+8*i,y,(n/temp)+'0',size,mode);
		n = n%temp;
		temp = temp/10;			
	}		
}

/*
 * 函数名：LCD_ShowString
 * 描述  ：显示一个字符串
 *		   x,y:区域的坐标
 *		   *p:字符串首地址
 *		   mode:叠加方式(1)还是非叠加方式(0)
 * 输入  ：x y *p mode
 * 输出  ：无
 * 调用  ：外部调用
 */
void LCD_ShowString(u16 x,u16 y,const u8 *p,u8 mode)
{
	while(*p != '\0')
	{
		if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;LCD_Clear(BLACK);}
		LCD_ShowChar(x,y,*p,16,mode);
		x += 8; 
		p++;	
	}	
}


/*
 * 函数名：LCD_ShowFloatNum
 * 描述  ：显示正小数
 *		   x,y:区域的坐标
 *		   num:小数数值
 *		   fl_n:小数位数
 *         nu_n:整数位数
 *		   mode:叠加方式(1)还是非叠加方式(0)
 * 输入  ：x y *p mode
 * 输出  ：无
 * 调用  ：外部调用
 */
void LCD_ShowFloatNum(u16 x,u16 y,float num,u8 nu_n,u8 fl_n,u8 size,u8 mode)
{
	u16 i;
	u16 temp1 = (u16)num;//整数部分
	float temp2;//小数部分
	
	i = pow(10,fl_n);
	temp2 = (num-(u16)num)*i;
	
	LCD_ShowNum(x,y,temp1,nu_n,size,mode);//显示整数部分
	LCD_ShowChar(x+nu_n*8,y,'.',size,mode);//显示一个字符
	LCD_ShowNum(x+nu_n*8+8,y,temp2,fl_n,size,mode);//显示整数部分
}



/*
 * 函数名：Draw_Circle
 * 描述  ：画圆(普通算法-解方程)
 * 输入  ：x0 X轴坐标 y0 Y轴坐标 r圆的半径
 * 输出  ：无
 * 调用  ：外部调用
 */
void Draw_Circle(u16 x0,u16 y0,u8 r)
{
	u16 i;
	u16 temp1,temp2;
	
	for(i=x0-r;i<=x0;i++)
	{
		temp1 = y0+sqrtf(r*r-((s16)i-x0)*((s16)i-x0));
		temp2 = y0-sqrtf(r*r-((s16)i-x0)*((s16)i-x0));
		LCD_DrawPoint(i,temp1);
		LCD_DrawPoint(i,temp2);
		LCD_DrawPoint(2*x0-i,temp1);
		LCD_DrawPoint(2*x0-i,temp2);
	}
 	
	for(i=y0-r;i<=y0;i++)
	{
		temp1 = x0+sqrtf(r*r-((s16)i-y0)*((s16)i-y0));
		temp2 = x0-sqrtf(r*r-((s16)i-y0)*((s16)i-y0));
		LCD_DrawPoint(temp1,i);
		LCD_DrawPoint(temp2,i);
		LCD_DrawPoint(temp1,2*y0-i);
		LCD_DrawPoint(temp2,2*y0-i);	
	}			
}


/*
 * 函数名：LCD_DrawLine
 * 描述  ：画线(普通算法-解方程)
 * 输入  ：x1 y1 起始坐标 x2 y2结束坐标
 * 输出  ：无
 * 调用  ：外部调用
 */
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 i;
	float k_x,b_x;
	float k_y,b_y;

	if(x1 == x2) //斜率无穷大
	{
		if(y1>y2)
		{
			for(i=y2;i<=y1;i++) LCD_DrawPoint(x1,i); 
		}
		else
		{
			for(i=y1;i<=y2;i++) LCD_DrawPoint(x1,i);	
		}	
	}
	else if(y1 == y2)//斜率为0
	{
		if(x1>x2)
		{
			for(i=x2;i<=x1;i++) LCD_DrawPoint(i,y1);  
		}
		else
		{
			for(i=x1;i<=x2;i++) LCD_DrawPoint(i,y1);  	
		}
	}
	else
	{
		k_x = ((float)y2-y1)/((float)x2-x1);//x为自变量时的斜率
		b_x = (float)((y1*x2)-(float)(y2*x1))/(float)(x2-x1);//x为自变量时的常数
		
		k_y = 1/k_x;//y为自变量时的斜率
		b_x = -b_x/k_x;//y为自变量时的常数

		if(fabsf(x2-x1)>fabsf(y2-y1))
		{
			if(x1>x2)
			{
				for(i=x2;i<=x1;i++) LCD_DrawPoint(i,(i*k_x+b_x));  
			}
			else
			{
				for(i=x1;i<=x2;i++) LCD_DrawPoint(i,(i*k_x+b_x)); 
			}	
		}
		else
		{
			if(y1>y2)
			{
				for(i=y2;i<=y1;i++) LCD_DrawPoint((i*k_y+b_y),i); 
			}
			else
			{
				for(i=y1;i<=y2;i++) LCD_DrawPoint((i*k_y+b_y),i);	
			}	
		}		
	}
}

/*
 * 函数名：LCD_DrawRectangle
 * 描述  ：画长方形
 * 输入  ：x1 y1 起始坐标 x2 y2结束坐标
 * 输出  ：无
 * 调用  ：外部调用
 */
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x1,y2);//画2垂直线
	LCD_DrawLine(x2,y1,x2,y2);
	LCD_DrawLine(x1,y1,x2,y1);//画2横线
	LCD_DrawLine(x1,y2,x2,y2);			
}



