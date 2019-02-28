#include "maincom.h"
#include "dma.h"
#include "control.h"

u8 DMA_DATA_SEND_FLAG;
u8 Receive_Complete;

/*
 * 函数名：USART1_Config
 * 描述  ：USART1 GPIO 配置,工作模式配置。115200 8-N-1
 *		 : 接收中断模式
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void USART1_Config(u32 speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//开启GPIOA和USART1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);

	//IO口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//PA9复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//PA10浮空输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//IO口配置写入

	//USART1配置
	USART_InitStructure.USART_BaudRate = speed;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//发送停止位 1位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//接收发送使能
	
	USART_Init(USART1, &USART_InitStructure);//USART1配置写入
	USART_Cmd(USART1, ENABLE);//USART1使能
	
	//USART1接收中断使能
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	//设置NVIC中USART1中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//通道设置为USART1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//响应3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//打开USART1中断1通道
	NVIC_Init(&NVIC_InitStructure);//写入配置 
 			
}

/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
	/* 将Printf内容发往串口 */
	USART_SendData(USART1, (unsigned char) ch);//USART1发送数据函数
  	while (!(USART1->SR & USART_FLAG_TXE));//判断是否送入"发送移位"寄存器
	return (ch);	
}


//USART1 发送一个字节
void USART1_Send1Char(u8 c)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	USART_SendData(USART1,c);
}

u8 recv_dat[32];
u8 recv_point=0,recv_buf=0;
//*****USART1中断函数*****//
void USART1_IRQHandler(void)
{
	u8 temp;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)//接收中断
	{ 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断位
		
		temp = USART_ReceiveData(USART1);
		
		if(temp == 0x8A)
		{
			recv_buf = 1;//开始接收数据
		}
		
		if(recv_buf == 1)
		{
			recv_dat[recv_point++] = temp;
			
			if(recv_point == 32)
			{
				recv_point = 0;
				recv_buf = 0;//已经接收了32个数据，停止接收
				Receive_Complete = 1;//标志已经接收到了32位数据
			}
		}
	} 	
}


/////////////////////////////////////////////////////串口发送/////////////////////////////////////////////////////////////////////


//发送姿态角和传感器数据
void Data_Send_Attitude(s16* acc,s16* gyro,s16* mag,float rool,float pitch,float yaw)
{
	u8 updata[32];
	u8 sum,i,point=0;
	s16 angle[3];
	
	angle[0] = (s16)(rool*100);
	angle[1] = (s16)(pitch*100);
	angle[2] = (s16)(yaw*10);
	
	updata[point++] = 0x88;
	updata[point++] = 0xAF;
	updata[point++] = 0x1C;
	
	for(i=0;i<3;i++)
	{
		updata[point++] = acc[i]>>8;
		updata[point++] = acc[i]&0xFF;
	}
	for(i=0;i<3;i++)
	{
		updata[point++] = gyro[i]>>8;
		updata[point++] = gyro[i]&0xFF;
	}
	for(i=0;i<3;i++)
	{
		updata[point++] = mag[i]>>8;
		updata[point++] = mag[i]&0xFF;
	}
	for(i=0;i<3;i++)
	{
		updata[point++] = angle[i]>>8;
		updata[point++] = angle[i]&0xFF;
	}
	for(i=0;i<4;i++)
	{
		updata[point++] = 0x00;
	}
	
	for(i=0;i<31;i++)//校验和
	{
		sum += updata[i];
	}
	updata[31] = sum;
	
	DMA1_USART1_SEND((u32)updata,32);	
}

//发送遥控数据 和 电机PWM数据
void Data_Send_Control(u16 *rc_ch,u16 PWM1,u16 PWM2,u16 PWM3,u16 PWM4,u16 votage)
{
	u8 updata[32];
	u8 i,sum,point=0;
	
	updata[point++] = 0x88;
	updata[point++] = 0xAE;
	updata[point++] = 0x12;
	
	updata[point++] = rc_ch[3-1]>>8;//3通道 油门
	updata[point++] = rc_ch[3-1]&0xFF;
	
	updata[point++] = rc_ch[4-1]>>8;//4通道 偏航
	updata[point++] = rc_ch[4-1]&0xFF;
	
	updata[point++] = rc_ch[1-1]>>8;//1通道 横滚
	updata[point++] = rc_ch[1-1]&0xFF;
	
	updata[point++] = rc_ch[2-1]>>8;//2通道 俯仰
	updata[point++] = rc_ch[2-1]&0xFF;
	
	for(i=0;i<5;i++)//遥控器数值
	{
		updata[point++] = 0;//高8位
		updata[point++] = 0;//低8位
	}
	
	updata[point++] = PWM1>>8;
	updata[point++] = PWM1&0xFF;
	updata[point++] = PWM2>>8;
	updata[point++] = PWM2&0xFF;
	updata[point++] = PWM3>>8;
	updata[point++] = PWM3&0xFF;
	updata[point++] = PWM4>>8;
	updata[point++] = PWM4&0xFF;
	
	
	
	updata[point++] = votage>>8;
	updata[point++] = votage&0xFF;
	
	for(i=0;i<31;i++)//校验和
	{
		sum += updata[i];
	}
	updata[31] = sum;
	
	
	DMA1_USART1_SEND((u32)updata,32);
}

//发送偏移数据
void Data_Send_Offset(s16 acc_offset_x,s16 acc_offset_y,s16 acc_offset_z,s16 gyro_offset_x,s16 gyro_offset_y,s16 gyro_offset_z)
{
	u8 updata[32];
	u8 i,sum,point=0;
	
	updata[point++]=0x88;
	updata[point++]=0xAC;
	updata[point++]=0x1C;
	updata[point++]=0xAC;
	
	updata[point++]=acc_offset_x>>8;
	updata[point++]=acc_offset_x&0xFF;
	
	updata[point++]=acc_offset_y>>8;
	updata[point++]=acc_offset_y&0xFF;
	
	updata[point++]=acc_offset_z>>8;
	updata[point++]=acc_offset_z&0xFF;
	
	updata[point++]=gyro_offset_x>>8;
	updata[point++]=gyro_offset_x&0xFF;
	
	updata[point++]=gyro_offset_y>>8;
	updata[point++]=gyro_offset_y&0xFF;
	
	updata[point++]=gyro_offset_z>>8;
	updata[point++]=gyro_offset_z&0xFF;
	
	sum = 0;
	for(i=0;i<31;i++)
	{
		sum += updata[i];
	}
	
	updata[31]=sum;
	
	//NRF_TxPacket(NRF24L01_TXDATA,32);
	
	DMA1_USART1_SEND((u32)updata,32);
}

//发送PID数据
void Data_Send_PID(float Roll_P,float Roll_I,float Roll_D,
									 float Pitch_P,float Pitch_I,float Pitch_D,
								   float Yaw_P,float Yaw_I,float Yaw_D)
{
	u8 updata[32];
	u8 i,sum,point=0;
	u16 temp;
	
	updata[point++]=0x88;
	updata[point++]=0xAC;
	updata[point++]=0x1C;
	updata[point++]=0xAD;
	
	temp = Roll_P * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Roll_I * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Roll_D * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	
	temp = Pitch_P * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Pitch_I * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Pitch_D * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	
	temp = Yaw_P * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Yaw_I * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	temp = Yaw_D * 1000;
	updata[point++]=temp>>8;
	updata[point++]=temp&0xFF;
	
	sum = 0;
	for(i=0;i<31;i++)
	{
		sum += updata[i];
	}
	
	updata[31]=sum;
	
	DMA1_USART1_SEND((u32)updata,32);
}


//////////////////////////////////////////////////////////////串口接收/////////////////////////////////////////////////////////////

//判断从上位机接收到的命令
//返回值：
//0:无效数据
//1:校准遥控器
//2:读取PID值
//3:上位机给飞控发送PID数据
u8 Recv_Command(void)
{
	
	if(Receive_Complete == 1)//接收到数据
	{
		Receive_Complete = 0;
		
		if(recv_dat[0] == 0x8A && recv_dat[1] == 0x8B && recv_dat[2] == 0x1C //校准遥控器
				&& recv_dat[3] == 0xAA && recv_dat[4] == 0xA3)
		{
			return 1;
		}
		else if(recv_dat[0] == 0x8A && recv_dat[1] == 0x8B && recv_dat[2] == 0x1C //读取PID值
				&& recv_dat[3] == 0xAD)
		{
			return 2;
		}
		else if(recv_dat[0] == 0x8A && recv_dat[1] == 0x8B && recv_dat[2] == 0x1C //上位机给飞控发送PID数据
				&& recv_dat[3] == 0xAE)
		{
			return 3;
		}
		else return 0;
	}
	else return 0;//没有接收到数据
}

//从上位机获取PID值
void Get_Recv_PID(void)
{
	u8 point = 4;
	u16 temp;
	
	temp = recv_dat[point++];//高8位
	temp <<= 8;
	temp |= recv_dat[point++];//低8位
	PID_ROL.P = (float)temp/1000;	
	temp = recv_dat[point++];//高8位
	temp <<= 8;
	temp |= recv_dat[point++];//低8位
	PID_ROL.I = (float)temp/1000;
	temp = recv_dat[point++];//高8位
	temp <<= 8;
	temp |= recv_dat[point++];//低8位
	PID_ROL.D = (float)temp/1000;
	
	temp = recv_dat[point++];//高8位
	temp <<= 8;
	temp |= recv_dat[point++];//低8位
	PID_PIT.P = (float)temp/1000;
	temp = recv_dat[point++];//高8位
	temp <<= 8;
	temp |= recv_dat[point++];//低8位
	PID_PIT.I = (float)temp/1000;
	temp = recv_dat[point++];//高8位
	temp <<= 8;
	temp |= recv_dat[point++];//低8位
	PID_PIT.D = (float)temp/1000;
	
	temp = recv_dat[point++];//高8位
	temp <<= 8;
	temp |= recv_dat[point++];//低8位
	PID_YAW.P = (float)temp/1000;
	temp = recv_dat[point++];//高8位
	temp <<= 8;
	temp |= recv_dat[point++];//低8位
	PID_YAW.I = (float)temp/1000;
	temp = recv_dat[point++];//高8位
	temp <<= 8;
	temp |= recv_dat[point++];//低8位
	PID_YAW.D = (float)temp/1000;
	
}


