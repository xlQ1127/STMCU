#include "usart.h"

void USART1_Config(void)   
{   
	GPIO_InitTypeDef   GPIO_InitStructure;   
	USART_InitTypeDef  USART_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;	
	/* config USART1 clock */  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);           
	/* USART1 GPIO config */  
	/* Configure USART1 Tx (PA.09) as alternate function pushpull */  
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;   
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;   
	GPIO_Init(GPIOA, &GPIO_InitStructure);       
	/* Configure USART1 Rx (PA.10) as input floating */  
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
	GPIO_Init(GPIOA, &GPIO_InitStructure);             
	/* USART1 mode config */  
	USART_InitStructure.USART_BaudRate 						= 115200;   
	USART_InitStructure.USART_WordLength 					= USART_WordLength_8b;   
	USART_InitStructure.USART_StopBits 						= USART_StopBits_1;   
	USART_InitStructure.USART_Parity 							= USART_Parity_No ;   
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
	USART_InitStructure.USART_Mode 								= USART_Mode_Rx | USART_Mode_Tx;   
	USART_Init(USART1, &USART_InitStructure);  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel 										= USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 3;				   //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 3;				   //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd 								= ENABLE;		   //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);																		   //根据指定的参数初始化VIC寄存器
   
//  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART1, ENABLE);   
}
//************************************************//
//            发送字节                  				  //
//************************************************//
void usart_send(char ch)
{
	USART_SendData(USART1, (uint8_t) ch);	
  while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);
}
//************************************************//
//            打印字符串			          				  //
//************************************************//
void Printt(char *p )
{
  while( *p != '\0' ){
  usart_send(*p);p++;}
}
//************************************************//
//            发送int变量               				  //
//************************************************//
void UART_Send_int(int16_t Data)
{ 
  if(Data<0)
	{
	usart_send(0x2D);
	Data=-Data;
	}
	usart_send(Data/10000+48);	   
	Data=Data%10000;
	usart_send(Data/1000+48);
	Data=Data%1000;
	usart_send(Data/100+48);
	Data=Data%100;
	usart_send(Data/10+48);
	usart_send(Data%10+48);
}
//************************************************//
//          发送float变量               				  //
//************************************************//
void UART_Send_float( float DATA)
{
  int32_t Data;
	Data=DATA*100;
	if(Data<0)
	{
	usart_send(0x2D);
	Data=-Data;
	}	   
	usart_send(Data/100000+48);
	Data=Data%100000;
	usart_send(Data/10000+48);
	Data=Data%10000;
	usart_send(Data/1000+48);
	Data=Data%1000;
	usart_send(Data/100+48);
	Data=Data%100;
	usart_send(0x2E);
	usart_send(Data/10+48);
	usart_send(Data%10+48);
}
//void USART1_IRQHandler(void)                	            //串口1中断服务程序
//{
//	uint8_t Res;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)   //接收中断
//  {
//	Res =USART_ReceiveData(USART1);                       	//读取接收到的数据
//	usart_send(Res);
//	}
//}
unsigned char Uart1_Put_Char( unsigned char Send_Data )
{
	usart_send(Send_Data);
	return Send_Data;
}
unsigned char UART1_Put_int(unsigned int Send_Data )
{
	unsigned char sum = 0;
	sum +=Uart1_Put_Char( Send_Data/256 );
	sum +=Uart1_Put_Char( Send_Data%256 );
	return sum;
}
void UART1_ReportIMU(int ax,int ay,int az,int gx,int gy,int gz,int hx,int hy,int hz,int roll,int pitch,int yaw)
{
 	unsigned char sum = 0;
	sum += Uart1_Put_Char(0x88);
	sum += Uart1_Put_Char(0xAF);
	sum += Uart1_Put_Char(0x1C);
	sum += UART1_Put_int( ax );
	sum += UART1_Put_int( ay );
	sum += UART1_Put_int( az );
	sum += UART1_Put_int( gx );
	sum += UART1_Put_int( gy );
	sum += UART1_Put_int( gz );
	sum += UART1_Put_int( hx );
	sum += UART1_Put_int( hy );
	sum += UART1_Put_int( hz );
	sum += UART1_Put_int( roll );
	sum += UART1_Put_int( pitch );
	sum += UART1_Put_int( yaw );
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	Uart1_Put_Char(sum);
}
void UART1_ReportCON(int throt,int yaw,int roll,int pitch,int aux1,int aux2,int aux3,int aux4,int aux5,int pwm1,int pwm2,int pwm3,int pwm4,int votage)
{
 	unsigned char sum = 0;
	sum += Uart1_Put_Char(0x88);
	sum += Uart1_Put_Char(0xAE);
	sum += Uart1_Put_Char(0x12);
	sum += UART1_Put_int( throt );
	sum += UART1_Put_int( yaw );
	sum += UART1_Put_int( roll );
	sum += UART1_Put_int( pitch );
	sum += UART1_Put_int( aux1 );
	sum += UART1_Put_int( aux2 );
	sum += UART1_Put_int( aux3 );
	sum += UART1_Put_int( aux4 );
	sum += UART1_Put_int( aux5 );
	sum += UART1_Put_int( pwm1 );
	sum += UART1_Put_int( pwm2 );	
	sum += UART1_Put_int( pwm3 );
	sum += UART1_Put_int( pwm4 );
	sum += UART1_Put_int( votage );	
	Uart1_Put_Char(sum);
}
