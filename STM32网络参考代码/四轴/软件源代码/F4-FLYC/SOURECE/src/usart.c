#include "usart.h"

void usart_x_init(u32 BaudRate)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
  
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);					//开启外设时钟

  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);  
  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);


  	USART_InitStructure.USART_BaudRate = BaudRate;					//波特率设置
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  	USART_InitStructure.USART_Parity = USART_Parity_No;
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
  	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);  				//关闭发送中断      
    USART_ClearFlag(USART3,USART_FLAG_TC);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);					//使能接收中断

	USART_ITConfig(USART3, USART_IT_PE, ENABLE);    //开启PE错误接收中断Bit 8PEIE: PE interrupt enable
  
  	USART_ITConfig(USART3, USART_IT_ERR, ENABLE);		//CR2 开启ERR中断

  	USART_Cmd(USART3, ENABLE);	  									//启动UART3

}
void USART3_SendData(unsigned char ch)
{

		USART_SendData(USART3, (u8) ch);
  		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
}

void SendData(signed int A, signed int B, signed int C, signed int D)
{
  uint8_t check_byte;
  USART3_SendData(0x55);
  USART3_SendData(0x34);
  
  USART3_SendData(0x08);                                 //数据长度 8 byte
  
	//if(A<0)A=32768-A;
  USART3_SendData(A>>8);
  USART3_SendData(A);
	
	//if(B<0)B=32768-B;
  USART3_SendData(B>>8);
  USART3_SendData(B);
	
	//if(C<0)C=32768-C;
  USART3_SendData(C>>8);
  USART3_SendData(C);
	
	//if(D<0)D=32768-D;
  USART3_SendData(D>>8);
  USART3_SendData(D);
  
  check_byte  = (A>>8)^A^(B>>8)^B^(C>>8)^C^(D>>8)^D;                            //简单校验 异或所有数据
  
  USART3_SendData(check_byte);
}


#if 1
#pragma import(__use_no_semihosting)                              
struct __FILE 
{ 
	int handle; 
}; 
FILE __stdout;         
_sys_exit(int x) 
{ 
	x = x; 
} 
int fputc(int ch, FILE *f)
{
  	USART_SendData(USART3, (u8) ch);
  	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
  	return ch;
}
#endif
