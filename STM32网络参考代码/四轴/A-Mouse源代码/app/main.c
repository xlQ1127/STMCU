#include "queue.h"
#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"   
#include "adc.h" 
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_driver.h" 

void usart2_send_char(u8 c)
{
	while((USART2->SR&0X40)==0);
	USART2->DR=c;   	
} 

void usart1_send_char(u8 c)
{
	while((USART1->SR&0X40)==0);
	USART1->DR=c;   	
} 

//=======================发送adc===========================

void adc_send_data(u16 adc[])
{
	u8 i;
	u8 tbuf[12]; 
	tbuf[0]=(adc[0]>>8)&0XFF;
	tbuf[1]=adc[0]&0XFF;
	tbuf[2]=(adc[1]>>8)&0XFF;
	tbuf[3]=adc[1]&0XFF;
	tbuf[4]=(adc[2]>>8)&0XFF;
	tbuf[5]=adc[2]&0XFF; 
	tbuf[6]=(adc[3]>>8)&0XFF;
	tbuf[7]=adc[3]&0XFF;
	tbuf[8]=(adc[4]>>8)&0XFF;
	tbuf[9]=adc[4]&0XFF;
	tbuf[10]=(adc[5]>>8)&0XFF;
	tbuf[11]=adc[5]&0XFF;
	//usart1_niming_report(0XA1,tbuf,12);//????,0XA1
	for(i=0;i<12;i++)usart1_send_char(tbuf[i]);
}	

//=======================发送加速度，角速度===========================

void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 i;
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	//usart1_niming_report(0XA1,tbuf,12);//????,0XA1
	for(i=0;i<12;i++)usart1_send_char(tbuf[i]);
}	

//============================数据手套数据发送===========
//数据还未处理
//控制不准确

void usart1_report_mouse(u8 key,u8 aacx,u8 aacy,u8 aacz)
{
	u8 tbuf[8]; 
	u8 i;
	for(i=0;i<8;i++)tbuf[i]=0;
	tbuf[0]=0x08;
	tbuf[1]=0x00;
	tbuf[2]=0xA1;
	tbuf[3]=0x02;
	tbuf[4]=key&0xFF;
	tbuf[5]=aacx&0xFF; 
	tbuf[6]=aacy&0xFF;
	tbuf[7]=aacz&0xFF;
	for(i=0;i<8;i++)usart1_send_char(tbuf[i]);
}   

//=====================键盘数据发送=====================
void usart_report_keyscan(u8 key0)
{
	u8 tbuf[12]; 
	u8 i;
	for(i=0;i<12;i++)tbuf[i]=0;
	tbuf[0]=0X0C;
	tbuf[1]=0X00;
	tbuf[2]=0XA1;
	tbuf[3]=0X01;
	tbuf[4]=0X00;
	tbuf[5]=0X00;
	tbuf[6]=key0;
	tbuf[7]=0X00;
	tbuf[8]=0X00;
	tbuf[9]=0X00;
	tbuf[10]=0X00;
	tbuf[11]=0X00;
	for(i=0;i<8;i++)usart1_send_char(tbuf[i]);
}


//======================给智能手机发送数据========================
void usart1_report_smartphone()
{
	u8 tbuf[8]; 
	u8 i;
	for(i=0;i<8;i++)tbuf[i]=0;
	tbuf[0]=0X08;
	tbuf[1]=0X00;
	tbuf[2]=0XA1;
	tbuf[3]=0X03;
	//tbuf[4]=(aacz>>8)&0XFF;
	//tbuf[5]=aacz&0XFF; 
	//tbuf[6]=(gyrox>>8)&0XFF;
	//tbuf[7]=gyrox&0XFF;
	for(i=0;i<8;i++)usart1_send_char(tbuf[i]);
}

//==========================数据手套位移数据处理=========================

void data_processing(short *ax,short *ay,short *az)
{
		float tempx,tempy,tempz,tempx1,tempy1,tempz1;
		tempx = *ax/1000-1;
		tempy = *ay/1000-1;
		tempz = *az/1000-1;
		tempx1 = *ax%1000/100;
		tempy1 = *ay%1000/100;
		tempz1 = *az%1000/100;
	
		if(tempz==-1 || tempz==-2)
		{*az = 0;}
		else if(tempz>-1)
		{*az = tempz*13+tempz1*2;}
		else
		{*az = (tempz+2)*13+tempz*2;}
		
		if(tempx==-1 ||tempx == -2)
		{*ax=0;}
		else if(tempx>-1)
		{*ax = tempx*13+tempx1*2;}
		else
		{*ax = (tempx+2)*13+tempx1*2;}
		
		if(tempy==-1 ||tempy == -2) 
		{*ay = 0;}
		else if(tempy>-1)
		{*ay = tempy*13+tempy1*2;}
		else
		{*ay = (tempy+2)*13+tempy1*2;}
}

//=========================数据手套按键数据处理=========================
u8 data_key_processing(u16 adc[])
{
	u8 key_adc = 0x00;
	//adc0: 1166 - 1930
	if(adc[0]<1600)
	{key_adc = key_adc|0x10;}
	if(adc[1]<2600)
		//adc1: 2305 - 3070
	{key_adc = key_adc|0x01;}
	if(adc[2]<2500)
		//adc2: 1787 2454
	{key_adc = key_adc|0x02;}
	if(adc[3]<2500)
		//adc3: 2041 - 3070
	{key_adc = key_adc|0x04;}
	if(adc[4]<2600)
		//adc4: 1932 - 2841
	{key_adc = key_adc|0x08;}
	return key_adc;
}

//==================LED亮灭控制===================
int led_commend(u8 led_k,u16 adc[])
{
		u8 key_led=0;
		int num=0;
		delay_ms(10);//
		Get_Adc_Average(adc,5);
		key_led = data_key_processing(adc);
		//LED1=!LED1;//
		//delay_ms(30);//
		if(led_k == key_led)
		{
			//LED1=!LED1;//
			//delay_ms(30);//
			//LED1=!LED1;//
				do{
				num++;
				if(num>3000)
						return 0;
				Get_Adc_Average(adc,5);
				key_led = data_key_processing(adc);
				//usart2_send_char(1);//
				//usart2_send_char(key_led);//
				}while(key_led != 0x1f && key_led != 0x00);
				num=0;
				if(key_led == 0x1f)
				{
					delay_ms(10);
					Get_Adc_Average(adc,5);
					key_led = data_key_processing(adc);
					if(key_led == 0x1f)
					{
						do{
						delay_ms(10);
						num++;
						if(num>3000)
							return 0;
						Get_Adc_Average(adc,5);
						key_led = data_key_processing(adc);
						//usart2_send_char(2);//
						//usart2_send_char(key_led);//
						}while(key_led != 0x00);
						delay_ms(10);
						Get_Adc_Average(adc,5);
						key_led = data_key_processing(adc);
						if(key_led == 0x00)
						{
							usart2_send_char(led_k|0x20);
							//LED1=!LED1;//
							//delay_ms(100);//
							//LED1=!LED1;//
						}
					}
				}
				else{
					delay_ms(10);
					Get_Adc_Average(adc,5);
					key_led = data_key_processing(adc);
					if(key_led == 0x00)
					{
						do{
						delay_ms(10);
						num++;
						if(num>3000)
							return 0;
						Get_Adc_Average(adc,5);
						key_led = data_key_processing(adc);
						//usart2_send_char(3);//
						//usart2_send_char(key_led);//
						}while(key_led != 0x1f);
						delay_ms(10);
						Get_Adc_Average(adc,5);
						key_led = data_key_processing(adc);
						if(key_led == 0x1f)
						{
							usart2_send_char(led_k|0x00);
							//LED0=!LED0;//
							//delay_ms(100);//
							//LED0=!LED0;//
						}
					}
				}
			}
		return 1;
}

//================手势处理=======================
void data_identification(short *data_az)
{
	if(*data_az<=15800 && *data_az>=14800)
		*data_az = 15300;
}


//=============主函数=================

int main(void)
 { 
	u8 t=0;			//??????report=1,
	// short i;
	u8 mouse_x,mouse_y,mouse_z;
	u16 adc[5];
	u8 key = 0x00;
	//u8 before_key = 0x00;
	//float pitch,roll,yaw; 		//???
	short aacx,aacy,aacz;		//??????????
	short gyrox,gyroy,gyroz;	//???????
	 short temp_gyroz=0;
	// short get_z;
	// 	short shibie_max1,shibie_max2,shibie_min1,shibie_min2;
	 //short read_all[150];
	//short temp;					//??	    
	//PQUEUE read_z;
	//PQUEUE read_all;
	//CreateQueue(read_all,150);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init();	    	   
	uart_init(9600);	
  uart2_init(9600);
	Adc_Init();
	LED_Init();		  			
	MPU_Init();
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能ADC1软件开始转换
	while(mpu_dmp_init())
	{
		delay_ms(200);		
	}
	//CreateQueue(read_z,20);
 	while(1)
	{
		Get_Adc_Average(adc,5);
		key = data_key_processing(adc);
		while(key&0x10 && !(key&0x08))
		{
				//usart1_report_mouse(key,0,0,0);
				//usart2_send_char(key);
				//if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
				//{ 
					//temp=MPU_Get_Temperature();	
				//MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
				//	gyrox=(int)(roll*100);
				//	gyroy=(int)(pitch*100);
				//	gyroz=(int)(yaw*10);
				data_processing(&gyrox,&gyroy,&gyroz);
				temp_gyroz = gyroz - temp_gyroz;
				mouse_x = gyrox&0xFF;
				mouse_y = gyroy&0xFF;
				temp_gyroz = gyroy&0xFF;
				if(gyrox&0x8000)
				{
					mouse_x = mouse_x|0x80;
				}
				if(gyroy&0x8000)
				{
					mouse_y = mouse_y|0x80;
				}
				if(gyroz&0x8000)
				{
					mouse_z = mouse_z|0x80;
				}
				usart1_report_mouse(key,mouse_y,mouse_x,0);
				Get_Adc_Average(adc,5);
				key = data_key_processing(adc);
				//usart2_send_char(key);
				//usart2_send_char(0);
		}
		switch (key)
		{
			case 0x1E:led_commend(key,adc); break;
			case 0x1C:led_commend(key,adc);break;
			case 0x18:led_commend(key,adc); break;
			case 0x1D:led_commend(key,adc); break;
			//case 0x1F:led_commend(key,adc); break;
			default: break;
		}
		
			if(t>10) 
			{LED1 = !LED1;t=0;}
			t++;
		//}
		delay_ms(20);
		LED0=!LED0;//LED闪烁
	} 	
}

