/***********************************************

标题: mian.c
作者: 秋阳电子
网址：http://qiuyangdz.taobao.com
日期: 2014/05/18
版本：v1.0
MDK-ARM 版本: v4.12
ST 库版本：v3.50  
功能: 四轴飞控
说明：
*************************************************/
#include "stm32f10x.h"
#include "iic.h"
#include "timer.h"
#include "usart.h"
#include "mpu6050.h"
#include "nrf24l01.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"
#include <stdio.h>

#define  COM_MODEL	   0x00
#define  NORMAL_MODEL  0x01
#define  BUT  (GPIOB->IDR  & GPIO_Pin_12)
#define  LED_BLINK  GPIOA->ODR ^= (1 << 1)
#define  TIME_OUT  1200

u8 model = NORMAL_MODEL;
u8 flg_get_senor_data;
u8 flg_recive_data;
u8 out[35] = {0x5f, 0x60, 0};
u8 offset_data[5];
u8 rx_data_buf[16]; 
s16 gx, gy, gz, ax ,ay, az, temperature;
float set_pitch, set_roll, yaw, yaw1, pitch, roll, left_yaw, right_yaw; 
float f_gx, f_gy, f_gz;
s16 temp;
u16 time_out = 0;
u8 send_fre = 0;
volatile u8 thr = 0, rud = 0, ele = 0, ail = 0;
volatile float yaw_offset = 0, pitch_offset = 0, roll_offset = 0;
s16 gx_offset = 0, gy_offset = 0, gz_offset = 0; 
s32 gx_sum = 0, gy_sum = 0, gz_sum = 0;
/*************************************************

名称：void delay(u32 count)
功能：延时
输入参数：延时数
输出参数：无
返回值：  无
**************************************************/
void delay(u32 count)
{
  for(; count != 0; count--);
}
/*************************************************

名称：void led_blink()
功能：LED闪烁
输入参数：无
输出参数：无
返回值：  无
**************************************************/
void led_blink()
{
  u8 j;
  for(j = 0; j < 30; j++)
  {
    LED_BLINK;
	delay(0x80000);
  }
}
/*************************************************

名称：receive_control_data()
功能：接收遥控器数据
输入参数：无
输出参数：油门控制等信息
返回值：无
**************************************************/
void receive_control_data()
{
  u16 check = 0;
  u8 i;
  if(nrf24l01_rx_data(&rx_data_buf[0]))    
  {
     for (i = 0; i < 15; ++i) 
	 {
	   check += rx_data_buf[i];
	 }
	 if((check & 0xff) == rx_data_buf[15])
	 {
       LED_BLINK;
       time_out = 0;
       thr = rx_data_buf[0];
       rud = rx_data_buf[1];
	   ele = rx_data_buf[2];
	   ail = rx_data_buf[3];

	   yaw_offset = (rx_data_buf[4] - 0x40) * 1.0;
	   pitch_offset = (rx_data_buf[5] - 0x40) * 0.25;
	   roll_offset = (rx_data_buf[6] - 0x40) * 0.25;
	 }
  }
  else  //接收命令超时
  {
     if(time_out++ > TIME_OUT)
     {
	   time_out = TIME_OUT;
	   thr = 0;
	   rud = 0;
	   ele = 0;
	   ail = 0;
     }
  } 
}
/*************************************************

名称：send_sensor_data()
功能：通过串口发送传感器及欧拉角数据
输入参数：无
输出参数：无
返回值：无
**************************************************/
void send_sensor_data()
{
  u8 j;

  LED_BLINK;

  out[2] = (u8)(gx >> 8);
  out[3] = (u8)(gx);
  out[4] = (u8)(gy >> 8);
  out[5] = (u8)(gy);
  out[6] = (u8)(gz >> 8);
  out[7] = (u8)(gz);
  out[8] = (u8)(ax >> 8);
  out[9] = (u8)(ax);
  out[10] = (u8)(ay >> 8);
  out[11] = (u8)(ay);
  out[12] = (u8)(az >> 8);
  out[13] = (u8)(az);

  temp = (s16)(pitch * 100); 
  out[20] = (u8)(temp >> 8);
  out[21] = (u8)(temp);

  temp = (s16)(roll * 100);
  out[22] = (u8)(temp >> 8);
  out[23] = (u8)(temp);

  temp = (s16)(yaw1 * 100);
  out[24] = (u8)(temp >> 8);
  out[25] = (u8)(temp);

  for(j = 0; j < 26; j++)
  {
    USART_SendData(USART1, out[j]);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  }
}
/*************************************************

名称：main(void)
功能：主函数
输入参数：无
输出参数：无
返回值：无
**************************************************/
int main(void)
{
  u16 k;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);  // BLUE LED

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);  // BUT

  GPIO_SetBits(GPIOA, GPIO_Pin_1);

  motor_init();

  delay(800000);

  timer_init();
  iic_init();
  mpu6050_init();
  usart_init();
  nrf24l01_init();
  pid_config();
  
  k = 1500;
  while(k)  //模式选择时间
  {
    if(flg_get_senor_data)
    {
      flg_get_senor_data = 0;
	  k--;

	  mpu6050_get_data(&gx, &gy, &gz, &ax, &ay , &az, &temperature);

	  gx_sum += gx;
	  gy_sum += gy;
	  gz_sum += gz;

	  f_gx = gx * GYRO_SCALE;
	  f_gy = gy * GYRO_SCALE;
	  f_gz = gz * GYRO_SCALE;

	  get_euler_angle(f_gx, f_gy, f_gz, ax, ay, az, &pitch, &roll);

	  if(BUT)
      {
 	    model = COM_MODEL;
	    led_blink();
	    k = 0;
      }
    }
  }

  gx_offset = gx_sum / 1500;
  gy_offset	= gy_sum / 1500;
  gz_offset	= gz_sum / 1500;

		while (1)
		{
					if(flg_get_senor_data)
					{
						flg_get_senor_data = 0;

					mpu6050_get_data(&gx, &gy, &gz, &ax, &ay , &az, &temperature);

					gx -= gx_offset;
					gy -= gy_offset;
					gz -= gz_offset; 

					f_gx = gx * GYRO_SCALE;
					f_gy = gy * GYRO_SCALE;
					f_gz = gz * GYRO_SCALE;

					get_euler_angle(f_gx, f_gy, f_gz, ax, ay, az, &pitch, &roll);

					yaw = -gz;
					yaw1 = -f_gz; 
					
						if(model == NORMAL_MODEL)  //正常模式
						{
							receive_control_data();

							pitch += pitch_offset;
							roll -= roll_offset; 
							yaw += yaw_offset;
						 
						 calculate(pitch, roll, yaw, thr, rud, ele, ail, f_gx, f_gy, f_gz);
						}
							else if(model == COM_MODEL)  //串口传输模式 
							{
									if(send_fre++ > 4)
									{
										send_fre = 0;
										send_sensor_data();
									} 
							}
					}							   
		}  // end while
}
/************************END OF FILE************************************************************/
