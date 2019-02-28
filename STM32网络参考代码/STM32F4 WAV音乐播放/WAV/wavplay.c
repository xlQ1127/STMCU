#include "wavplay.h"
#include "delay.h"
#include "timer.h"
#include "lcd.h"
#include "dac.h"
#include "key.h"

WAV_file wav1;//wav文件
u8 wav_buf[1024];
u16 DApc;
u8 CHanalnum;
u8 Bitnum;
u8 DACdone;
extern u8 volume;
FileInfoStruct *CurFile;//当前解码/操作的文件	 
u8 WAV_Init(u8* pbuf)//初始化并显示文件信息
{
	if(Check_Ifo(pbuf,"RIFF"))return 1;//RIFF标志错误
	wav1.wavlen=Get_num(pbuf+4,4);//文件长度，数据偏移4byte
	if(Check_Ifo(pbuf+8,"WAVE"))return 2;//WAVE标志错误
	if(Check_Ifo(pbuf+12,"fmt "))return 3;//fmt标志错误
	wav1.formart=Get_num(pbuf+20,2);//格式类别
	wav1.CHnum=Get_num(pbuf+22,2);//通道数
	CHanalnum=wav1.CHnum;
	wav1.SampleRate=Get_num(pbuf+24,4);//采样率
	wav1.speed=Get_num(pbuf+28,4);//音频传送速率
	wav1.ajust=Get_num(pbuf+32,2);//数据块调速数
	wav1.SampleBits=Get_num(pbuf+34,2);//样本数据位数
	Bitnum=wav1.SampleBits;
	if(Check_Ifo(pbuf+36,"data"))return 4;//data标志错误
	wav1.DATAlen=Get_num(pbuf+40,4);//数据长度	
	///////////////////////////////////////////////
	if(wav1.wavlen<0x100000)
	{
		LCD_ShowNum(170,30,(wav1.wavlen)>>10,3,16);//文件长度
		LCD_ShowString(200,30,"Kb");
	}
	else
	{
		LCD_ShowNum(170,30,(wav1.wavlen)>>20,3,16);//文件长度
		LCD_ShowString(200,30,"Mb");
	}
	if(wav1.formart==1)LCD_ShowString(170,50,"WAV PCM");
	if(wav1.CHnum==1)LCD_ShowString(170,70,"single");
	else LCD_ShowString(170,70,"stereo");
	LCD_ShowNum(170,90,(wav1.SampleRate)/1000,3,16);//采样率
	LCD_ShowString(200,90,"KHz");
	LCD_ShowNum(170,110,(wav1.speed)/1000,3,16);//传送速度
	LCD_ShowString(200,110,"bps");
	LCD_ShowNum(177,130,wav1.SampleBits,2,16);//样本数据位数
	LCD_ShowString(200,130,"bit");
	return 0;
}

u8 Playwav(FileInfoStruct *FileName)
{
	u16 i,times;
	CurFile=FileName;
	if(CurFile->F_Type!=T_WAV)return 1;
	F_Open(CurFile);
	F_Read(CurFile,wav_buf);//先读512字节到
	F_Read(CurFile,wav_buf+512);//再读512字节
	while(WAV_Init(wav_buf))LCD_ShowString(35,70,"format illegal!");
	//根据采样率（wav1.SampleRate）设置定时器，在中断中进行DA转换
	DACdone=0;
	DApc=44;//DA转换地址(跳过头信息)
	LCD_DrawRectangle(18,258,222,272);//进度框
	LCD_Fill(20,260,220,270,WHITE);//进度条
	Timerx_Init(1000000/wav1.SampleRate,72);//1MHz的计数频率,产生和采样率一样的中断频率
	times=(wav1.DATAlen>>10)-1;
	for(i=0;i<times;i++)//循环一次转换1KB数据
	{	
		while(!DACdone);//等待前面512字节转换完成
		DACdone=0;
		F_Read(CurFile,wav_buf);//读512字节
		LCD_Fill(20,260,20+(200*i/times),270,BLUE);//进度条
		while(!DACdone);//等待前面512字节转换完成
		DACdone=0;
		F_Read(CurFile,wav_buf+512);//读512字节	
		if((KEY1&KEY2&KEY3&KEY4)==0)
		{			
			if((KEY1&KEY2)==0){TIM3->CR1&=~0x01;break;}//关定时器
			else if(KEY3==0&&volume<255)volume++;
			else if(KEY4==0&&volume>10)volume--;				  
		}
	}
	return 0;
}

u8 Check_Ifo(u8* pbuf1,u8* pbuf2)
{
	u8 i;
	for(i=0;i<4;i++)if(pbuf1[i]!=pbuf2[i])return 1;//不同
	return 0;//相同
}

u32 Get_num(u8* pbuf,u8 len)
{
    u32 num;
	if(len==2)num=(pbuf[1]<<8)|pbuf[0];
	else if(len==4)num=(pbuf[3]<<24)|(pbuf[2]<<16)|(pbuf[1]<<8)|pbuf[0];
	return num;
}




