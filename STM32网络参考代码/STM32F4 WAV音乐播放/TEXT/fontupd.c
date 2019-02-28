#include "fontupd.h"
#include "sys.h"
#include "fat.h"
#include "flash.h"
#include "usart.h"
#include "delay.h"
#include "lcd.h"
#include "spi.h"

//Mini STM32开发板
//中文汉字支持程序 V1.1
//包括字体更新,以及字库首地址获取2个函数.
//正点原子@ALIENTEK
//2010/5/23	

//以下下字段一定不要乱改!!!
//字节0~3,  记录UNI2GBKADDR;字节4~7  ,记录UNI2GBKADDR的大小
//字节8~11, 记录FONT16ADDR ;字节12~15,记录FONT16ADDR的大小
//字节16~19,记录FONT12ADDR ;字节20~23,记录FONT12ADDR的大小		    
//字节24,用来存放字库是否存在的标志位,0XAA,表示存在字库;其他值,表示字库不存在!

extern unsigned char *folder[];	 //系统文件夹
extern unsigned char *sysfile[]; //系统图标

//字节0~3,  记录UNI2GBKADDR;字节4~7  ,记录UNI2GBKADDR的大小
//字节8~11, 记录FONT16ADDR ;字节12~15,记录FONT16ADDR的大小
//字节16~19,记录FONT12ADDR ;字节20~23,记录FONT12ADDR的大小
//字体存放,从100K处开始
//100K,存放UNICODE2GBK的转换码	

u32 FONT16ADDR ;//16字体存放的地址
u32 FONT12ADDR ;//12字体存放的地址
u32 UNI2GBKADDR;//UNICODE TO GBK 表存放地址	  	 

//更新字体文件
//返回值:0,更新成功;
//		 其他,错误代码.
//正点原子@ALIENTEK
//V1.1
#ifdef EN_UPDATE_FONT
u8 temp[512];  //零时空间
u8 Update_Font(void)
{
	u32 fcluster=0;	   
	u32 i;
	//u8 temp[512];  //零时空间	 在这里定义,会内存溢出
	u32 tempsys[2];  //临时记录文件起始位置和文件大小
	float prog;
	u8 t;		 
	FileInfoStruct FileTemp;//零时文件夹   				    	 
	//得到根目录的簇号
	if(FAT32_Enable)fcluster=FirstDirClust;
	else fcluster=0;			   
	FileTemp=F_Search(fcluster,(unsigned char *)folder[0],T_FILE);//查找system文件夹	  
	if(FileTemp.F_StartCluster==0)return 1;						  //系统文件夹丢失			  
	{	 										 
		//先查找字体
		FileTemp=F_Search(FileTemp.F_StartCluster,(unsigned char *)folder[1],T_FILE);//在system文件夹下查找FONT文件夹
		if(FileTemp.F_StartCluster==0)return 2;//字体文件夹丢失	

		fcluster=FileTemp.F_StartCluster;      //字体文件夹簇号	    
		FileTemp=F_Search(fcluster,(unsigned char *)sysfile[2],T_SYS);//在system文件夹下查找SYS文件
		if(FileTemp.F_StartCluster==0)return 3;//FONT12字体文件丢失	 
		LCD_ShowString(20,90,"Write UNI2GBK to FLASH...");		
		LCD_ShowString(108,110,"%");		
		F_Open(&FileTemp);//打开该文件
		i=0;	  
		while(F_Read(&FileTemp,temp))//成功读出512个字节
		{		 
			if(i<FileTemp.F_Size)//不超过文件大小
			{
				SPI_Flash_Write(temp,i+100000,512);//从100K字节处开始写入512个数据   
				i+=512;//增加512个字节
			}
			prog=(float)i/FileTemp.F_Size;
			prog*=100;
			if(t!=prog)
			{
				t=prog;
				if(t>100)t=100;
				LCD_ShowNum(84,110,t,3,16);//显示数值
			}					    
		}
		UNI2GBKADDR=100000;//UNI2GBKADDR从100K处开始写入.
		tempsys[0]=UNI2GBKADDR;
		tempsys[1]=FileTemp.F_Size;	 	  //UNI2GBKADDR 大小
		SPI_Flash_Write((u8*)tempsys,0,8);//记录在地址0~7处

		delay_ms(1000);			    
		//printf("UNI2GBK写入FLASH完毕!\n");
		//printf("写入数据长度:%d\n",FileTemp.F_Size);
		//printf("UNI2GBKSADDR:%d\n\n",UNI2GBKADDR);
		
		FONT16ADDR=FileTemp.F_Size+UNI2GBKADDR;//F16的首地址 
		FileTemp=F_Search(fcluster,(unsigned char *)sysfile[0],T_FON);//在system文件夹下查找FONT16字体文件
		if(FileTemp.F_StartCluster==0)return 4;//FONT16字体文件丢失	 
	
		LCD_ShowString(20,90,"Write FONT16 to FLASH... ");		
 		//printf("开始FONT16写入FLASH...\n");		
		F_Open(&FileTemp);//打开该文件
		i=0;	  
		while(F_Read(&FileTemp,temp))//成功读出512个字节
		{
			if(i<FileTemp.F_Size)//不超过文件大小
			{
				SPI_Flash_Write(temp,i+FONT16ADDR,512);//从0开始写入512个数据   
				i+=512;//增加512个字节
			}
			prog=(float)i/FileTemp.F_Size;
			prog*=100;
			if(t!=prog)
			{
				t=prog;
				if(t>100)t=100;
				LCD_ShowNum(84,110,t,3,16);//显示数值
			}
		}												   
		tempsys[0]=FONT16ADDR;
		tempsys[1]=FileTemp.F_Size;	 	  //FONT16ADDR 大小
		SPI_Flash_Write((u8*)tempsys,8,8);//记录在地址8~15处
		
		delay_ms(1000);	    
		//printf("FONT16写入FLASH完毕!\n");
		//printf("写入数据长度:%d\n",FileTemp.F_Size);


		FONT12ADDR=FileTemp.F_Size+FONT16ADDR;//F16的首地址
		//printf("FONT16SADDR:%d\n\n",FONT16ADDR);
		//LCD_ShowString(20,60,"Write FONT12 to FLASH... ");		
		//FONT12暂时不加入
		/*			  
		FileTemp=F_Search(fcluster,(unsigned char *)sysfile[1],T_FON);//在system文件夹下查找FONT12字体文件
		if(FileTemp.F_StartCluster==0)return 5;//FONT12字体文件丢失	 
		printf("开始FONT12写入FLASH...\n");		
		F_Open(&FileTemp);//打开该文件
		i=0;	  
		while(F_Read(&FileTemp,temp))//成功读出512个字节
		{
			if(i<FileTemp.F_Size)//不超过文件大小
			{
				SPI_Flash_Write(temp,i+FONT12ADDR,512);//从0开始写入512个数据   
				i+=512;//增加512个字节
			}
			prog=(float)i/FileTemp.F_Size;
			prog*=100;
			if(t!=prog)
			{
				t=prog;
				if(t>100)t=100;
				LCD_ShowNum(84,80,t,3,16);//显示数值
			}
		}	    
		tempsys[0]=FONT12ADDR;
		tempsys[1]=FileTemp.F_Size;	 	  //FONT16ADDR 大小
		SPI_Flash_Write((u8*)tempsys,16,8);//记录在地址16~23处

		printf("FONT12写入FLASH完毕!\n");
		printf("写入数据长度:%d\n",FileTemp.F_Size);   
		printf("FONT12SADDR:%d\n\n",FONT12ADDR); */
	}
	t=0XAA;
   	SPI_Flash_Write(&t,24,1);//写入字库存在标志	0XAA
	LCD_ShowString(20,90,"  Font Update Successed  ");		    
	delay_ms(1000);		
	delay_ms(1000);		
	return 0;//成功
}
#endif

//用这个函数得到字体地址
//在系统使用汉字支持之前,必须调用该函数
//包括FONT16ADDR,FONT12ADDR,UNI2GBKADDR;  
u8 Font_Init(void)
{
	u32 tempsys[2];//临时记录文件起始位置和文件大小
	u8 fontok=0;
	//SPIx_SetSpeed(SPI_SPEED_16);//设置到低速模式
	SPI_Flash_Read(&fontok,24,1);//读出字库标志位,看是否存在字库
	if(fontok!=0XAA)return 1;
	SPI_Flash_Read((u8*)tempsys,0,8);//读出8个字节   
	UNI2GBKADDR=tempsys[0];					  
	SPI_Flash_Read((u8*)tempsys,8,8);//读出8个字节   
	FONT16ADDR=tempsys[0];	
	SPI_Flash_Read((u8*)tempsys,16,8);//读出8个字节   
	return 0;	 
}





























