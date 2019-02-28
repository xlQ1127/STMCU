#include "stm32f10x.h"

/************************************************************************************

* 摘 要:扩展 1K FLASH 作为 EEPROMG  

*	 StartAddr--是FLASH存放数据的首地址，定义为数组，每隔2K设一个首地址
	 EndAddr--是FLASH存放数据的末地址，定义为数组，可以设定为当前页之内的任何一个地址，根据需要存放程序的大小来设定
	 *Data--要写入FLASH的数据，指向该数据所在内存的地址
			 

秋阳 2013-05-15
*************************************************************************************/
#define FLASH_PAGE_SIZE    ((u16)0x400) //1024  1K为一页 （0-3FF）
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;



u32 StartAddr =0x08008000;  //要写入Flash的数据的首地址
u32 EndAddr   =0x080083FF;//要写入Flash的数据的末地址

u32 FlashAddress=0x00;//Flash的内部地址	 
//u32 Data=0x12345678;	 //要写入的数据，指向该数据所在内存的地址


 
union 
{
u32 Data[8];
float F_Data[8];
}ftoc;



vu32 NbrOfPage = 0x00; //要擦除的页面数量

//u32 *p=(u32 *)0x08008000; //定义指针指向要传送的数据的地址

volatile FLASH_Status FLASHStatus;
volatile TestStatus MemoryProgramStatus;

ErrorStatus HSEStartUpStatus;





/*******************************************************************************
* Function Name  : Writeflash
* Description    : 写函数 把数据写入FLASH中
*                  先擦除1K然后写入

*******************************************************************************/
void Writeflash()
{ u8 num,t_num=0; 
	
	num=sizeof(ftoc.Data)/sizeof(ftoc.Data[0]);//计算出数组有几个元素
	
	FLASHStatus = FLASH_COMPLETE;
  MemoryProgramStatus = PASSED; 

    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock();	

    /* Clear All pending flags */
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);//清标志位
	
//擦除后写数据		
//*******************************************************************************  

      /* Define the number of page to be erased *///要擦的页面的数量：1
      //NbrOfPage = (EndAddr - StartAddr) / FLASH_PAGE_SIZE;
	   
       /* Erase the FLASH pages *///页面擦除,即0x8008000-0x8008F33
    	FLASHStatus = FLASH_ErasePage(StartAddr); 
	    
	    //写数据	
      FlashAddress = StartAddr;
		
			
	  
      while((FlashAddress < EndAddr) && (FLASHStatus == FLASH_COMPLETE))
      {
     	  if(t_num<num) FLASHStatus = FLASH_ProgramWord(FlashAddress, ftoc.Data[t_num]);
        else          FLASHStatus = FLASH_ProgramWord(FlashAddress, 0x1234);
        
				t_num++;
				FlashAddress = FlashAddress + 4;
      }

	 }


/*******************************************************************************
* Function Name  : Readflash
* Description    : 读数据，从FLASH中读出需要的数据
*                
* Input          : None
* Output         : Data输出要取出的数据
* Return         : None
*******************************************************************************/
/*
void Readflash(void)
{

    FlashAddress = StartAddr;
	//读数据

    Data=*p;
//    printf ("\r\n Bellow is the data from memory*************************************\n\r\n");

	while(p<( u32 *)EndAddr)//&&(p>(u32 *)StartAddr))
	{
//	    printf("%x",Data);

	    p++;
	}
	 
 
}
*/
