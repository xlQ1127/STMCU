/**
  ******************************************************************************
  * @file   CeFlash.h
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinks平台Flash对象的操作头文件
  ****************************************************************************** 
  * @attention
  *
  *1)不同Flash的读取、写入、擦除等操作要求的块/扇区/页大小不一致，用户需仔细查看
  *  CeMcu.h中CE_FLASH_SIZE、CE_FLASH_READ_SIZE、CE_FLASH_WRITE_SIZE、
  *  CE_FLASH_ERASE_SIZE 的值。
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_FLASH_H__
#define __CE_FLASH_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  结构体，FLASH对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*write)(uint32 addressIndex, uint8* dataInBuf, uint32 writeCount);/*!<
                                                     @brief 向Flash中写数据，需要保证此区域已被擦除，否则写入操作失败！
                                                     @param address:要定入的首地址，范围0到(CE_FLASH_DATA_SIZE - 1)；且必须为 CE_FLASH_WRITE_SIZE的整数倍-1，否则返回参数错误
                                                     @param dataInBuf:要写入的数据缓冲区
                                                     @param writeCount:要写入的数据长度，必须是 CE_FLASH_WRITE_SIZE的整数倍-1，否则返回参数错误
                                                     @return 返回CE_SUCCESS则表示写入成功，返回 CE_STATUS_PAR_ERROR 表示参数错误，返回 CE_STATUS_FAILE 表示操作失败*/

    uint8*      (*read)(uint32 addressIndex, uint8* dataOutBuf, uint32 readCount);/*!<
                                                     @brief 从Flash中读数据
                                                     @param address:要读取的首地址，范围0到(CE_FLASH_DATA_SIZE - 1)；且必须为 CE_FLASH_READ_SIZE的整数倍-1
                                                     @param dataOutBuf:读取的数据存放的缓冲区
                                                     @param readCount:要读取的数据长度，必须 CE_FLASH_READ_SIZE的整数倍-1，返回参数错误
                                                     @return 读取数据缓存*/

    CE_STATUS   (*erase)(uint32 addressIndex, uint32 eraseCount);/*!<
                                                     @brief 对Flash指定地址进行擦除操作
                                                     @param address:要擦除的首地址，范围0到(CE_FLASH_DATA_SIZE - 1)；且必须为 CE_FLASH_ERASE_SIZE的整数倍-1，否则返回参数错误
                                                     @param eraseCount:要擦除的长度，必须是 CE_FLASH_ERASE_SIZE的整数倍-1，否则返回参数错误
                                                     @return 返回CE_SUCCESS则表示擦除成功，返回CE_STATUS_FAILE表示擦除失败*/
}CeFlashOp;
extern const CeFlashOp ceFlashOp;          /*!< 所有与Flash相关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_FLASH_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境)
* @function 读取Flash口转换值，并通过Uart口传输给串口调试助手
******************************************************************************
#include "Creelinks.h"
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                        //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作       
    };
}
******************************************************************************
*/
