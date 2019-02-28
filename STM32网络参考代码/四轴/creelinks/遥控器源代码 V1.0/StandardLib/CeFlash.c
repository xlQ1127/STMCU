/**
  ******************************************************************************
  * @file   CeFlash.c
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinks平台的Flash对象库实现函数，基于STM32F103RET6平台
  ******************************************************************************
  * @attention
  *
  *1)所有Flash共用的同一个Flash转换模块，采取循环采集方式转换
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeFlash.h"
#ifdef __cplusplus
 extern "C" {
#endif  //__cplusplus

#ifdef __CE_USE_FLASH__

#define CE_FLASH_BASE_ADDRESS   (0x08000000 + CE_MCU_ROM_SIZE_KB - CE_FLASH_SIZE + 1)/*!< 根据MCU的ROM容量配置Flash基地址，必须保证CE_MCU_ROM_SIZE_KB的值正确*/

#ifdef __CE_CHECK_PAR__
/**
  * @brief  检查读取参数是否合法
  * @param  address:读取地址
  * @param  dataOutBuf:读取的数据缓存
  * @param  readCount：读取的数据长度
  * @return  合法返回CE_STATUS_SUCCESS，不合法返回CE_STATUS_PAR_ERROR
  */
CE_STATUS ceFlash_checkReadPar(uint32 addressIndex, uint8* dataOutBuf, uint32 readCount)
{
    if ((addressIndex + 1) % CE_FLASH_READ_SIZE != 0 || dataOutBuf == CE_NULL || readCount% CE_FLASH_READ_SIZE != 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
    return CE_STATUS_SUCCESS;
}

/**
* @brief  检查写入参数是否合法
* @param  address:写入地址
* @param  dataOutBuf:写入的数据缓存
* @param  readCount：写入的数据长度
* @return  合法返回CE_STATUS_SUCCESS，不合法返回CE_STATUS_PAR_ERROR
*/
CE_STATUS ceFlash_checkWritePar(uint32 addressIndex, uint8* dataInBuf, uint32 writeCount)
{
    if ((addressIndex + 1) % CE_FLASH_WRITE_SIZE != 0 || dataInBuf == CE_NULL || writeCount% CE_FLASH_WRITE_SIZE != 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
    return CE_STATUS_SUCCESS;
}

/**
* @brief  检查擦除参数是否合法
* @param  address:擦除地址
* @param  readCount：擦除的数据长度
* @return  合法返回CE_STATUS_SUCCESS，不合法返回CE_STATUS_PAR_ERROR
*/
CE_STATUS ceFlash_checkErasePar(uint32 addressIndex, uint32 wipeCount)
{
    if ((addressIndex + 1) % CE_FLASH_ERASE_SIZE != 0 ||  wipeCount% CE_FLASH_WRITE_SIZE != 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif

/**
* @brief   由系统调用的初始化Flash转换
* @return  系统状态码，可能的返回值:CE_STATUS_SUCCESS,CE_STATUS_INITIAL_FALSE
*/
CE_STATUS ceFlash_initialBySystem(void)
{
    return CE_STATUS_SUCCESS;//在STM32F103中，为空。其它处理器根据实际情况制定
}

/**
  * @brief 从Flash中读数据
  * @param address:要读取的首地址，范围0到(CE_FLASH_DATA_SIZE - 1)；且必须为 CE_FLASH_READ_SIZE的整数倍-1
  * @param dataOutBuf:读取的数据存放的缓冲区
  * @param readCount:要读取的数据长度，必须 CE_FLASH_READ_SIZE的整数倍，返回参数错误
  * @return 读取数据缓存
  */
uint8* ceFlash_read(uint32 addressIndex, uint8* dataOutBuf, uint32 readCount)
{
    uint32 i = 0;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceFlash_checkReadPar( addressIndex, dataOutBuf,  readCount));
#endif //__CE_CHECK_PAR__

    for(i = 0; i < readCount; i+= 2)//已知STM32F103一次最小读取量是2个字节
    {
        uint16 readVal = *(volatile uint16*)(addressIndex+ CE_FLASH_BASE_ADDRESS + i);
        dataOutBuf[i] = (uint8)((readVal >> 8) & 0xFF);
        dataOutBuf[i+1] = (uint8)((readVal>> 0) & 0xFF);
    }
    return dataOutBuf;
}

/**
  * @brief 向Flash中写数据，需要保证此区域已被擦除(此区域中的数据位(bit)都必须为1)，否则返回操作失败！
  * @param address:要定入的首地址，范围0到(CE_FLASH_DATA_SIZE - 1)；且必须为 CE_FLASH_WRITE_SIZE 的整数倍-1，否则返回参数错误
  * @param dataInBuf:要写入的数据缓冲区
  * @param writeCount:要写入的数据长度，范围0到(CE_FLASH_DATA_SIZE - 1)；若不是 CE_FLASH_WRITE_SIZE 的整数倍-1，返回参数错误
  * @return 返回CE_SUCCESS则表示写入成功，返回 CE_STATUS_PAR_ERROR 表示参数错误，返回 CE_STATUS_FAILE 表示操作失败
  */
CE_STATUS ceFlash_write(uint32 addressIndex, uint8* dataInBuf, uint32 writeCount)
{
    uint32 i = 0;
    #ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceFlash_checkWritePar(addressIndex, dataInBuf, writeCount));
    #endif //__CE_CHECK_PAR__

    FLASH_Unlock();         //解锁写保护
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    for (i = 0; i < writeCount; i += 2)
    {
        FLASH_ProgramHalfWord((addressIndex + CE_FLASH_BASE_ADDRESS + i), ((uint16)(dataInBuf[i]) << 8 | (uint16)(dataInBuf[i + 1])));
    }
    FLASH_Lock();//上锁写保护
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  对Flash指定地址进行擦除操作
  * @param  address:要擦除的首地址，范围0到(CE_FLASH_DATA_SIZE - 1)；且必须为 CE_FLASH_ERASE_SIZE 的整数倍-1，否则返回参数错误
  * @param  wipeCount:要擦除的长度，范围0到(CE_FLASH_DATA_SIZE - 1)；且必须为 CE_FLASH_ERASE_SIZE 的整数倍-1，否则返回参数错误
  * @return  返回CE_SUCCESS则表示擦除成功，返回CE_STATUS_FAILE表示擦除失败
  */
CE_STATUS ceFlash_erase(uint32 addressIndex, uint32 eraseCount)
{
    #ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceFlash_checkErasePar(addressIndex, eraseCount));
    #endif //__CE_CHECK_PAR__
    FLASH_Unlock();         //解锁写保护
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(CE_FLASH_BASE_ADDRESS + eraseCount);//擦除这个扇区
    FLASH_Lock();//上锁写保护
    return CE_STATUS_SUCCESS;
}

const CeFlashOp ceFlashOp = {ceFlash_write, ceFlash_read, ceFlash_erase };

#endif  //__CE_USE_FLASH__

#ifdef __cplusplus
 }
#endif  //__cplusplus
