/**
  ******************************************************************************
  * @file    CeExtra.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   与处理器平台无关的CeExtra资源函数实现库文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeExtra.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief   对参数取绝对值
  * @param   val:需要计算的值
  * @return  计算结果
  */
fp32 ceAbs(fp32 val)
{
    if(val < 0)
    {
        return -val;
    }else
    {
        return val;
    }
}

const CeMathOp ceMathOp = {ceAbs};

/**
  * @brief   计算字符串长度，不计算'\0'
  * @param   str:需要计算的值
  * @return  字符串长度
  */
uint32 ceStrlen(const char* str)
{
    uint32 i=0;
    while(1)
    {
        if(str[i] == '\0')
        {
            return i;
        }
        i++;
    }
}

/**
  * @brief   字符串拷贝
  * @param   dest:拷贝的目标字符串
  * @param   src:拷贝的源字符串
  * @return  返回目标字符串的地址
  */
char*  ceStrcpy(char *dest, char *src)
{
    uint16 i;
    for (i = 0; i < 65535; i++)
    {
        dest[i] = src[i];
        if (src[i] == '\0')
        {
            break;
        }
    }
    return dest;
}

/**
  * @brief   字符串比较
  * @param   str1:欲比较的字符串1
  * @param   str2:欲比较的字符串2
  * @return  若两个字符串相同返回0，不同返回-1
  */
int8 ceStrcmp(const char* str1, const char* str2)
{
    uint16 i;
    for (i = 0; i < 65535; i++)
    {
        if (str1[i] == str2[i])
        {
            if (str1[i] == '\0')
            {
                return 0;//is same
            }
        }
        else
        {
            if (str1[i] == '\0' || str2[i] == '\0')
            {
                return -1;
            }
        }
    }
    return 0;
}

/**
  * @brief   将整数转化为字符串
  * @param   val:整数
  * @param   str:字符串缓存
  * @param   radix:需要转换的进制
  * @return  转换完成后的字符串
  */
char* ceItoa(int32 val, char* charBuff, uint8 radix)
{
    char index[] = "0123456789ABCDEF";
    unsigned unum;/*中间变量*/
    char temp;
    int i = 0, j, k;
    /*确定val的值*/
    if (radix == 10 && val < 0)/*十进制负数*/
    {
        unum = (unsigned) -val;
        charBuff[i++] = '-';
    }
    else
        unum = (unsigned) val;/*其他情况*/
    /*转换*/
    do
    {
        charBuff[i++] = index[unum % (unsigned) radix];
        unum /= radix;
    } while (unum);
    charBuff[i] = '\0';
    /*逆序*/
    if (charBuff[0] == '-')
        k = 1;/*十进制负数*/
    else
        k = 0;

    for (j = k; j <= (i - 1) / 2; j++)
    {
        temp = charBuff[j];
        charBuff[j] = charBuff[i - 1 + k - j];
        charBuff[i - 1 + k - j] = temp;
    }
    return charBuff;

}

/**
  * @brief   将字符串转换为整数
  * @param   str:要转换的字符串
  * @return  转换后的整数
  */
int32 ceAtoi(const char* str)
{
    uint8  bMinus = 0x00;
    int result = 0;
    if (('0' > *str || *str > '9') && (*str == '+' || *str == '-'))
    {
        if (*str == '-')
            bMinus = 0x01;
        str++;
    }
    while (*str != '\0')
    {
        if ('0' > *str || *str > '9')
            break;
        else
            result = result * 10 + (*str++ - '0');
    }

    if (*str != '\0')//no-normal end
        return -2;

    return (bMinus == 0x01) ? -result : result;
}

const CeStringOp ceStringOp = {ceStrlen, ceStrcmp,ceStrcpy, ceItoa, ceAtoi};


/**
  * @brief   Fifo初始化
  * @param   ceFifo:Cefifo对象属性指针
  */
void ceFifo_initial(CeFifo* ceFifo)
{
    ceFifo->readIndex = 0;
    ceFifo->writeIndex = 1;
    ceFifo->isReadLock = 0x00;
    ceFifo->isWriteLock = 0x00;
}

/**
  * @brief   检测Fifo是否为空，即是否有数据可读
  * @param   ceFifo:Cefifo对象属性指针
  * @return  Fifo是否为空，返回0：Fifo不为空；返回1：Fifo为空
  */
uint8 ceFifo_isEmpty(CeFifo* ceFifo)
{
    if (ceFifo->writeIndex  - ceFifo->readIndex== 1 || (ceFifo->readIndex == ceFifo->buffSize-1 && ceFifo->writeIndex == 0))
    {
        return 0x01;
    } else
    {
        return 0x00;
    }
}

/**
  * @brief   获得可读的数据长度
  * @param   ceFifo:Cefifo对象属性指针
  * @return  可读的数据长度
  */
uint16 ceFifo_getCanReadSize(CeFifo* ceFifo)
{
    uint16 temp = 0;
    if (ceFifo->readIndex < ceFifo->writeIndex)
    {
        temp = ceFifo->writeIndex - ceFifo->readIndex - 1;
    } else
    {
        temp = (ceFifo->writeIndex - 0) + (ceFifo->buffSize - ceFifo->readIndex - 1);
    }
    return temp;
}

/**
  * @brief   获得可写入的数据长度
  * @param   ceFifo:Cefifo对象属性指针
  * @return  可写入的数据长度
  */
uint16 ceFifo_getCanWriteSize(CeFifo* ceFifo)
{
    uint16 temp = 0;
    if(ceFifo->readIndex > ceFifo->writeIndex)
    {
        temp = ceFifo->readIndex - ceFifo->writeIndex -1;
    }else
    {
        if(ceFifo->readIndex == 0)
        {
            temp = ceFifo->buffSize- ceFifo->writeIndex-1;
        }else
        {
            temp = (ceFifo->buffSize- ceFifo->writeIndex-1) + (ceFifo->readIndex-0);
        }
    }
    return temp;
}

/**
  * @brief   向Fifo中写入数据
  * @param   ceFifo:Cefifo对象属性指针
  * @param   dataInBuf:待写入的数据缓存
  * @param   dataInCount:待写入的数据长度
  * @return  实际写入的数据长度，即可能Fifo已满
  */
uint16 ceFifo_write(CeFifo* ceFifo, uint8* dataInBuf, uint16 dataInCount)
{
    uint16 temp = 0;
    while (ceFifo->isWriteLock == 0x01)
        ceSystemOp.delayMs(0);
    ceFifo->isWriteLock = 0x01;
    while(1)
    {
        if(ceFifo->writeIndex < ceFifo->readIndex)
        {
            if(ceFifo->writeIndex == ceFifo->readIndex-1)
            {
                break;
            }
        }else
        {
            if((ceFifo->writeIndex == ceFifo->buffSize-1) && (ceFifo->readIndex == 0))
            {
                break;
            }
        }
        ceFifo->buff[ceFifo->writeIndex] = dataInBuf[temp];
        temp++;
        ceFifo->writeIndex++;
        if(ceFifo->writeIndex == ceFifo->buffSize)
        {
            ceFifo->writeIndex = 0;
        }
        if(temp >= dataInCount)
        {
            break;
        }
    }
    ceFifo->isWriteLock = 0x00;
    return temp;

}

/**
  * @brief   从Fifo中读取数据
  * @param   ceFifo:Cefifo对象属性指针
  * @param   dataOutBuf:读取数据所存放的缓存
  * @param   dataOutCount:需要读取的数据长度
  * @return  实际读取的数据长度
  */
uint16 ceFifo_read(CeFifo* ceFifo, uint8* dataOutBuf, uint16 dataOutCount)
{
    uint16 temp = 0;
    while (ceFifo->isReadLock == 0x01)
        ceSystemOp.delayMs(0);
    ceFifo->isReadLock = 0x01;

    while(ceFifo->readIndex != ceFifo->writeIndex-1 && !(ceFifo->readIndex== ceFifo->buffSize-1 && ceFifo->writeIndex == 0))
    {
        if(ceFifo->readIndex < ceFifo->writeIndex)
        {
            if(ceFifo->readIndex == ceFifo->writeIndex-1)
            {
                break;
            }
        }else
        {
            if((ceFifo->readIndex == ceFifo->buffSize-1) && (ceFifo->writeIndex == 0))
            {
                break;
            }
        }
        ceFifo->readIndex++;
        if(ceFifo->readIndex == ceFifo->buffSize)
        {
            ceFifo->readIndex = 0;
        }
        dataOutBuf[temp] = ceFifo->buff[ceFifo->readIndex];
        temp++;
        if(temp >= dataOutCount)
        {
            break;
        }
    }
    ceFifo->isReadLock = 0x00;
    return temp;
}

/**
  * @brief   清空Fifo中的数据
  * @param   ceFifo:Cefifo对象属性指针
  */
void ceFifo_clear(CeFifo* ceFifo)
{
    while (ceFifo->isReadLock == 0x01)
        ceSystemOp.delayMs(0);
    ceFifo->isReadLock = 0x01;

    while (ceFifo->isWriteLock == 0x01)
        ceSystemOp.delayMs(0);
    ceFifo->isWriteLock = 0x01;

    ceFifo->readIndex = 0;
    ceFifo->writeIndex = 1;

    ceFifo->isWriteLock = 0x00;
    ceFifo->isReadLock = 0x00;
}

/**
  * @brief   获得Fifo是否正在使用的状态
  * @param   ceFifo:Cefifo对象属性指针
  * @return  返加0：Fifo未被使用；1：Fifo正在进行读或写操作
  */
uint8   ceFifo_getReadLockStatus(CeFifo* ceFifo)
{
    return ceFifo->isWriteLock;
}

/**
  * @brief   获得Fifo是否处在写保护状态
  * @param   ceFifo:Cefifo对象属性指针
  * @return  返加0：Fifo未被使用；1：Fifo正在进行读或写操作
  */
uint8   ceFifo_getWriteLockStatus(CeFifo* ceFifo)
{
    return ceFifo->isWriteLock;
}

const CeFifoOp ceFifoOp = {ceFifo_initial, ceFifo_isEmpty, ceFifo_getCanReadSize, ceFifo_getCanWriteSize, ceFifo_write, ceFifo_read, ceFifo_clear, ceFifo_getWriteLockStatus};


/**
  * @brief   Fifo初始化
  * @param   ceDoubleFifo:CeDoubleFifo对象属性指针
  */
void ceDoubleFifo_initial(CeDoubleFifo* ceDoubleFifo)
{
    ceFifoOp.initial(&(ceDoubleFifo->ceFifoOne));
    ceFifoOp.initial(&(ceDoubleFifo->ceFifoTwo));
}

/**
  * @brief   更新缓存，即将一级缓存中为数据复制到二级缓存当中
  * @param   ceDoubleFifo:CeDoubleFifo对象属性指针
  */
void ceDoubleFifo_updata(CeDoubleFifo* ceDoubleFifo)//检查一级fifo中是否有数据需要复制到二级中。
{
    uint16 count = ceFifoOp.getCanReadSize(&(ceDoubleFifo->ceFifoOne));
    if (count > 0)
    {
        uint16 i;
        uint8 temp;
        for (i = 0; i < count; i++)
        {
            if (ceFifoOp.getCanWriteSize(&ceDoubleFifo->ceFifoTwo) >= 1 && ceFifoOp.getWriteLockStatus(&(ceDoubleFifo->ceFifoTwo)) == 0x00)
            {
                ceFifoOp.read(&(ceDoubleFifo->ceFifoOne), &temp, 1);
                ceFifoOp.write(&(ceDoubleFifo->ceFifoTwo), &temp, 1);
            } else
            {
                break;
            }
        }
    }
}

/**
  * @brief   检测Fifo是否为空，即是否有数据可读
  * @param   ceDoubleFifo:CeDoubleFifo对象属性指针
  * @return  Fifo是否为空，返回0：Fifo不为空；返回1：Fifo为空
  */
uint8 ceDoubleFifo_isEmpty(CeDoubleFifo* ceDoubleFifo)
{
    ceDoubleFifo_updata(ceDoubleFifo);
    return ceFifoOp.isEmpty(&(ceDoubleFifo->ceFifoTwo));
}

/**
  * @brief   获得可读的数据长度
  * @param   ceDoubleFifo:CeDoubleFifo对象属性指针
  * @return  可读的数据长度
  */
uint16 ceDoubleFifo_getCanReadSize(CeDoubleFifo* ceDoubleFifo)
{
    ceDoubleFifo_updata(ceDoubleFifo);
    return ceFifoOp.getCanReadSize(&(ceDoubleFifo->ceFifoTwo));
}

/**
  * @brief   获得可写入的数据长度
  * @param   ceDoubleFifo:CeDoubleFifo对象属性指针
  * @return  可写入的数据长度
  */
uint16 ceDoubleFifo_getCanWriteSize(CeDoubleFifo* ceDoubleFifo)
{
    ceDoubleFifo_updata(ceDoubleFifo);
    return ceFifoOp.getCanWriteSize(&(ceDoubleFifo->ceFifoTwo));
}

/**
  * @brief   向Fifo中写入数据
  * @param   ceDoubleFifo:CeDoubleFifo对象属性指针
  * @param   dataInBuf:待写入的数据缓存
  * @param   dataInCount:待写入的数据长度
  * @return  实际写入的数据长度，即可能Fifo已满
  */
uint16 ceDoubleFifo_write(CeDoubleFifo* ceDoubleFifo, uint8* dataInBuf, uint16 dataInCount)//中断调用，能够保证状态不被变换
{
    if(ceDoubleFifo->ceFifoTwo.isWriteLock == 0x01)
    {
        return ceFifoOp.write(&(ceDoubleFifo->ceFifoOne),dataInBuf, dataInCount);
    }else
    {
        ceDoubleFifo_updata(ceDoubleFifo);
        return ceFifoOp.write(&(ceDoubleFifo->ceFifoTwo),dataInBuf, dataInCount);
    }
}

/**
  * @brief   从Fifo中读取数据
  * @param   ceDoubleFifo:CeDoubleFifo对象属性指针
  * @param   dataOutBuf:读取数据所存放的缓存
  *  @param  dataOutCount:需要读取的数据长度
  * @return  实际读取的数据长度
  */
uint16 ceDoubleFifo_read(CeDoubleFifo* ceDoubleFifo, uint8* dataOutBuf, uint16 dataOutCount)//程度调用，操作过程可能会被中断打断
{
    ceDoubleFifo_updata(ceDoubleFifo);
    return ceFifoOp.read(&(ceDoubleFifo->ceFifoTwo),dataOutBuf, dataOutCount);
}

/**
  * @brief   清空Fifo中的数据
  * @param   ceDoubleFifo:CeDoubleFifo对象属性指针
  */
void ceDoubleFifo_clear(CeDoubleFifo* ceDoubleFifo)
{
    ceFifoOp.clear(&(ceDoubleFifo->ceFifoOne));
    ceFifoOp.clear(&(ceDoubleFifo->ceFifoOne));
}

const CeDoubleFifoOp ceDoubleFifoOp = {ceDoubleFifo_initial, ceDoubleFifo_isEmpty, ceDoubleFifo_getCanReadSize, ceDoubleFifo_getCanWriteSize,
                                            ceDoubleFifo_write, ceDoubleFifo_read, ceDoubleFifo_clear};

#ifdef __cplusplus
 }
#endif //__cplusplus
