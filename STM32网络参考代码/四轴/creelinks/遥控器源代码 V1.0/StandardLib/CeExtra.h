/**
  ******************************************************************************
  * @file    CeExtra.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台CeExtra库头文件
  ******************************************************************************
  * @attention
  *
  *1)因Creelinks需要保证最大的兼容性，大部分C语言库函数均由Creelinks自己实现
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_EXTRA_H__
#define __CE_EXTRA_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @brief  结构体，Creelinks与数学计算相关的函数
  */
typedef struct
{
    fp32 (*abs)(fp32 val);                                    /*!< @brief 对参数取绝对值
                                                                     @param val:需要计算的值
                                                                     @return 计算结果*/
}CeMathOp;
extern const CeMathOp ceMathOp;                             /*!< 所有与数学计算相关的操作*/

/**
  * @brief  结构体，Creelinks与字符串相关的操作函数
  */
typedef struct
{
    uint32  (*strlen)(const char* str);                         /*!< @brief 计算字符串长度，不计算'\0'
                                                                     @param str:需要计算的值
                                                                     @return 字符串长度*/

    int8    (*strcmp)(const char* str1, const char* str2);      /*!< @brief 比较两个字符串是否相等
                                                                     @param str1:字符串1
                                                                     @param str2:字符串2
                                                                     @return 0相等， 1不相等*/

    char *  (*strcpy)(char *dest, char *src);                   /*!< @brief 字符串拷贝
                                                                     @param dest:目的地址
                                                                     @param src:源地址字符串
                                                                     @return 目的地址的指针*/

    char*   (*itoa)(int32 val, char* str, uint8 radix);         /*!< @brief 将整数转化为字符串
                                                                     @param val:整数
                                                                     @param str:字符串缓存
                                                                     @param radix:需要转换的进制，8/10/16
                                                                     @return 转换完成后的字符串*/

    int32   (*atoi)(const char* str);                           /*!< @brief 将字符串转换为整数
                                                                     @param str:要转换的字符串
                                                                     @return 转换后的整数*/

}CeStringOp;
extern const CeStringOp ceStringOp;                         /*!< 所有与字符串相关的操作*/

/**
  * @brief  结构体，Fifo对象属性集合
  */
typedef struct
{
    uint8*  buff;                                               /*!< Fifo缓存*/
    uint16  buffSize;                                           /*!< Fifo缓存容量*/
    uint16  readIndex;                                          /*!< Fifo中读索引*/
    uint16  writeIndex;                                         /*!< Fifo中写索引*/
    uint8   isReadLock;                                         /*!< Fifo读保护，防止多个线程同时读*/
    uint8   isWriteLock;                                        /*!< Fifo写保护，防止多个纯种同时写*/
}CeFifo;

/**
  * @brief  结构体，Creelinks与Fifo相关的操作函数
  */
typedef struct
{
    void    (*initial)(CeFifo* ceFifo);                         /*!< @brief Fifo初始化
                                                                     @param ceFifo:Cefifo对象属性指针*/

    uint8   (*isEmpty)(CeFifo* ceFifo);                         /*!< @brief 检测Fifo是否为空，即是否有数据可读
                                                                     @param ceFifo:Cefifo对象属性指针
                                                                     @return Fifo是否为空，返回0：Fifo不为空；返回1：Fifo为空*/

    uint16  (*getCanReadSize)(CeFifo* ceFifo);                  /*!< @brief 获得可读的数据长度
                                                                     @param ceFifo:Cefifo对象属性指针
                                                                     @return 可读的数据长度*/

    uint16  (*getCanWriteSize)(CeFifo* ceFifo);                 /*!< @brief 获得可写入的数据长度
                                                                     @param ceFifo:Cefifo对象属性指针
                                                                     @return 可写入的数据长度*/

    uint16  (*write)(CeFifo* ceFifo, uint8* dataInBuf, uint16 dataInCount);/*!<
                                                                     @brief 向Fifo中写入数据
                                                                     @param ceFifo:Cefifo对象属性指针
                                                                     @param dataInBuf:待写入的数据缓存
                                                                     @param dataInCount:待写入的数据长度
                                                                     @return 实际写入的数据长度，即可能Fifo已满*/

    uint16  (*read)(CeFifo* ceFifo, uint8* dataOutBuf, uint16 dataOutCount);/*!<
                                                                     @brief 从Fifo中读取数据
                                                                     @param ceFifo:Cefifo对象属性指针
                                                                     @param dataOutBuf:读取数据所存放的缓存
                                                                     @param dataOutCount:需要读取的数据长度
                                                                     @return 实际读取的数据长度*/

    void    (*clear)(CeFifo* ceFifo);                           /*!< @brief 清空Fifo中的数据
                                                                     @param ceFifo:Cefifo对象属性指针*/

    uint8   (*getReadLockStatus)(CeFifo* ceFifo);               /*!< @brief 获得Fifo是否处在读保护状态
                                                                     @param ceFifo:Cefifo对象属性指针
                                                                     @return 返加0：Fifo未被使用；1：Fifo正在进行读或写操作*/
    uint8   (*getWriteLockStatus)(CeFifo* ceFifo);              /*!< @brief 获得Fifo是否处在写保护状态
                                                                     @param ceFifo:Cefifo对象属性指针
                                                                     @return 返加0：Fifo未被使用；1：Fifo正在进行读或写操作*/

}CeFifoOp;
extern const CeFifoOp ceFifoOp;

/**
  * @brief  结构体，双缓存的Fifo对象属性集合
  */
typedef struct
{
    CeFifo ceFifoOne;                                           /*!< Fifo一级缓存*/
    CeFifo ceFifoTwo;                                           /*!< Fifo二级缓存*/
}CeDoubleFifo;

/**
  * @brief  结构体，Creelinks与双缓存Fifo相关的操作函数
  */
typedef struct
{
    void    (*initial)(CeDoubleFifo* ceDoubleFifo);             /*!< @brief Fifo初始化
                                                                     @param ceDoubleFifo:CeDoubleFifo对象属性指针*/

    uint8   (*isEmpty)(CeDoubleFifo* ceDoubleFifo);             /*!< @brief 检测Fifo是否为空，即是否有数据可读
                                                                     @param ceDoubleFifo:CeDoubleFifo对象属性指针
                                                                     @return Fifo是否为空*/

    uint16  (*getCanReadSize)(CeDoubleFifo* ceDoubleFifo);      /*!< @brief 获得可读的数据长度
                                                                     @param ceDoubleFifo:CeDoubleFifo对象属性指针
                                                                     @return 可读的数据长度*/

    uint16  (*getCanWriteSize)(CeDoubleFifo* ceDoubleFifo);     /*!< @brief 获得可写入的数据长度
                                                                     @param ceDoubleFifo:CeDoubleFifo对象属性指针
                                                                     @return 可写入的数据长度*/

    uint16  (*write)(CeDoubleFifo* ceDoubleFifo, uint8* dataInBuf, uint16 dataInCount);/*!<
                                                                     @brief 向Fifo中写入数据
                                                                     @param ceDoubleFifo:CeDoubleFifo对象属性指针
                                                                     @param dataInBuf:待写入的数据缓存
                                                                     @param dataInCount:待写入的数据长度
                                                                     @return 实际写入的数据长度，即可能Fifo已满*/

    uint16  (*read)(CeDoubleFifo* ceDoubleFifo, uint8* dataOutBuf, uint16 dataOutCount);/*!<
                                                                     @brief 从Fifo中读取数据
                                                                     @param ceDoubleFifo:CeDoubleFifo对象属性指针
                                                                     @param dataOutBuf:读取数据所存放的缓存
                                                                     @param dataOutCount:需要读取的数据长度
                                                                     @return 实际读取的数据长度*/

    void    (*clear)(CeDoubleFifo* ceDoubleFifo);               /*!< @brief 清空Fifo中的数据
                                                                     @param ceDoubleFifo:CeDoubleFifo对象属性指针*/

}CeDoubleFifoOp;
extern const CeDoubleFifoOp ceDoubleFifoOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_EXTRA_H__
