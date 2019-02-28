/**
  ******************************************************************************
  * @file    CeBmp180.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeBmp180模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_BMP180_H__
#define __CE_BMP180_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_BMP180_VERSION__ 1                                             /*!< 此驱动文件的版本号*/
#define __CE_BMP180_NEED_CREELINKS_VERSION__ 1                               /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_BMP180_NEED_CREELINKS_VERSION__)       /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeBmp180.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else



typedef struct
{
    fp32     temperature;        /*!< 温度值*/
    int32    pressure;           /*!< 气压值*/
    fp32     altitude;           /*!< 海拔高度*/
}CeBmp180Environment;

/*
 *CeBmp180属性对像
 */
typedef struct
{
    CeI2cMaster ceI2cMaster;
    int16       AC1;
    int16       AC2;
    int16       AC3;
    uint16      AC4;
    uint16      AC5;
    uint16      AC6;
    int16       B1;
    int16       B2;
    int16       MB;
    int16       MC;
    int16       MD;
    int32       UT;
    int32       UP;
    CeBmp180Environment environment;
    uint32 lastSystemTimeMs;    /*!< 由于Bmp180进行一次气压读取需要至少4.5ms时间，此值用于计算从开始转换到*/
    uint8       asyncStep;
    fp32        lastAltiude;

}CeBmp180;
/*
 *CeBmp180操作对像
 */
typedef struct
{
    CE_STATUS               (*initial)(CeBmp180* ceBmp180, CE_RESOURCE ceI2cMaster);/*!< 
                                                                                 @brief CeBmp180模块初始化
                                                                                 @param ceBmp180:CeBmp180属性对象指针
                                                                                 @param ceI2cMaster:CeBmp180模块使用的资源号*/
                                                                             
    CeBmp180Environment*    (*getEnvironment)(CeBmp180* ceBmp180);          /*!< @brief 获取已经过校正的温度、气压、海拔高度
                                                                                 @param ceBmp180:CeBmp180属性对象指针*/

    CeBmp180Environment*    (*getEnvironmentAsync)(CeBmp180* ceBmp180);     /*!< @brief 获取已经过校正的温度、气压、海拔高度。非操作系统环境下异步调用，可保证无10ms等待延时，计算完成后返回正确结果，否则返回CE_NULL
                                                                                 @param ceBmp180:CeBmp180属性对象指针*/
}CeBmp180Op;
/*
 *CeBmp180操作对象实例
 */
extern const CeBmp180Op ceBmp180Op;

#endif // (__CE_CREELINKS_VERSION__ < __CE_BMP180_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_BMP180_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function xxxxxzzzz
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
