/**
  ******************************************************************************
  * @file    CeFilter.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   脉冲干扰、滑动平均、一阶、二阶、四元数、卡尔曼滤波（姿态解算）器。
  ******************************************************************************
  * @attention
  *
  *1)所有滤波器参数，均在initial中进行初始化
  *2)默认使用卡尔曼滤波方式
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_FILTER_H__
#define __CE_FILTER_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#include "CePackage.h"

/*
  *结构体，姿态角结构体
  */
typedef struct
{
    fp32 roll;          /*!< 翻滚角*/
    fp32 picth;         /*!< 俯仰角*/
    fp32 yaw;           /*!< 偏航角*/
    fp32 altitude;      /*!< 海拔高度*/
    fp32 accelerator;   /*!< 上升与下降的油门，范围CE_IMU_MIX_DRIVER_POWER~CE_IMU_MAX_DRIVER_POWER*/
}CeAngles;
/**
  * @brief  结构体，加速度
  */
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
}CeAcc;
/**
  * @brief  结构体，陀螺仪角速度
  */
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
}CeGyr;


#define CE_SLIDE_FILTER_SIZE    20//滑动平均滤波数组长度
/**
  * @brief  结构体，滑动平均滤波属性对象
  */
typedef struct
{
    fp32 array[CE_SLIDE_FILTER_SIZE];
}CeFilterSlider;
/**
  * @brief  结构体，滑动平均滤波操作对象
  */
typedef struct
{
    void    (*initial)(CeFilterSlider* ceSlidefilter);              /*!< @brief  滑动平均滤波初始化
                                                                         @param  ceSlidefilter:滑动平均滤波属性对象*/

    fp32    (*filter)(CeFilterSlider* ceFilterSlider, fp32 newVal); /*!< @brief  输入新值，返回滤波后的值
                                                                         @param  ceSlidefilter:滑动平均滤波属性对象
                                                                         @param  newVal:未进行滤波的新值
                                                                         @return 滤波后的值*/
}CeFilterSliderOp;
/**
  * @brief  结构体，滑动平均滤波操作对象定义
  */
extern const CeFilterSliderOp ceFilterSliderOp;



/*
 * @brief  毛刺滤波器属性
 */
typedef struct
{
    fp32 lastVal;                   /*!< 上一次的值*/
    fp32 maxAbs;                    /*!< 两次数据允许的最大差距值*/
    fp32 coes[10];                  /*!< 权值数组*/
    uint16 index;                 
    uint16 isUp;                   
}CeFilterBase;

typedef struct
{
    void(*initial)(CeFilterBase* ceFilterBase,fp32 maxAbs);             /*!< @brief  毛刺滤波器初始化
                                                                             @param  ceFilterBase:毛刺滤波器属性对象
                                                                             @param  maxAbs:两次数据之差绝对值的最大值*/

    fp32(*filter)(CeFilterBase* ceFilterBase, fp32 newVal);             /*!< @brief  对数据进行毛刺滤波，根据新值计算旧值
                                                                             @param  ceFilterBase:毛刺滤波器属性对象
                                                                             @param  newVal:新采集到的待滤波的值
                                                                             @return 滤波后的值*/  
}CeFilterBaseOp;

extern CeFilterBaseOp ceFilterBaseOp;

/**
  * @brief  结构体，一阶滤波操作对象
  */
typedef struct
{
    fp32 K1;
    fp32 angle;
}CeFilterYijie;

/**
  * @brief  结构体，一阶滤波操作对象
  */
typedef struct
{
    void    (*initial)(CeFilterYijie* ceFilterYijie);               /*!< @brief  一阶滤波初始化
                                                                         @param  ceFilterYijie:一阶滤波属性对象*/

    fp32    (*filter)(CeFilterYijie* ceFilterYijie, fp32 angle_m, fp32 gyro_m,fp32 dt);/*!< 
                                                                         @brief  输入新值，返回滤波后的角度值，详细可参考CREELINKS相关文档
                                                                         @param  ceFilterYijie:一阶滤波属性对象
                                                                         @param  angle_m:未滤波的由加速度直接获取的姿态角度
                                                                         @param  gyro_m:未滤波的角速度
                                                                         @return 滤波后的角度值*/
}CeFilterYijieOp;
/**
  * @brief  结构体，一阶滤波操作对象定义
  */
extern const CeFilterYijieOp ceFilterYijieOp;




/**
  * @brief  结构体，二阶滤波属性对象
  */
typedef struct
{
    fp32 K2;
    fp32 angle;
    fp32 x1;
    fp32 x2;
    fp32 y1;
}CeFilterErjie;
/**
  * @brief  结构体，二阶滤波操作对象
  */
typedef struct
{
    void    (*initial)(CeFilterErjie* ceFilterErjie);               /*!< @brief  二阶滤波初始化
                                                                         @param  ceFilterErjie:二阶滤波滤波属性对象*/


    fp32    (*filter)(CeFilterErjie* ceFilterErjie, fp32 angle_m, fp32 gyro_m,fp32 dt);/*!< 
                                                                         @brief  输入新值，返回滤波后的角度值，详细可参考CREELINKS相关文档
                                                                         @param  ceFilterErjie:二阶滤波属性对象
                                                                         @param  angle_m:未滤波的由加速度直接获取的姿态角度
                                                                         @param  gyro_m:未滤波的角速度
                                                                         @return 滤波后的角度值*/
}CeFilterErjieOp;
/**
  * @brief  结构体，二阶滤波操作对象定义
  */
extern const CeFilterErjieOp ceFilterErjieOp;





/**
  * @brief  结构体，四元数+互补滤波属性对象，有关四元数与互补滤波的关系，请参考CREELINKS相关文档
  */
typedef struct
{
    fp32 q0;
    fp32 q1;
    fp32 q2;
    fp32 q3;
    fp32 kp;
    fp32 ki;
}CeFilterIMU;
/**
  * @brief  结构体，四元数+互补滤波操作对象定义
  */
typedef struct
{
    void    (*initial)(CeFilterIMU* ceFilterIMU);                   /*!< @brief  四元数+互补滤波初始化
                                                                         @param  ceFilterIMU:四元数+互补滤波属性对象*/

    void    (*filter)(CeFilterIMU* ceFilterIMU, CeAcc* nowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngle,fp32 halfT);/*!< 
                                                                         @brief  输入新值，返回滤波后的角度值，详细可参考CREELINKS相关文档
                                                                         @param  ceFilterIMU:四元数+互补滤波属性对象
                                                                         @param  nowAcc:当前无人机加速度数据
                                                                         @param  ceNowGyr:当前无人机角速度数据
                                                                         @param  ceNowAngle:当前无人机姿态数据，四元数姿态角计算完毕后会修改此指针中的内容*/
}CeFilterIMUOp;
/**
  * @brief  结构体，四元数+互补操作对象定义
  */
extern const CeFilterIMUOp ceFilterIMUOp;




/**
  * @brief  结构体，卡尔曼滤波属性对象，详细请参考CREELINKS相关文档
  */
typedef struct
{
    fp32 angle;
    fp32 angle_dot; 
    fp32 P[2][2];
    fp32 Pdot[4];
    fp32 Q_angle;
    fp32 Q_gyro;
    fp32 R_angle;
    fp32 C_0;
    fp32 q_bias;
    fp32 angle_err;
    fp32 PCt_0;
    fp32 PCt_1;
    fp32 E; 
    fp32 K_0;
    fp32 K_1; 
    fp32 t_0; 
    fp32 t_1;
}CeFilterKalman;
/**
  * @brief  结构体，卡尔曼滤波操作对象定义
  */
typedef struct
{
    void    (*initial)(CeFilterKalman* ceFilterKalman);             /*!< @brief  卡尔曼滤波初始化
                                                                         @param  ceFilterErjie:卡尔曼滤波属性对象*/


    void    (*filter)(CeFilterKalman* ceFilterKalman, fp32* angle_m, fp32* gyro_m,fp32 dt);/*!< 
                                                                         @brief  输入新值，返回滤波后的角度值，详细可参考CREELINKS相关文档
                                                                         @param  ceFilterErjie:卡尔曼滤波属性对象
                                                                         @param  angle_m:未滤波的由加速度直接获取的姿态角度，计算完成后会将计算结果写入此指针地址
                                                                         @param  gyro_m:未滤波的角速度，计算完成后会将计算结果写入此指针地址*/
}CeFilterKalmanOp;
/**
  * @brief  结构体，卡尔曼操作对象定义
  */
extern const CeFilterKalmanOp ceFilterKalmanOp;




/*
 *CeFilter属性对像
 */
typedef struct
{
    uint16          ceFilterType;        /*!< 当前无人机姿态解算及滤波方式*/

    fp32            dt;

    CeFilterSlider  ceFilterSliderAccX; /*!< X轴加速度的滑动平均滤波器*/
    CeFilterSlider  ceFilterSliderAccY; /*!< Y轴加速度的滑动平均滤波器*/
    CeFilterSlider  ceFilterSliderAccZ; /*!< Z轴加速度的滑动平均滤波器*/

    CeFilterSlider  ceFilterSliderGyrX; /*!< X轴角速度的滑动平均滤波器*/
    CeFilterSlider  ceFilterSliderGyrY; /*!< Y轴角速度的滑动平均滤波器*/
    CeFilterSlider  ceFilterSliderGyrZ; /*!< Z轴角速度的滑动平均滤波器*/

    CeFilterBase    ceFilterBaseAccX;
    CeFilterBase    ceFilterBaseAccY;
    CeFilterBase    ceFilterBaseAccZ;
    CeFilterBase    ceFilterBaseGyrX;
    CeFilterBase    ceFilterBaseGyrY;
    CeFilterBase    ceFilterBaseGyrZ;

    CeFilterYijie   ceFilterYijiePitch; /*!< 一阶Pitch角滤波器*/
    CeFilterYijie   ceFilterYijieRoll;  /*!< 一阶Roll角滤波器*/

    CeFilterErjie   ceFilterErjiePitch; /*!< 二阶Pitch角滤波器*/
    CeFilterErjie   ceFilterErjieRoll;  /*!< 二阶Roll角滤波器*/

    CeFilterIMU     ceFilterIMU;        /*!< 四元数+互补滤波器*/

    CeFilterKalman  ceFilterKalmanPitch;/*!< 卡尔曼Pitch角滤波器*/
    CeFilterKalman  ceFilterKalmanRoll; /*!< 卡尔曼Roll角滤波器*/

    CePackageSend*  cePackageSend;
    CePackageRecv*  cePackageRecv;

    CeAngles     ceAnglesZero;              /*!< 用于校正姿态零点，以消除机械误差*/
}CeFilter;
/*
 *CeFilter操作对像
 */
typedef struct
{
    void (*initial)(CePackageSend* cePackageSend, CePackageRecv* cePackageRecv);/*!< 
                                                                             @brief  滤波初始化
                                                                             @param  cePackageSend:数据打包并发送使用的结构体
                                                                             @param  cePackageRecv:数据拆包并解析使用的结构体*/

    void (*filter)(CeAcc* nowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngle, fp32 dtS); /*!< 
                                                                             @brief  输入当前角速度及加速度来计算当前无人机的姿态角，详细可参考CREELINKS相关文档
                                                                             @param  nowAcc:当前无人机加速度数据
                                                                             @param  ceNowGyr:当前无人机角速度数据
                                                                             @param  ceNowAngle:当前无人机姿态数据，四元数姿态角计算完毕后会修改此指针中的内容
                                                                             @param  dtS:程序执行周期，单位S*/  
}CeFilterOp;
/*
 *CeFilter操作对象实例
 */
extern const CeFilterOp ceFilterOp;



#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_FILTER_H__
