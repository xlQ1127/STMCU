/**
  ******************************************************************************
  * @file    CeJoystick.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeJoystick模块的驱动库文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeJoystick.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  CeJoystick模块初始化
  * @param  ceJoystick:CeJoystick属性对象
  * @param  ceXX:CeJoystick模块使用的资源号
  * @return 系统状态码
  */
CE_STATUS ceJoystick_initial(CeJoystick* ceJoystick, CE_RESOURCE ceAd1, CE_RESOURCE ceAd2, CE_RESOURCE ceGpio)
{
    int32 sum = 0;
    int i;
    ceJoystick->ceJoystickAxis.x = 0;
    ceJoystick->ceJoystickAxis.y = 0;
    ceJoystick->ceAdY.ceResource = ceAd1;
    ceAdOp.initial(&(ceJoystick->ceAdY));

    ceJoystick->ceAdX.ceResource = ceAd2;
    ceAdOp.initial(&(ceJoystick->ceAdX));

    ceJoystick->ceGpio.ceResource = ceGpio;
    ceJoystick->ceGpio.ceGpioMode = CE_GPIO_MODE_IN_FLOATING;
    ceGpioOp.initial(&(ceJoystick->ceGpio));

    for(i=0;i<32;i++)
    {
        sum += ceAdOp.getConvertValue(&(ceJoystick->ceAdX));
        ceSystemOp.delayMs(2);
    }
    ceJoystick->ceCalibrationZero.x = sum /32;
    sum = 0;
    for(i=0;i<32;i++)
    {
        sum += ceAdOp.getConvertValue(&(ceJoystick->ceAdY));
        ceSystemOp.delayMs(2);
    }
    ceJoystick->ceCalibrationZero.y = sum /32;

    for(i = 0;i<CE_JOY_STICK_SLIDER_SIZE;i++)
    {
        ceJoystick->sliderX[i] = 0;
        ceJoystick->sliderY[i] = 0;
    }

    return CE_STATUS_SUCCESS;
}



/**
  * @brief  获取坐标
  * @param  ceJoystick:CeJoystick属性对象
  * @return 系统状态码
  */
CeJoystickAxis* ceJoystick_getAxis(CeJoystick* ceJoystick)
{
    int32 tempX,tempY,i;
    int32 x=0,y=0;
    tempX = ceAdOp.getConvertValue(&(ceJoystick->ceAdX));
    tempY = ceAdOp.getConvertValue(&(ceJoystick->ceAdY));

    tempX = (2000 * ( ceJoystick->ceCalibrationZero.x - tempX)) / 4096;
    tempY = (2000 * (tempY - ceJoystick->ceCalibrationZero.y)) / 4096;

    for(i = 0;i<CE_JOY_STICK_SLIDER_SIZE-1;i++)
    {
        ceJoystick->sliderX[i] = ceJoystick->sliderX[i+1];
        ceJoystick->sliderY[i] = ceJoystick->sliderY[i+1];
        x += ceJoystick->sliderX[i];
        y += ceJoystick->sliderY[i];
    }
    ceJoystick->sliderX[CE_JOY_STICK_SLIDER_SIZE-1] = tempX;
    ceJoystick->sliderY[CE_JOY_STICK_SLIDER_SIZE-1] = tempY;
    x += tempX;
    y += tempY;
    tempX = x/CE_JOY_STICK_SLIDER_SIZE;
    tempY = y/CE_JOY_STICK_SLIDER_SIZE;

    if(tempX >1000)
        tempX = 1000;
    else if(tempX < -1000)
        tempX = -1000;

    if(tempY >1000)
        tempY = 1000;
    else if(tempY < -1000)
        tempY = -1000;    

    ceJoystick->ceJoystickAxis.x = tempX;
    ceJoystick->ceJoystickAxis.y = tempY;
    // ceDebugOp.printf("X = %d Y = %d \n\n", tempX, tempY);    
    return &(ceJoystick->ceJoystickAxis);
}

/**
  * @brief  校准零点
  * @param  ceJoystick:CeJoystick属性对象
  * @return 系统状态码
  */
void ceJoystick_calibrationZero(CeJoystick* ceJoystick)
{
    uint32 tempX = 0, tempY = 0;//这里最好滤波一下。
    int i = 0;
    for(i = 0; i < 10; i++)
    {
        tempX += ceAdOp.getConvertValue(&(ceJoystick->ceAdX));
        tempY += ceAdOp.getConvertValue(&(ceJoystick->ceAdY));
        ceSystemOp.delayMs(1);
    }

    ceJoystick->ceCalibrationZero.x = (2000 * (int16)(tempX / 10)) / 4096 - 1000;
    ceJoystick->ceCalibrationZero.y = (2000 * (int16)(tempY / 10)) / 4096 - 1000;
}

/**
  * @brief  获取按钮的状态
  * @param  ceJoystick:CeJoystick属性对象
  * @return 系统状态码
  */
uint8 ceJoystick_getBtnStatus(CeJoystick* ceJoystick)
{
      if( ceGpioOp.getBit(&ceJoystick->ceGpio) == 0x00)
            return 0x01;
        else 
            return 0x00;
}

/**
  * @brief  CeJoystick模块操作对象定义
  */
const CeJoystickOp ceJoystickOp = {ceJoystick_initial, ceJoystick_calibrationZero, ceJoystick_getAxis, ceJoystick_getBtnStatus};

#ifdef __cplusplus
 }
#endif //__cplusplus
