/**
  ******************************************************************************
  * @file    CeBmp180.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeBmp180模块的驱动库文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeBmp180.h"
#include "math.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus



/*!<         BMP180地址及相关的寄存器*/
#define BMP180_ADDR       0xEE  // default I2C address

#define CE_BMP180_MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5 超低功耗
#define CE_BMP180_MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4 标准
#define CE_BMP180_MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3 高线性度
#define CE_BMP180_MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25 超高线性度

#define CE_BMP180_MODE  CE_BMP180_MODE_ULTRA_HIGHRES//超高线性度,转换时间25ms可接受

#define CE_BMP180_FILTER_A   0.055f
//    fp32 CE_BMP180_FILTER_A= 0.1f;
/**
  * @brief  初始化I2c
  * @param  ceI2cMaster:ceI2cMaster
  * @param  ceI2cMasterResource:ceI2cMasterResource
  * @return None
  */
void ceBmp180_I2Cx_Init(CeI2cMaster *ceI2cMaster, CE_RESOURCE ceI2cMasterResource)
{
    ceI2cMaster->ceResource = ceI2cMasterResource;
    ceI2cMaster->ceI2cMasterSpeed = CE_I2C_SPEED_3_4MBPS;
    ceI2cMasterOp.initial(ceI2cMaster);
}

/**
  * @brief  Write an byte to the specified device address through I2C bus.
  * @param  ceI2cMaster:ceI2cMaster
  * @param  DevAddr: The address byte of the slave device
  * @param  RegAddr: The address byte of  register of the slave device
  * @param  Data: the data would be writen to the specified device address
  * @return  None
  */
void ceBmp180_I2C_WriteOneByte(CeI2cMaster *ceI2cMaster, uint8 DevAddr, uint8 RegAddr, uint8 Data)
{
    ceI2cMasterOp.start(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, DevAddr);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, RegAddr);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, Data);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.stop(ceI2cMaster);
}

/**
  * @brief Read an byte from the specified device address through I2C bus.
  * @param  ceI2cMaster:ceI2cMaster
  * @param DevAddr: The address byte of the slave device
  * @param RegAddr: The address byte of  register of the slave device
  * @return  the byte read from I2C bus
  */

uint8 ceBmp180_I2C_ReadOneByte(CeI2cMaster *ceI2cMaster, uint8 DevAddr, uint8 RegAddr)
{
    uint8 TempVal = 0;
    ceI2cMasterOp.start(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, DevAddr);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, RegAddr);
    ceI2cMasterOp.waitAck(ceI2cMaster);
    ceI2cMasterOp.start(ceI2cMaster);
    ceI2cMasterOp.writeByte(ceI2cMaster, DevAddr | 0x01);
    ceI2cMasterOp.start(ceI2cMaster);
    TempVal = ceI2cMasterOp.readByte(ceI2cMaster, 0x00);
    ceI2cMasterOp.stop(ceI2cMaster);
    return TempVal;
}

/**
  * @brief Read couples of bytes  from the specified device address through I2C bus.
  *
  * @param DevAddr: The address byte of the slave device
  * @param RegAddr: The address byte of  register of the slave device
  * @param Num: the sizes of the specified buffer
  * @param pBuff: point to a the specified buffer that would read bytes from I2C bus
  * @retval  false: paramter error
  *                true: read a buffer succeed
**/

uint8 ceBmp180_I2C_ReadBuff(CeBmp180 *ceBmp180,uint8 DevAddr, uint8 RegAddr, uint8 Num, uint8 *pBuff)
{
    uint8 i;

    if(Num == 0 || pBuff == CE_NULL)
    {
        return 0x00;
    }
    ceI2cMasterOp.start(&(ceBmp180->ceI2cMaster));
    ceI2cMasterOp.writeByte(&(ceBmp180->ceI2cMaster),DevAddr);
    ceI2cMasterOp.waitAck(&(ceBmp180->ceI2cMaster));
    ceI2cMasterOp.writeByte(&(ceBmp180->ceI2cMaster),RegAddr);
    ceI2cMasterOp.waitAck(&(ceBmp180->ceI2cMaster));
    ceI2cMasterOp.start(&(ceBmp180->ceI2cMaster));
    ceI2cMasterOp.writeByte(&(ceBmp180->ceI2cMaster),DevAddr|0x01);
    ceI2cMasterOp.waitAck(&(ceBmp180->ceI2cMaster));

    for(i = 0; i < Num; i ++)
    {
        if((Num - 1) == i)
        {
            *(pBuff + i) = ceI2cMasterOp.readByte(&(ceBmp180->ceI2cMaster),0x00);
        }
        else
        {
            *(pBuff + i) = ceI2cMasterOp.readByte(&(ceBmp180->ceI2cMaster),0x01);
        }
    }

    ceI2cMasterOp.stop(&(ceBmp180->ceI2cMaster));

    return 0x01;
}

/**
  * @brief  Read calibration data from the EEPROM of the BMP180//从BMP180的EEPROM中读取校准数据
  * @param  ceBmp180:ceBmp180
  * @return None
  */
void ceBmp180_ReadCalibrationData(CeBmp180 *ceBmp180)
{
    uint8 RegBuff[2] = { 0 };
    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xaa, 2, RegBuff);
    ceBmp180->AC1 = ((int16)RegBuff[0] << 8 | ((int16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xac, 2, RegBuff);
    ceBmp180->AC2 = ((int16)RegBuff[0] << 8 | ((int16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xae, 2, RegBuff);
    ceBmp180->AC3 = ((int16)RegBuff[0] << 8 | ((int16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xb0, 2, RegBuff);
    ceBmp180->AC4 = ((uint16)RegBuff[0] << 8 | ((uint16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xb2, 2, RegBuff);
    ceBmp180->AC5 = ((uint16)RegBuff[0] << 8 | ((uint16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xb4, 2, RegBuff);
    ceBmp180->AC6 = ((uint16)RegBuff[0] << 8 | ((uint16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xb6, 2, RegBuff);
    ceBmp180->B1 = ((int16)RegBuff[0] << 8 | ((int16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xb8, 2, RegBuff);
    ceBmp180->B2 = ((int16)RegBuff[0] << 8 | ((int16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xba, 2, RegBuff);
    ceBmp180->MB = ((int16)RegBuff[0] << 8 | ((int16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xbc, 2, RegBuff);
    ceBmp180->MC = ((int16)RegBuff[0] << 8 | ((int16)RegBuff[1]));

    ceBmp180_I2C_ReadBuff(ceBmp180, BMP180_ADDR, 0xbe, 2, RegBuff);
    ceBmp180->MD = ((int16)RegBuff[0] << 8 | ((int16)RegBuff[1]));
}

/**
  * @brief  读取未经校正的温度值
  * @param  ceBmp180:ceBmp180属性对象
  * @return 返回未经矫正的温度值
  */
int16 ceBmp180_BMP180ReadUnsetTemperature(CeBmp180 *ceBmp180)
{
    uint8 msb, lsb;
    ceBmp180_I2C_WriteOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf4, 0x2e);  
    ceSystemOp.delayMs(5);
    msb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 );
    lsb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 + 1 );
    return  ((int16)msb<<8) + lsb;

}

/**
  * @brief  读取未经校正的气压值
  * @param  ceBmp180:ceBmp180属性对象
  * @return 返回未经矫正的气压值
  */
int32 ceBmp180_BMP180ReadUnsetPressure(CeBmp180 *ceBmp180)
{
    uint8 msb, lsb,xlsb;
    ceBmp180_I2C_WriteOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf4, 0x34 +(CE_BMP180_MODE << 6));
    ceSystemOp.delayMs(25);
    msb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 );
    lsb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 + 1 );
    xlsb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 + 2);

    return (((uint32)msb << 16 )+((uint32)lsb << 8 ) + xlsb) >> (8 - CE_BMP180_MODE);
}

/**
  * @brief  获取已经过校正的温度、气压、海拔高度
  * @param  ceBmp180:ceBmp180属性对象
  * @return 温度、气压、海拔高度
  */
CeBmp180Environment* ceBmp180_getEnvironment(CeBmp180 *ceBmp180)
{
    int32 x1, x2, B5, B6, x3, B3, p;
    uint64 b4, b7;
    int32 UnsetTemperature, UnsetGasPress;
    //温度校正
    UnsetTemperature = ceBmp180_BMP180ReadUnsetTemperature(ceBmp180);
    x1 = ((UnsetTemperature) - ceBmp180->AC6) * (ceBmp180->AC5) >> 15;
    x2 = ((int32)(ceBmp180->MC) << 11) / (x1 + ceBmp180->MD);
    B5 = x1 + x2;
    ceBmp180->environment.temperature = ((fp32)((B5 + 8) >> 4)) / 10;

    //气压校正
    UnsetGasPress = ceBmp180_BMP180ReadUnsetPressure(ceBmp180);
    B6 = B5 - 4000;
    x1 = ((int32)(ceBmp180->B2) * (B6 * B6 >> 12)) >> 11;
    x2 = ((int32)ceBmp180->AC2) * B6 >> 11;
    x3 = x1 + x2;
    B3 = ((((int32)(ceBmp180->AC1) * 4 + x3) << CE_BMP180_MODE) + 2) / 4;
    x1 = ((int32)ceBmp180->AC3) * B6 >> 13;
    x2 = ((int32)(ceBmp180->B1) * (B6 * B6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = ((int32)(ceBmp180->AC4) * (unsigned long) (x3 + 32768)) >> 15;
    b7 = ((unsigned long)(UnsetGasPress) - B3) * (50000 >> CE_BMP180_MODE);
    if( b7 < 0x80000000)
        p = (b7 * 2) / b4 ;
    else
        p = (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = ((int32)x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    ceBmp180->environment.pressure = p + ((x1 + x2 + 3791) >> 4);
    ceBmp180->environment.altitude = (44330.0 * (1.0 - pow((float)(ceBmp180->environment.pressure) / 101325.0, 1.0 / 5.255)) ) / 100;//海拔计算

    return &(ceBmp180->environment);
}

#include "CePackage.h"

extern CePackageSend cePackageSend;
extern CePackageRecv cePackageRecv;

/**
  * @brief 获取已经过校正的温度、气压、海拔高度。非操作系统环境下异步调用，可保证无10ms等待延时，计算完成后返回正确结果，否则返回CE_NULL
  * @param ceBmp180:CeBmp180属性对象指针
  * @return 温度、气压、海拔高度
  */
CeBmp180Environment*    ceBmp180_getEnvironmentAsync(CeBmp180* ceBmp180)
{
    if(ceBmp180->asyncStep == 0)
    {
        ceBmp180_I2C_WriteOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf4, 0x2e);  
            ceBmp180->lastSystemTimeMs = ceSystemOp.getSystemTickMs();
        ceBmp180->asyncStep =1;
    }else if(ceBmp180->asyncStep == 1)
    {
        if(ceSystemOp.getSystemTickMs()-ceBmp180->lastSystemTimeMs >= 5)
        {
            uint8 msb, lsb;
            msb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 );
            lsb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 + 1 );
            ceBmp180->UT = ((int16)msb<<8) + lsb;
            ceBmp180_I2C_WriteOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf4, 0x34 +(CE_BMP180_MODE << 6));
            ceBmp180->asyncStep =2;
            ceBmp180->lastSystemTimeMs = ceSystemOp.getSystemTickMs();
        }
    }else if(ceBmp180->asyncStep == 2)
    {
        if(ceSystemOp.getSystemTickMs()-ceBmp180->lastSystemTimeMs >= 25)
        {
            int32 x1, x2, B5, B6, x3, B3, p;
            uint64 b4, b7;
            int32 UnsetTemperature, UnsetGasPress;
            uint8 msb, lsb,xlsb;
            msb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 );
            lsb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 + 1 );
            xlsb = ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf6 + 2);
            ceBmp180->UP = (((uint32)msb << 16 )+((uint32)lsb << 8 ) + xlsb) >> (8 - CE_BMP180_MODE);

            ceBmp180_I2C_WriteOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xf4, 0x2e);
            ceBmp180->asyncStep =1;

            //温度校正
            UnsetTemperature = ceBmp180->UT;
            x1 = ((UnsetTemperature) - ceBmp180->AC6) * (ceBmp180->AC5) >> 15;
            x2 = ((int32)(ceBmp180->MC) << 11) / (x1 + ceBmp180->MD);
            B5 = x1 + x2;
            ceBmp180->environment.temperature = ((fp32)((B5 + 8) >> 4)) / 10;

            //气压校正
            UnsetGasPress = ceBmp180->UP;
            B6 = B5 - 4000;
            x1 = ((int32)(ceBmp180->B2) * (B6 * B6 >> 12)) >> 11;
            x2 = ((int32)ceBmp180->AC2) * B6 >> 11;
            x3 = x1 + x2;
            B3 = ((((int32)(ceBmp180->AC1) * 4 + x3) << CE_BMP180_MODE) + 2) / 4;
            x1 = ((int32)ceBmp180->AC3) * B6 >> 13;
            x2 = ((int32)(ceBmp180->B1) * (B6 * B6 >> 12)) >> 16;
            x3 = ((x1 + x2) + 2) >> 2;
            b4 = ((int32)(ceBmp180->AC4) * (unsigned long) (x3 + 32768)) >> 15;
            b7 = ((unsigned long)(UnsetGasPress) - B3) * (50000 >> CE_BMP180_MODE);
            if( b7 < 0x80000000)
                p = (b7 * 2) / b4 ;
            else
                p = (b7 / b4) * 2;
            x1 = (p >> 8) * (p >> 8);
            x1 = ((int32)x1 * 3038) >> 16;
            x2 = (-7357 * p) >> 16;

            ceBmp180->environment.pressure = p + ((x1 + x2 + 3791) >> 4);
            ceBmp180->environment.altitude = (44330.0 * (1.0 - pow((float)(ceBmp180->environment.pressure) / 101325.0, 1.0 / 5.255)));//海拔计算
                                        
            if(ceMathOp.abs((int32)(ceBmp180->environment.altitude - ceBmp180->lastAltiude)) > 100 && ceBmp180->lastAltiude != 0)
                ceBmp180->environment.altitude = ceBmp180->lastAltiude;
                        
            cePackageSend.pressure = ((int32)(ceBmp180->environment.altitude*1000));
                        
                        //cePackageSend.temperature = (int32)(ceBmp180->environment.altitude*1000);                        
                        
                        //CE_BMP180_FILTER_A = (fp32)cePackageRecv.imuKp / 1000;
            ceBmp180->environment.altitude = (ceBmp180->environment.altitude) * CE_BMP180_FILTER_A +  ceBmp180->lastAltiude* (1- CE_BMP180_FILTER_A);//一阶滞后滤波算法
           
                        ceBmp180->lastAltiude = ceBmp180->environment.altitude;

            ceBmp180->lastSystemTimeMs = ceSystemOp.getSystemTickMs();
        }
    }
    return &(ceBmp180->environment);
}

/**
  * @brief  CeBmp180模块初始化
  * @param  ceBmp180:CeBmp180属性对象
  * @param  ceXX:CeBmp180模块使用的资源号
  * @return 系统状态码
  */
CE_STATUS ceBmp180_initial(CeBmp180 *ceBmp180, CE_RESOURCE ceI2cMaster)
{
    ceBmp180->environment.altitude = 0;
    ceBmp180->environment.pressure = 0;
    ceBmp180->environment.temperature = 0;
    ceBmp180->environment.pressure = 0;
    ceBmp180->asyncStep = 0;
    ceBmp180->lastAltiude = 0;

    ceBmp180_I2Cx_Init(&(ceBmp180->ceI2cMaster), ceI2cMaster);

    if(ceBmp180_I2C_ReadOneByte(&(ceBmp180->ceI2cMaster), BMP180_ADDR, 0xd0 ) == 0x55) //存在
    {
        ceBmp180_ReadCalibrationData(ceBmp180);
        ceDebugOp.printf("BMP180 initial success!\n");
        return CE_STATUS_SUCCESS;
    }
    else//不存在
    {
        ceDebugOp.printf("BMP180 initial faile!\n");
        return CE_STATUS_FAILE;
    }
}
/**
  * @brief  CeBmp180模块操作对象定义
  */
const CeBmp180Op ceBmp180Op = {ceBmp180_initial, ceBmp180_getEnvironment,ceBmp180_getEnvironmentAsync};

#ifdef __cplusplus
}
#endif //__cplusplus
