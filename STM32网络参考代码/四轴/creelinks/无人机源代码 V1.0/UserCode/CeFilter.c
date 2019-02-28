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
#include "CeFilter.h"
#include "Math.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  滑动平均滤波初始化
  * @param  ceSlidefilter:滑动平均滤波属性对象
  */
void ceFilterSlider_initial(CeFilterSlider* ceFilterSlider)
{
    int i;
    for(i=0;i<CE_SLIDE_FILTER_SIZE;i++)
    {
        ceFilterSlider->array[i]= 0.0f;
    }
}
/**
  * @brief  输入新值，返回滤波后的值
  * @param  ceSlidefilter:滑动平均滤波属性对象
  * @param  newVal:未进行滤波的新值
  * @return 滤波后的值
  */
fp32 ceFilterSlider_filter(CeFilterSlider* ceFilterSlider,fp32 newVal)
{
    fp32 sum = 0;
    int i;    
    for(i=0;i<CE_SLIDE_FILTER_SIZE-1;i++)
    {
        ceFilterSlider->array[i] = ceFilterSlider->array[i+1];
    }
    ceFilterSlider->array[CE_SLIDE_FILTER_SIZE-1] = newVal;

    for(i=0;i<CE_SLIDE_FILTER_SIZE;i++)
    {
        sum += ceFilterSlider->array[i];
    }
    return sum/CE_SLIDE_FILTER_SIZE;
}
/**
  * @brief  初始滑动平均滤波操作对象
  */
const CeFilterSliderOp ceFilterSliderOp = { ceFilterSlider_initial,ceFilterSlider_filter };


/**
  * @brief  毛刺滤波器初始化
  * @param  ceFilterBase:毛刺滤波器属性对象
  * @param  maxAbs:两次数据之差绝对值的最大值
  */
void ceFilterBase_initial(CeFilterBase* ceFilterBase, fp32 maxAbs)
{
    ceFilterBase->lastVal = 0;
    ceFilterBase->index = 0;
    ceFilterBase->isUp = 0;
    ceFilterBase->maxAbs = maxAbs;
    ceFilterBase->coes[0] = 0.00f;
    ceFilterBase->coes[1] = 0.00f;
    ceFilterBase->coes[2] = 0.00f;
    ceFilterBase->coes[3] = 0.00f;
    ceFilterBase->coes[4] = 0.00f;
    ceFilterBase->coes[5] = 0.01f;
    ceFilterBase->coes[6] = 0.02f;
    ceFilterBase->coes[7] = 0.03f;
    ceFilterBase->coes[8] = 0.04f;
    ceFilterBase->coes[9] = 0.05f;
}
/**
  * @brief  对数据进行毛刺滤波，根据新值计算旧值
  * @param  ceFilterBase:毛刺滤波器属性对象
  * @param  newVal:新采集到的待滤波的值
  * @return 滤波后的值
  */
fp32 ceFilterBase_filter(CeFilterBase* ceFilterBase, fp32 newVal)
{
    if (ceMathOp.abs(newVal - ceFilterBase->lastVal) > ceFilterBase->maxAbs)
    {
        fp32 n = 0;
        if (newVal >= 0)
        {
            if (ceFilterBase->isUp == 0x00)
            {
                ceFilterBase->index = 0;
            }
            ceFilterBase->isUp = 0x01;
            n = ceFilterBase->lastVal + ceMathOp.abs(ceFilterBase->lastVal - newVal) * ceFilterBase->coes[ceFilterBase->index];
        }
        else
        {
            if (ceFilterBase->isUp == 0x01)
            {
                ceFilterBase->index = 0;
            }
            ceFilterBase->isUp = 0x00;
            n = ceFilterBase->lastVal - ceMathOp.abs(newVal - ceFilterBase->lastVal) * ceFilterBase->coes[ceFilterBase->index];
        }
        ceFilterBase->index++;
        if (ceFilterBase->index >= 10)
        {
            ceFilterBase->index = 0;
            ceFilterBase->lastVal = newVal;
            return newVal;
        }
        return n;
    }
    else
    {
        ceFilterBase->lastVal = newVal;
        ceFilterBase->index = 0;
        return newVal;
    }
}

CeFilterBaseOp ceFilterBaseOp = {ceFilterBase_initial,ceFilterBase_filter};


/**
  * @brief  一阶滤波初始化
  * @param  ceFilterYijie:一阶滤波属性对象
  */
void ceFilterYijie_Yinitial(CeFilterYijie* ceFilterYijie)
{
    ceFilterYijie->K1 = 0.05f;
    ceFilterYijie->angle = 0.0f;
}
/**
  * @brief  输入新值，返回滤波后的角度值，详细可参考CREELINKS相关文档
  * @param  ceFilterYijie:一阶滤波属性对象
  * @param  angle_m:未滤波的由加速度直接获取的姿态角度
  * @param  gyro_m:未滤波的角速度
  * @return 滤波后的角度值
  */
fp32 ceFilterYijie_filter(CeFilterYijie* ceFilterYijie, fp32 angle_m, fp32 gyro_m,fp32 dt)
{
    ceFilterYijie->angle = ceFilterYijie->K1 * angle_m + (1 - ceFilterYijie->K1) * (ceFilterYijie->angle + gyro_m * dt);
    return ceFilterYijie->angle;
}
/**
* @brief  初始化一阶滤波操作对象
*/
const CeFilterYijieOp ceFilterYijieOp = { ceFilterYijie_Yinitial ,ceFilterYijie_filter};




/**
  * @brief  二阶滤波初始化
  * @param  ceFilterErjie:二阶滤波滤波属性对象
  */
void ceFilterErjie_initial(CeFilterErjie* ceFilterErjie)
{
    ceFilterErjie->K2 = 0.05f;
    ceFilterErjie->angle = 0.0f;
    ceFilterErjie->x1 = 0.0f;
    ceFilterErjie->x2 = 0.0f;
    ceFilterErjie->y1 = 0.0f;
}
/**
  * @brief  输入新值，返回滤波后的角度值，详细可参考CREELINKS相关文档
  * @param  ceFilterErjie:二阶滤波属性对象
  * @param  angle_m:未滤波的由加速度直接获取的姿态角度
  * @param  gyro_m:未滤波的角速度
  * @return 滤波后的角度值
  */
fp32 ceFilterErjie_filter(CeFilterErjie* ceFilterErjie, fp32 angle_m, fp32 gyro_m,fp32 dt)
{
    ceFilterErjie->x1 = (angle_m - ceFilterErjie->angle)*(1 - ceFilterErjie->K2)*(1 - ceFilterErjie->K2);
    ceFilterErjie->y1 = ceFilterErjie->y1 + ceFilterErjie->x1*dt;
    ceFilterErjie->x2 = ceFilterErjie->y1 + 2 * (1 - ceFilterErjie->K2)*(angle_m - ceFilterErjie->angle) + gyro_m;
    ceFilterErjie->angle = ceFilterErjie->angle + ceFilterErjie->x2*dt;
    return ceFilterErjie->angle;
}
/**
  * @brief  初始化二阶滤波操作对象
  */
const CeFilterErjieOp ceFilterErjieOp = { ceFilterErjie_initial,ceFilterErjie_filter };




/**
  * @brief  四元数+互补滤波初始化
  * param  ceFilterIMU:四元数+互补滤波属性对象
  */
void ceFilterIMU_initial(CeFilterIMU* ceFilterIMU)
{
    ceFilterIMU->q0 = 1;
    ceFilterIMU->q1 = 0;
    ceFilterIMU->q2 = 0;
    ceFilterIMU->q3 = 0;

    ceFilterIMU->kp = 3.000f;
    ceFilterIMU->ki = 0.002f;
}
/**
  * @brief  输入新值，返回滤波后的角度值，详细可参考CREELINKS相关文档
  * @param  ceFilterIMU:四元数+互补滤波属性对象
  * @param  nowAcc:当前无人机加速度数据
  * @param  ceNowGyr:当前无人机角速度数据
  * @param  ceNowAngle:当前无人机姿态数据，四元数姿态角计算完毕后会修改此指针中的内容
  */
void ceFilterIMU_filter(CeFilterIMU* ceFilterIMU, CeAcc* nowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngle,fp32 halfT)
{
    float  norm;
    float  vx, vy, vz;
    float  ex, ey, ez;
    float ax, ay, az;
    float gx, gy, gz;

    float q0 = ceFilterIMU->q0;
    float q1 = ceFilterIMU->q1;
    float q2 = ceFilterIMU->q2;
    float q3 = ceFilterIMU->q3;

    float  q0q0 = q0*q0;
    float  q0q1 = q0*q1;
    float  q0q2 = q0*q2;
    //float  q0q3 = q0*q3;//有地磁传感器后，才会使用到的数据
    float  q1q1 = q1*q1;
    //float  q1q2 = q1*q2;//有地磁传感器后，才会使用到的数据
    float  q1q3 = q1*q3;
    float  q2q2 = q2*q2;
    float  q2q3 = q2*q3;
    float  q3q3 = q3*q3;
    float  exInt = 0, eyInt = 0, ezInt = 0;

    ax = nowAcc->x;
    ay = nowAcc->y;
    az = nowAcc->z;

    gx = ceNowGyr->x * 0.0174533;//角度转弧度
    gy = ceNowGyr->y * 0.0174533;
    gz = ceNowGyr->z * 0.0174533;

    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    exInt = exInt + ex * ceFilterIMU->ki;
    eyInt = eyInt + ey * ceFilterIMU->ki;
    ezInt = ezInt + ez * ceFilterIMU->ki;

    gx = gx + ceFilterIMU->kp*ex + exInt;
    gy = gy + ceFilterIMU->kp*ey + eyInt;
    gz = gz + ceFilterIMU->kp*ez + ezInt;

    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    ceFilterIMU->q0 = q0;
    ceFilterIMU->q1 = q1;
    ceFilterIMU->q2 = q2;
    ceFilterIMU->q3 = q3;

    ceNowAngle->picth = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.2957795f; // pitch
    ceNowAngle->roll = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.2957795f; // roll    
    //ceNowAngle->yaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.2957795f;//未使用地磁传感器的情况下，偏航角会有偏移，所有不对此角进行解析

    ceNowGyr->x = gx/0.0174533;
    ceNowGyr->y = gy/0.0174533;
    ceNowGyr->z = gz/0.0174533;

    //ceIMU.anglesNow.picth = -atan(ax / sqrt(ay*ay + az*az))*57.2957795f;
    //ceIMU.anglesNow.roll = -atan(ay / sqrt(ax*ax + az*az))*57.2957795f;
}
/**
  * @brief  初始化四元数+互补操作对象
  */
const CeFilterIMUOp ceFilterIMUOp = {ceFilterIMU_initial,ceFilterIMU_filter};



/**
  * @brief  卡尔曼滤波初始化
  * @param  ceFilterErjie:卡尔曼滤波属性对象
  */
void ceFilterKalman_initial(CeFilterKalman* ceFilterKalman)
{
    ceFilterKalman->angle = 0;
    ceFilterKalman->angle_dot = 0;

    ceFilterKalman->P[0][0] = 1;
    ceFilterKalman->P[0][1] = 0;
    ceFilterKalman->P[1][0] = 0;
    ceFilterKalman->P[1][1] = 1;

    ceFilterKalman->Pdot[0] = 0;
    ceFilterKalman->Pdot[1] = 0;
    ceFilterKalman->Pdot[2] = 0;
    ceFilterKalman->Pdot[3] = 0;

    ceFilterKalman->R_angle =  0.020f;
    ceFilterKalman->Q_angle = 0.006f;  //角度数据置信度,角速度数据置信度
    ceFilterKalman->Q_gyro = 0.010f;

    ceFilterKalman->C_0 = 1;

    ceFilterKalman->q_bias = 0;//这里需要初始化为0？
    ceFilterKalman->angle_err = 0;
    ceFilterKalman->PCt_0 = 0;
    ceFilterKalman->PCt_1 = 0;
    ceFilterKalman->E = 0;
    ceFilterKalman->K_0 = 0;
    ceFilterKalman->K_1 = 0;
    ceFilterKalman->t_0 = 0;
    ceFilterKalman->t_1 = 0;
}
/**
  * @brief  输入新值，返回滤波后的角度值，详细可参考CREELINKS相关文档
  * @param  ceFilterErjie:卡尔曼滤波属性对象
  * @param  angle_m:未滤波的由加速度直接获取的姿态角度，计算完成后会将计算结果写入此指针地址
  * @param  gyro_m:未滤波的角速度，计算完成后会将计算结果写入此指针地址
  */
void ceFilterKalman_filter(CeFilterKalman* ceFilterKalman,fp32* angle_m, fp32* gyro_m,fp32 dt)
{
    ceFilterKalman->angle += (*gyro_m - ceFilterKalman->q_bias) * dt;
    ceFilterKalman->angle_err = *angle_m - ceFilterKalman->angle;

    ceFilterKalman->Pdot[0] = ceFilterKalman->Q_angle - ceFilterKalman->P[0][1] - ceFilterKalman->P[1][0];
    ceFilterKalman->Pdot[1] = -ceFilterKalman->P[1][1];
    ceFilterKalman->Pdot[2] = -ceFilterKalman->P[1][1];
    ceFilterKalman->Pdot[3] = ceFilterKalman->Q_gyro;
    ceFilterKalman->P[0][0] += ceFilterKalman->Pdot[0] * dt;
    ceFilterKalman->P[0][1] += ceFilterKalman->Pdot[1] * dt;
    ceFilterKalman->P[1][0] += ceFilterKalman->Pdot[2] * dt;
    ceFilterKalman->P[1][1] += ceFilterKalman->Pdot[3] * dt;
    ceFilterKalman->PCt_0 = ceFilterKalman->C_0 * ceFilterKalman->P[0][0];
    ceFilterKalman->PCt_1 = ceFilterKalman->C_0 * ceFilterKalman->P[1][0];
    ceFilterKalman->E = ceFilterKalman->R_angle + ceFilterKalman->C_0 * ceFilterKalman->PCt_0;
    ceFilterKalman->K_0 = ceFilterKalman->PCt_0 / ceFilterKalman->E;
    ceFilterKalman->K_1 = ceFilterKalman->PCt_1 / ceFilterKalman->E;
    ceFilterKalman->t_0 = ceFilterKalman->PCt_0;
    ceFilterKalman->t_1 = ceFilterKalman->C_0 * ceFilterKalman->P[0][1];
    ceFilterKalman->P[0][0] -= ceFilterKalman->K_0 * ceFilterKalman->t_0;
    ceFilterKalman->P[0][1] -= ceFilterKalman->K_0 * ceFilterKalman->t_1;
    ceFilterKalman->P[1][0] -= ceFilterKalman->K_1 * ceFilterKalman->t_0;
    ceFilterKalman->P[1][1] -= ceFilterKalman->K_1 * ceFilterKalman->t_1;
    ceFilterKalman->angle += ceFilterKalman->K_0 * ceFilterKalman->angle_err; //最优角度
    ceFilterKalman->q_bias += ceFilterKalman->K_1 * ceFilterKalman->angle_err;

    ceFilterKalman->angle_dot = *gyro_m - ceFilterKalman->q_bias;//最优角速度

    *angle_m = ceFilterKalman->angle;
    *gyro_m = ceFilterKalman->angle_dot;
}
/**
  * @brief 初始化卡尔曼操作对象
  */
const CeFilterKalmanOp ceFilterKalmanOp = {ceFilterKalman_initial,ceFilterKalman_filter};

/**
  * @brief 全局变量
  */
CeFilter ceFilter;
/**
  * @brief 根据cePackageRecv中的内容，实时对滤波等算法的参数进行更新调试
  */
void ceFilter_updataFilterParment(void)
{
    if((ceFilter.ceFilterType & ceFilter.cePackageRecv->status) == 0)//已切换滤波算法
    {
        if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_YIJIEHUBU) != 0)
        {
            ceFilter.ceFilterType = CE_FILTER_IN_YIJIEHUBU;
            ceFilterYijieOp.initial(&(ceFilter.ceFilterYijiePitch));
            ceFilterYijieOp.initial(&(ceFilter.ceFilterYijieRoll));
        }
        else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_ERJIEHUBU) != 0)
        {
            ceFilter.ceFilterType = CE_FILTER_IN_ERJIEHUBU;
            ceFilterErjieOp.initial(&(ceFilter.ceFilterErjiePitch));
            ceFilterErjieOp.initial(&(ceFilter.ceFilterErjieRoll));
        }
        else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_IMU) != 0)
        {
            ceFilter.ceFilterType = CE_FILTER_IN_IMU;
            ceFilterIMUOp.initial(&(ceFilter.ceFilterIMU));
        }
        else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_KALMAN) != 0)
        {
            ceFilter.ceFilterType = CE_FILTER_IN_KALMAN;
            ceFilterKalmanOp.initial(&(ceFilter.ceFilterKalmanPitch));
            ceFilterKalmanOp.initial(&(ceFilter.ceFilterKalmanRoll));
        }
    }

		if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_ANGLE_ZERO) != 0)
    {
        ceFilter.ceAnglesZero.picth = (fp32)(ceFilter.cePackageRecv->zeroPitch) / 1000;
        ceFilter.ceAnglesZero.roll = (fp32)(ceFilter.cePackageRecv->zeroRoll) / 1000;
        ceFilter.ceAnglesZero.yaw = (fp32)(ceFilter.cePackageRecv->zeroYaw) / 1000;
    }
		
    if((ceFilter.cePackageRecv->status & CE_FILTER_IN_DEBUG) == 0)
        return;

    if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_YIJIEHUBU) != 0)//如果处于参数调节状态，则更新IMU与滤波器的参数
    {
        ceFilter.ceFilterYijiePitch.K1 = (fp32)(ceFilter.cePackageRecv->yijieK1) / 1000;
        ceFilter.ceFilterYijiePitch.K1 = (fp32)(ceFilter.cePackageRecv->yijieK1) / 1000;
    }
    else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_ERJIEHUBU) != 0)
    {
        ceFilter.ceFilterErjiePitch.K2 = (fp32)(ceFilter.cePackageRecv->erjieK2) / 1000;
        ceFilter.ceFilterErjieRoll.K2 = (fp32)(ceFilter.cePackageRecv->erjieK2) / 1000;
    }
    else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_IMU) != 0)
    {
        ceFilter.ceFilterIMU.kp = (fp32)(ceFilter.cePackageRecv->imuKp) / 1000;
        ceFilter.ceFilterIMU.ki = (fp32)(ceFilter.cePackageRecv->imuKi) / 1000;
    }
    else if ((ceFilter.cePackageRecv->status & CE_FILTER_IN_KALMAN) != 0)
    {
        ceFilter.ceFilterKalmanPitch.R_angle = (fp32)(ceFilter.cePackageRecv->filterR_angle) / 1000;
        ceFilter.ceFilterKalmanPitch.Q_angle = (fp32)(ceFilter.cePackageRecv->filterQ_angle) / 1000;
        ceFilter.ceFilterKalmanPitch.Q_gyro = (fp32)(ceFilter.cePackageRecv->filterQ_gyro) / 1000;
            
        ceFilter.ceFilterKalmanRoll.R_angle = (fp32)(ceFilter.cePackageRecv->filterR_angle) / 1000;
        ceFilter.ceFilterKalmanRoll.Q_angle = (fp32)(ceFilter.cePackageRecv->filterQ_angle)/ 1000;
        ceFilter.ceFilterKalmanRoll.Q_gyro = (fp32)(ceFilter.cePackageRecv->filterQ_gyro) / 1000;
    }

}
/**
  * @brief  滤波初始化
  * @param  cePackageSend:数据打包并发送使用的结构体
  * @param  cePackageRecv:数据拆包并解析使用的结构体
  */
void ceFilter_initial(CePackageSend* cePackageSend, CePackageRecv* cePackageRecv)
{
    ceFilter.cePackageSend = cePackageSend;                 //保存发送打包属性结构体，用于传输姿态解算中间数值以供观察
    ceFilter.cePackageRecv = cePackageRecv;
    ceFilter.ceFilterType = CE_FILTER_IN_KALMAN;            //默认Kalman滤波方式
    ceFilter.dt = 0.0f;

    ceFilter.ceAnglesZero.picth = 0;
    ceFilter.ceAnglesZero.roll = 0;
    ceFilter.ceAnglesZero.yaw = 0;

    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderAccX));
    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderAccY));
    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderAccZ));

    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderGyrX));
    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderGyrY));
    ceFilterSliderOp.initial(&(ceFilter.ceFilterSliderGyrZ));

    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseAccX),0.2);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseAccY),0.2);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseAccZ),0.2);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseGyrX),60);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseGyrY),60);
    ceFilterBaseOp.initial(&(ceFilter.ceFilterBaseGyrZ),60);

    ceFilterYijieOp.initial(&(ceFilter.ceFilterYijiePitch));
    ceFilterYijieOp.initial(&(ceFilter.ceFilterYijieRoll));


    ceFilterErjieOp.initial(&(ceFilter.ceFilterErjiePitch));
    ceFilterErjieOp.initial(&(ceFilter.ceFilterErjieRoll));

    ceFilterIMUOp.initial(&(ceFilter.ceFilterIMU));

    ceFilterKalmanOp.initial(&(ceFilter.ceFilterKalmanPitch));
    ceFilterKalmanOp.initial(&(ceFilter.ceFilterKalmanRoll));
}

/**
  * @brief  输入当前角速度及加速度来计算当前无人机的姿态角，详细可参考CREELINKS相关文档
  * @param  nowAcc:当前无人机加速度数据
  * @param  ceNowGyr:当前无人机角速度数据
  * @param  ceNowAngle:当前无人机姿态数据，四元数姿态角计算完毕后会修改此指针中的内容
  */
void ceFilter_filter(CeAcc* ceNowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngle, fp32 dtS)
{
    fp32 norm = 0;
    fp32 ax, ay, az;
    fp32 picthByAcc=0, rollByAcc=0;
		fp32 temp = 0;

    ceFilter_updataFilterParment();//根据cePackageRecv中的内容，实时对滤波等算法的参数进行更新调试
    
    ceFilter.cePackageSend->accX = (int32)(ceNowAcc->x * 1000);//保存未滤波的加速度数据，以供地面站观察波形，放大100位便于fp32以Int16的方式进行数据传输
    ceFilter.cePackageSend->accY = (int32)(ceNowAcc->y * 1000);
    ceFilter.cePackageSend->accZ = (int32)(ceNowAcc->z * 1000);
    ceFilter.cePackageSend->gyrX = (int32)(ceNowGyr->x * 1000);//保存未滤波的角速度数据，以供地面站观察波形，注：仅有四元数和Kalman滤波算法，会校正角速度漂移
    ceFilter.cePackageSend->gyrY = (int32)(ceNowGyr->y * 1000);
    ceFilter.cePackageSend->gyrZ = (int32)(ceNowGyr->z * 1000);

    ceNowAcc->x = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseAccX), ceNowAcc->x);//对角速度进行限幅滤波去毛刺
    ceNowAcc->y = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseAccY), ceNowAcc->y);
    ceNowAcc->z = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseAccZ), ceNowAcc->z);

    ceNowGyr->x = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseGyrX), ceNowGyr->x);//对加速度进行限幅滤波去毛刺
    ceNowGyr->y = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseGyrY), ceNowGyr->y);
    ceNowGyr->z = ceFilterBaseOp.filter(&(ceFilter.ceFilterBaseGyrZ), ceNowGyr->z);

    ceNowAcc->x = ceFilterSliderOp.filter(&(ceFilter.ceFilterSliderAccX), ceNowAcc->x);//对加速度进行滑动平均滤波
    ceNowAcc->y = ceFilterSliderOp.filter(&(ceFilter.ceFilterSliderAccY), ceNowAcc->y);
    ceNowAcc->z = ceFilterSliderOp.filter(&(ceFilter.ceFilterSliderAccZ), ceNowAcc->z);
       
    norm = sqrt(ceNowAcc->x*ceNowAcc->x + ceNowAcc->y*ceNowAcc->y + ceNowAcc->z*ceNowAcc->z);//加速度单位化
    ax = ceNowAcc->x / norm;
    ay = ceNowAcc->y / norm;
    az = ceNowAcc->z / norm;

    picthByAcc = -atan(ax / sqrt(ay*ay + az*az))*57.2957795f;//加速度直接解算出姿态角
    rollByAcc = -atan(ay / sqrt(ax*ax + az*az))*57.2957795f;

    ceFilter.cePackageSend->pitchByAcc = (int32)(picthByAcc * 1000);//保存未滤波前由加速度直接解析的姿态角数据，以供地面站观察波形
    ceFilter.cePackageSend->rollByAcc = (int32)(rollByAcc * 1000);
    ceFilter.cePackageSend->yawByAcc = (int32)(0 * 1000);

    ceFilter.cePackageSend->pitchByGyr = (int16)((ceFilter.cePackageSend->pitchByGyr + ceFilter.dt*ceNowGyr->y) * 100);//保存未滤波前由角速度直接解析的姿态角数据，以供地面站观察波形
    ceFilter.cePackageSend->rollByGyr = (int16)((ceFilter.cePackageSend->rollByGyr + ceFilter.dt*ceNowGyr->x) * 100);
    ceFilter.cePackageSend->yawByGyr = (int16)((ceFilter.cePackageSend->yawByGyr + ceFilter.dt*ceNowGyr->z) * 100);

    switch (ceFilter.ceFilterType)
    {
    case CE_FILTER_IN_YIJIEHUBU:
        ceNowAngle->picth = ceFilterYijieOp.filter(&(ceFilter.ceFilterYijiePitch), picthByAcc, ceNowGyr->y,dtS);//采用一阶互补对无人机进行姿态解析及滤波
        ceNowAngle->roll = ceFilterYijieOp.filter(&(ceFilter.ceFilterYijieRoll), rollByAcc, ceNowGyr->x,dtS);
        break;
    case CE_FILTER_IN_ERJIEHUBU:
        ceNowAngle->picth = ceFilterErjieOp.filter(&(ceFilter.ceFilterErjiePitch), picthByAcc, ceNowGyr->y,dtS);//采用二阶互补对无人机进行姿态解析及滤波
        ceNowAngle->roll = ceFilterErjieOp.filter(&(ceFilter.ceFilterErjieRoll), rollByAcc, ceNowGyr->x,dtS);
        break;
    case CE_FILTER_IN_IMU:
        ceNowGyr->x = -ceNowGyr->x;
        ceFilter.cePackageSend->gyrX = -ceFilter.cePackageSend->gyrX;
        ceFilterIMUOp.filter(&(ceFilter.ceFilterIMU), ceNowAcc, ceNowGyr, ceNowAngle,dtS/2);          //采用四元数+互补滤波对无人机进行姿态解析
        break;
    case CE_FILTER_IN_KALMAN:
        temp = ceNowGyr->x;
        ceFilterKalmanOp.filter(&(ceFilter.ceFilterKalmanRoll), &rollByAcc, &temp,dtS);   
        temp = ceNowGyr->y;
        ceFilterKalmanOp.filter(&(ceFilter.ceFilterKalmanPitch), &picthByAcc, &temp,dtS);  //采用Kalman滤波算法对无人机姿态进行解析，注：参数传入的为指针，函数内部修改指针值为最新计划的结果
        ceNowAngle->picth = picthByAcc;
        ceNowAngle->roll = rollByAcc;
        break;
    default:
        break;
    }
    ceNowAngle->picth += ceFilter.ceAnglesZero.picth;   //校正姿态角的零点，减少无人机漂移
    ceNowAngle->roll += ceFilter.ceAnglesZero.roll;
    ceNowAngle->yaw += ceFilter.ceAnglesZero.yaw;

    ceFilter.cePackageSend->accXByFilter = (int32)(ceNowAcc->x*1000);        //保存滑动滤波后的加速度数据，以供地面站观察波形
    ceFilter.cePackageSend->accYByFilter = (int32)(ceNowAcc->y * 1000);
    ceFilter.cePackageSend->accZByFilter = (int32)(ceNowAcc->z * 1000);
    ceFilter.cePackageSend->gyrXByFilter = (int32)(ceNowGyr->x * 1000);      //保存需点漂移校正过后的角速度数据，以供地面站观察波形，注：仅有四元数和Kalman滤波算法，会校正角速度漂移
    ceFilter.cePackageSend->gyrYByFilter = (int32)(ceNowGyr->y * 1000);
    ceFilter.cePackageSend->gyrZByFilter = (int32)(ceNowGyr->z * 1000);
    ceFilter.cePackageSend->pitchByFilter = (int32)(ceNowAngle->picth * 1000);//保存对加速度及角速度两者获取的姿态角进行融合滤波后的姿态角数据，以供地面站观察波形
    ceFilter.cePackageSend->rollByFilter = (int32)(ceNowAngle->roll * 1000);
    ceFilter.cePackageSend->yawByFilter = (int32)(0 * 1000);
}
/**
  * @brief  初始化CeFilter操作对象实例
  */
const CeFilterOp ceFilterOp = { ceFilter_initial ,ceFilter_filter };

#ifdef __cplusplus
 }
#endif //__cplusplus
