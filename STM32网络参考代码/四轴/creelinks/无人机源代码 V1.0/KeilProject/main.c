/**
  ******************************************************************************
  * @file    main.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-23
  * @brief   CREELINKS小四轴无人机飞行器主入口程序
  ******************************************************************************
  * @attention
  *
  * 1)注意：不建议直接阅读源代码，可先简要阅览CREELINKS小四轴无人机的软件结构及框架，
  *   下载地址：http://www.creelinks.com/uav
  * 2)注意：有关CREELINKS抽象接口的定义、使用方法、移植方法，请访问官网下载相关资料。
  *   官方网站：http://www.creelinks.com/stdlib
  * 3)注意：可加入CREELINKS小四轴交流群623083844
  *
  * 1)基于CREELINKS平台V1.0
  * 2)支持WIFI、蓝牙、2.4G射频三种通讯方式，当前版本(V1.0)暂不支持蓝牙通讯
  * 3)开机时按下右按钮，直到听到滴..滴..两声后，系统进入蓝牙工作方式
  * 4)开机时按下左按钮，直到听到滴..一声后，系统进入WIFI工作方式
  * 5)开机时不按任何按钮，系统默认进入2.4G射频工作方式
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "Creelinks.h"                  /*!< CREELINKS平台头文件*/
#include "Ce6Dof.h"                     /*!< MPU6050 6轴传感器模块驱动*/
#include "CeBmp180.h"                   /*!< BMP180气压传感器模块驱动*/
#include "CePID.h"                      /*!< PID参数计算*/
#include "CeFilter.h"                   /*!< 卡尔曼滤波、四元数法姿态解算*/
#include "CeTMU.h"                      /*!< 数据传输管理模块驱动*/
#include "CeMD.h"                       /*!< 电机电调控制模块驱动*/
#include "CeLedCtl.h"                   /*!< 四个LED灯控制，通过闪烁组合，表现无人机当前的工作状态*/
#include "CeBtnx1.h"                    /*!< 无人机上，左右两个按钮模块驱动*/
#include "CePC33V.h"                    /*!< 电池电压检测模块驱动，用来实现电池电量检测*/
#include "CeBeepNSrc.h"                 /*!< 峰鸣器相关*/
#include "CeFilter.h"                   /*!< 滤波器相关*/
#include "CePackage.h"                  /*!< 数据打包解包相关*/

#define     UAV_STATUS_READY        0   /*!< 当前无人机工作状态，待命，等待起飞，此时四个电机处于停止运行状态*/
#define     UAV_STATUS_FREEDOM_FLY  1   /*!< 当前无人机工作状态，自由飞行状态，左侧遥杆控制无人机油门*/
#define     UAV_STATUS_FIX_HIGHT_FLY 2  /*!< 当前无人机工作状态，定高飞行状态，左侧遥杆控制无人机海拔高度*/
  
Ce6Dof      ce6Dof;                     /*!< 定义6轴传感器对象*/
CeBmp180    ceBmp180;                   /*!< 定义BMP180大气压力传感器对象*/
CeMD        ceMD0;                      /*!< 定义第0路电机电调控制对象*/
CeMD        ceMD1;                      /*!< 定义第1路电机电调控制对象*/
CeMD        ceMD2;                      /*!< 定义第2路电机电调控制对象*/
CeMD        ceMD3;                      /*!< 定义第3路电机电调控制对象*/
CeBtnx1     ceBtnLeft;                  /*!< 定义无人机左按钮对象*/
CeBtnx1     ceBtnRight;                 /*!< 定义无人机右按钮对象*/
CePC33V     cePC33V;                    /*!< 定义电池电压检测对象*/
CeBeepNSrc  ceBeepNSrc;                 /*!< 定义无源蜂鸣器对象*/

CeAcc       ceNowAcc;                   /*!< 定义当前无人机加速度结构体*/
CeGyr       ceNowGyr;                   /*!< 定义当前无人机角速度结构体*/
CeAngles    ceNowAngles;                /*!< 定义当前的无人机姿态*/  
CeAngles    ceHopeAngles;               /*!< 定义用户期望的无人机姿态*/    
  
Ce6DofAcceleration*    acc;             /*!< 6轴传感器加速度数据*/
Ce6DofGyroscope*       gyr;             /*!< 6轴传感器角速度数据*/
CeDrivePower*          ceDrivePower;    /*!< 定义四个电机的驱动强度结构体指针*/
CeBmp180Environment*   ceEnvirmont;     /*!< 定义BMP180采集到的海拔温度等指针数据*/
CePackageSend       cePackageSend;      /*!< 数据打包并发送管理对象*/
CePackageRecv       cePackageRecv;      /*!< 数据接收并解译管理对象*/

uint8   uavStatus = UAV_STATUS_READY;   /*!< 无人机飞行状态指示变量*/
fp32    dtS = 0;                        /*!< 程序主while执行周期，单位S*/

/**
  * @brief  发送无人机状态信息到控制端
  * @return 无
  */
void sendStatusToCtl(void) 
{
    cePackageSend.batVoltage = (int32)(cePC33VOp.getVoltage(&cePC33V)*1000);             //整合当前无人机电池电压值，用于统计电池电量,cePackageSend中的其它参数，在CePID与CeFilter中更新
    cePackageSend.pressure = (int32)(ceEnvirmont->pressure);
    cePackageSend.temperature = (int32)(ceEnvirmont->temperature*1000);
    cePackageSend.altitude = (int32)(ceEnvirmont->altitude*1000);
    cePackageSend.accelerator = (int32)(ceNowAngles.accelerator);
    if(ceTMUOp.getSendIntervalMs() >= 35)  //限制两次发送数据的时间间隔
        ceTMUOp.sendData(cePackageOp.dealSend(&cePackageSend,cePackageRecv.status),CE_PACKAGE_SEND_BUF_SIZE);    //将cePackageSend结构体中无人机状态数据打包，并发送给地面站或遥控器
}

/**
  * @brief  TMU数据传输管理模块接收数据后，调用的回调函数
  * @param  dataBuf:接收到数据的缓存地址
  * @param  dataCount:接收到数据的长度
  * @return 无
  */
void recvDataCallBack(uint8* dataBuf, uint16 dataCount) 
{
    if(CE_STATUS_SUCCESS != cePackageOp.dealRecv(&cePackageRecv,dataBuf,dataCount))//数据解算不正确，则直接返回
        return;
    ceHopeAngles.yaw = ((ceMathOp.abs(cePackageRecv.leftX) > 700) ? (-(fp32)(cePackageRecv.leftX)/50):0.0f);        //设置偏航角有正负700的死区，防止打油门时偏航有误差导致飞行不稳定
    ceHopeAngles.picth = ((ceMathOp.abs(cePackageRecv.rightY) > 150) ? ((fp32)(cePackageRecv.rightY) /166) : 0.0f);  //限制角度在-6~+6度之间,并配置Pitch角的死区，以防止摇杆误差导致的飘移
    ceHopeAngles.roll = ((ceMathOp.abs(cePackageRecv.rightX) > 150) ? (-(fp32)(cePackageRecv.rightX) /166) : 0.0f);  //限制角度在-6~+6度之间 ,并配置Roll角的死区，以防止摇杆误差导致的飘移       

    if(uavStatus == UAV_STATUS_FREEDOM_FLY)
        ceHopeAngles.accelerator = cePackageRecv.leftY/2 + 500;     //将-1000~+1000，转换为0~1000，以保证期望油门在0~1000的范围内        
    else if(uavStatus == UAV_STATUS_FIX_HIGHT_FLY)
        ceHopeAngles.altitude += (ceMathOp.abs(cePackageRecv.leftY) > 200)?((fp32)(cePackageRecv.leftY)/10000):0.0f;   //配置死区，消除摇杆位于中间时的静差。期望高度增加或减小.

    if(ceHopeAngles.picth >3)           ceLedCtlOp.setMode(CE_LED_CTL_MODE_GOTO_FRONT);
    else if(ceHopeAngles.picth <(-3))   ceLedCtlOp.setMode(CE_LED_CTL_MODE_GOTO_BACK);
    else if(ceHopeAngles.roll >3)       ceLedCtlOp.setMode(CE_LED_CTL_MODE_GOTO_LEFT);
    else if(ceHopeAngles.roll <(-3))    ceLedCtlOp.setMode(CE_LED_CTL_MODE_GOTO_RIGHT);
    else if(ceHopeAngles.yaw >3)        ceLedCtlOp.setMode(CE_LED_CTL_MODE_FLASH_CYCLE_N);
    else if(ceHopeAngles.yaw <(-3))     ceLedCtlOp.setMode(CE_LED_CTL_MODE_FLASH_CYCLE_P);
    else                                ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_NORMAL);
    if((cePackageRecv.status & CE_CTL_BTN_S2D) != 0)//遥控器按下急停按钮S2D，关闭四个电机的输出，并将无人机配置为准备状态
    {
        uavStatus = UAV_STATUS_READY;
        ceHopeAngles.accelerator = 0;
        ceHopeAngles.picth = 0;
        ceHopeAngles.roll = 0;
        ceHopeAngles.yaw = 0;
        ceHopeAngles.altitude = -9999;//用于通知PID控制器识别当前无人机是定高飞行，还是自由飞行
    }
    else if(((cePackageRecv.status & CE_CTL_BTN_RIGHT) != 0) && (cePackageRecv.leftY <= (-800)))//当控制器左摇杆拉到最低(油门小于200)，按下右摇杆按钮时,并且电池电压大于2.6V时，则进入起飞状态。
    {
        uavStatus = UAV_STATUS_FREEDOM_FLY;
        ceHopeAngles.altitude = -9999;//用于通知PID控制器识别当前无人机是定高飞行，还是自由飞行
    }  
    else if((cePackageRecv.status & CE_CTL_BTN_RIGHT) != 0)//当按下S2A时，无人机处于定高飞行状态
    {
        uavStatus = UAV_STATUS_FIX_HIGHT_FLY;
        ceHopeAngles.altitude = ceNowAngles.altitude+ ((ceNowAngles.accelerator <= 200)?1.5f:0);//当前油门小于200，说明无人机还在地面上，则此时的将期望高度配置为高于地面1.5米；当油门大于200时，说明无人机正在飞行，此时需要将期望高度配置为当前高度
    }  
}

/**
  * @brief  检测通信连接是否正常
  * @return 无
  */
void checkConnectStatus(void)
{
    if(ceTMUOp.checkConnectStatus() != CE_STATUS_SUCCESS)//如果与无人机失联，姿态归中，并发出BEEP报警提示音
    {
        if(uavStatus == UAV_STATUS_FREEDOM_FLY)
            ceHopeAngles.accelerator -= (ceHopeAngles.accelerator>10? 10:0);
        else if(uavStatus == UAV_STATUS_FIX_HIGHT_FLY)
            ceHopeAngles.altitude -=  0.1f;        

        ceHopeAngles.picth = 0;
        ceHopeAngles.roll = 0;
        ceHopeAngles.yaw = 0;
        ceBeepNSrcOp.say(&ceBeepNSrc,30,0,1);

        if(ceNowAngles.accelerator <= CE_PID_MIN_DRIVER_POWER + 400) uavStatus = UAV_STATUS_READY;//当油门小到一定值后，关闭电机并进入准备状态
        ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_CFG);//LED灯状态为配置状态

    }else if(ceLedCtlOp.getMode() == CE_LED_CTL_MODE_IN_CFG)
        ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_NORMAL);//LED灯状态为正常状态
}

/**
  * @brief  检测无人机是否发生翻机情况，如果翻机持续时间1s，则关闭电机输出，并听进入准备状态
  * @return 无
  */
void checkTurnOver(void)
{
    static fp32 turnTime = 0;
    if((uavStatus != UAV_STATUS_READY ) && ((cePackageRecv.status & CE_CTL_TYPE_STATION) == 0))//处于飞行模式，并且非地面站调试模式下，才进行翻机检测
    {
        if(ceNowAcc.z < (-0.8f)) turnTime += dtS;
        else turnTime = 0;
        if(turnTime >= 1.0f)
        {
            ceHopeAngles.picth = 0;
            ceHopeAngles.roll = 0;
            ceHopeAngles.yaw = 0;
            ceHopeAngles.accelerator = 0;
            uavStatus = UAV_STATUS_READY;
        }
    }
}

/**
  * @brief  计算程序主while循环的执行周期
  * @return while执行周期，单位s
  */
void calSystemRunCycle(void)
{
    static uint64  lastRecordSysTimeUs= 0;
    dtS = (fp32)(ceSystemOp.getSystemTickUs() - lastRecordSysTimeUs)/1000000;
    lastRecordSysTimeUs = ceSystemOp.getSystemTickUs();
    if(dtS > 0.030f) dtS = 0.030f;//限制最大周期为30ms
}

/**
  * @brief  初始化属性对象结构体数据
  * @return 无
  */
void initialParment(void)              
{
    ceHopeAngles.accelerator = 0;
    ceHopeAngles.roll = 0;
    ceHopeAngles.picth = 0;
    ceHopeAngles.yaw = 0;
    
    ceHopeAngles.altitude = -9999;
}

/**
  * @brief  初始化各功能模块
  * @return 无
*/
void initialModule(void)
{   
    ceMDOp.initial(&(ceMD0), PC6GIP);                                   //使用一路Pwm资源号初始化电机电调0
    ceMDOp.initial(&(ceMD0), PC6GIP);                                   //使用一路Pwm资源号初始化电机电调0,注：此处初始化两次PC6对应的Pwm，是因为STM32F103和TIM8定时器似乎有BUG，四路通道，初始化次数及顺序均影响输出的PWM正反向，后继再做研究。
    ceMDOp.initial(&(ceMD1), PC7GIP);                                   //使用一路Pwm资源号初始化电机电调1
    ceMDOp.initial(&(ceMD3), PC9GIP);                                   //使用一路Pwm资源号初始化电机电调3
    ceMDOp.initial(&(ceMD2), PC8GIP);                                   //使用一路Pwm资源号初始化电机电调2

    ceBeepNSrcOp.initialByGpio(&ceBeepNSrc,PD2CGI);                     //使用一路Gpio资源口初始化无源蜂鸣器

    ceLedCtlOp.initial(PC0AGI,PC1AGI,PC2AGI,PC3AGI);                    //使用4个Gpio资源号进行用来显示无人机运行状态的四个LED初始化
    ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_CFG);                         //LED灯显示为配置状态

    ceBtnx1Op.initialByGpio(&ceBtnLeft,PA11GIP,CE_NULL);                //使用Gpio口初始化左按钮模块
    ceBtnx1Op.initialByGpio(&ceBtnRight,PA12CGI,CE_NULL);               //使用Gpio口初始化右按钮模块

    
    if(ceBtnx1Op.getStatus(&(ceBtnLeft)) == 0x01)                       //打开开关时，按下左按钮，则进入WIFI通讯模式
    {
        ceBeepNSrcOp.say(&ceBeepNSrc,100,0,1);   
        ceTMUOp.initial(CE_TMU_USE_WIFI,recvDataCallBack);              //初始化数据传输对象,每20ms进行一次数据发送。注：直接正常与控制端建立通讯后函数才返回
    }
    else if(ceBtnx1Op.getStatus(&(ceBtnRight)) == 0x01)                 //打开开关时，按下右按钮，则进入蓝牙通讯模式
    {
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,2);
        ceTMUOp.initial(CE_TMU_USE_BLUE,recvDataCallBack);      
    }
    else                                                                //其它情况则进入NRF24L01通讯方式            
    {
        ceTMUOp.initial(CE_TMU_USE_NRF,recvDataCallBack); 
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,3);            
        ceSystemOp.delayMs(3000);                                       //延时一大段时间，以确保无人机处于地面静止状态，防止MPU6050初始化计算角速度零点时出现误差
    }

    ce6DofOp.initial(&ce6Dof,I2c1);                                     //使用I2c资源号初始化6轴传感器
    ceBmp180Op.initial(&ceBmp180,I2c1);                                 //使用I2c资源号初始化BMP180大气压力传感器

    cePackageOp.initialRecv(&cePackageRecv);                            //初始化数据包解译对象
    cePackageOp.initialSend(&cePackageSend);                            //初始化数据包打包对象

    ceFilterOp.initial(&cePackageSend,&cePackageRecv);                  //无人机姿态解析及滤波控制对像，四种：一、二阶、四元数、卡尔曼
    cePIDOp.initial(&cePackageSend,&cePackageRecv);                     //无人机串级PID控制对象

    cePC33VOp.initial(&cePC33V,PC4AGI);                                 //使用一路Ad资源口初始化电压测量模块

    ceBeepNSrcOp.say(&ceBeepNSrc,1000,0,1);                             //蜂鸣器发出1S响，提示已初始化完成，可以起飞
    ceLedCtlOp.setMode(CE_LED_CTL_MODE_IN_NORMAL);                      //LED灯提示为可以起飞状态
}

/**
  * @brief  CREELINKS平台主入口函数(裸奔)
  * @return 0
  */
int main(void)
{
    ceSystemOp.initial();                           //Creelinks环境初始化
    ceDebugOp.initial(Uart4);                       //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    initialParment();                               //结构体的参数初始化
    initialModule();                                //初始化所有功能模块
    while (1)
    {
        ceTaskOp.mainTask();                        //Creelinks环境主循环任务，请保证此函数能够被周期调用        
        //TODO:请在此处插入用户操作   

        acc = ce6DofOp.getAcceleration(&ce6Dof);    //获取当前无人机加速度
        gyr = ce6DofOp.getGyroscope(&ce6Dof);       //获取当前无人机角速度        
        ceEnvirmont = ceBmp180Op.getEnvironmentAsync(&ceBmp180);//获取当前无人机大气压相关数据

        ceNowAcc.x = acc->x ;                       //转存加速度数据
        ceNowAcc.y = acc->y ;  
        ceNowAcc.z = acc->z ; 

        ceNowGyr.x = -gyr->x;                       //转存角速度数据
        ceNowGyr.y = gyr->y;
        ceNowGyr.z = gyr->z;

        ceNowAngles.altitude = ceEnvirmont->altitude;//当前高度暂存

        ceFilterOp.filter(&ceNowAcc,&ceNowGyr,&ceNowAngles,dtS);            //对当前加速度、当前角速度进行姿态解析及滤波，以获取无人机姿态角数据，注：会将计算结果更新到三个指针参数所指内容。            
        ceDrivePower = cePIDOp.calculate(&ceNowAcc, &ceNowGyr, &ceNowAngles, &ceHopeAngles,dtS);   //根据当前加速度及角速度、姿态角、还有期望姿态进行串级PID运算，并获得四个电机的驱动强度                

        ceMDOp.setDriverPower(&ceMD0,(uavStatus != UAV_STATUS_READY)? (ceDrivePower->driverPower0):0);    //配置第0路电机驱动强度，0~1000                
        ceMDOp.setDriverPower(&ceMD1,(uavStatus != UAV_STATUS_READY)? (ceDrivePower->driverPower1):0);    //配置第1路电机驱动强度，0~1000
        ceMDOp.setDriverPower(&ceMD2,(uavStatus != UAV_STATUS_READY)? (ceDrivePower->driverPower2):0);    //配置第2路电机驱动强度，0~1000
        ceMDOp.setDriverPower(&ceMD3,(uavStatus != UAV_STATUS_READY)? (ceDrivePower->driverPower3):0);    //配置第3路电机驱动强度，0~1000

        sendStatusToCtl();                          //整合当前数据，并发送给控制端
        checkConnectStatus();                       //检测连接是否断开，如果断开，无人机姿态规中并缓慢下降
        calSystemRunCycle();                        //计算程序执行周期时间
        checkTurnOver();                            //检测无人机是否翻机，如果是则关闭电机输出，以保护MOS管及电机
    };
}
