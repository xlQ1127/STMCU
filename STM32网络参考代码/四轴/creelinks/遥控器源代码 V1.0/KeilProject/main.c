/**
  ******************************************************************************
  * @file    main.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-23
  * @brief   CREELINKS小四轴无人机控制器主入口程序
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
  * 3)开机时按下S2C，直到听到滴..滴..两声后，系统进入蓝牙工作方式
  * 4)开机时按下S2D，直到听到滴..一声后，系统进入WIFI工作方式
  * 5)开机时不按任何按钮，系统默认进入2.4G射频工作方式
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "Creelinks.h"          /*!< CREELINKS平台头文件*/
#include "CeBtnx1.h"            /*!< 无人机上，左右两个按钮模块驱动*/
#include "CePC33V.h"            /*!< 电池电压检测模块驱动，用来实现电池电量检测*/
#include "CeBeepNSrc.h"         /*!< 峰鸣器相关*/
#include "CePackage.h"          /*!< 数据打包解包相关*/
#include "CeJoystick.h"         /*!< 摇杆相关*/
#include "CeTft180.h"           /*!< TFT显示屏相关*/
#include "CeTMU.h"              /*!< 通讯管理相关*/
#include "CeLed1C.h"            /*!< LED灯相关*/

#define CE_UAV_CTL_TEST     0   /*!< 配置当前遥控器处于测试状态*/
#define CE_UAV_CTL_NORMAL   1   /*!< 配置当前遥控器处于正常工作状态*/

CeBeepNSrc  ceBeepNSrc;         /*!< 定义无源蜂鸣器对象*/
CePC33V     cePC33V;            /*!< 定义电池电压检测对象*/
CeJoystick  ceJoystickLeft;     /*!< 定义左侧摇杆对象*/
CeJoystick  ceJoystickRight;    /*!< 定义右侧摇杆对象*/
CeTft180    ceTft180;           /*!< 定义TFT180显示屏对象*/

CeBtnx1     ceBtnS2A;           /*!< 定义S2A按钮对象*/
CeBtnx1     ceBtnS2B;           /*!< 定义S2B按钮对象*/
CeBtnx1     ceBtnS2C;           /*!< 定义S2C按钮对象*/
CeBtnx1     ceBtnS2D;           /*!< 定义S2D按钮对象*/

CeLed1C     ceLed1A;            /*!< 定义LED灯A对象*/
CeLed1C     ceLed1B;            /*!< 定义LED灯B对象*/
CeLed1C     ceLed1C;            /*!< 定义LED灯C对象*/
CeLed1C     ceLed1D;            /*!< 定义LED灯D对象*/

extern CeTMU    ceTMU;          /*!< 引用数据通讯管理对象*/

CePackageSend cePackageSend;    /*!< 定义数据打包并发送对象*/
CePackageRecv cePackageRecv;    /*!< 定义数据拆包并解析对象*/

uint8   workModel = CE_UAV_CTL_NORMAL;   /*!< 指示当前遥控器的工作状态*/
/**
  * @brief  发送遥控器状态到无人机
  * @return 无
  */
void sendStatusToCtl()
{
    CeJoystickAxis* axis;

    ceLed1COp.setOn(&ceLed1D);                          //打开发送提示LED灯LED1D
    axis = ceJoystickOp.getAxis(&ceJoystickLeft);       //获取左摇杆的位置，X轴－1000~+1000，Y轴-1000~+1000
    cePackageSend.leftX = axis->y;                      //因使用了CREELINKS摇杆的现在模块及驱动库，而模块的安装方向与CREELINKS驱动中的方向有差异，这里进行X轴及Y轴更换
    cePackageSend.leftY = -axis->x;
    axis = ceJoystickOp.getAxis(&ceJoystickRight);
    cePackageSend.rightX = -axis->y;
    cePackageSend.rightY = axis->x;
    cePackageSend.status =                              //整合6个按钮的数据，bit0:leftBtn, bit1:rightBtn, bit2:S2A, bit3:S2B, bit4:S2C, bit5:S2D
        ((ceJoystickOp.getBtnStatus(&ceJoystickLeft)== 0x00 )? 0x0000:CE_CTL_BTN_LEFT) | ((ceJoystickOp.getBtnStatus(&ceJoystickRight)== 0x00 )? 0x0000:CE_CTL_BTN_RIGHT)| 
        ((ceBtnx1Op.getStatus(&ceBtnS2A)== 0x00 )? 0x0000:CE_CTL_BTN_S2A)|((ceBtnx1Op.getStatus(&ceBtnS2B) == 0x00 )? 0x0000:CE_CTL_BTN_S2B)| 
        ((ceBtnx1Op.getStatus(&ceBtnS2C) == 0x00 )? 0x0000:CE_CTL_BTN_S2C)|((ceBtnx1Op.getStatus(&ceBtnS2D) == 0x00 )? 0x0000:CE_CTL_BTN_S2D) | CE_FILTER_IN_KALMAN | CE_CTL_TYPE_CTL;
    //其它PID等参数默认为0，发送时仅发送第一包，其它PID参数不用发送给无人机
    ceTMUOp.sendData(cePackageOp.dealSend(&cePackageSend),CE_PACKAGE_PACK_SIZE);//只发一帧过去即可
    ceLed1COp.setOff(&ceLed1D);                         //关闭发送控制LED灯LED1D
}

/**
  * @brief   将当前无人机姿态数据更新到TFT屏上，注：SPI通讯方式速度不高，此函数会花费过多时间
  * @return 无
  */
void showStatus()
{
    char temp[32];
    if(workModel == CE_UAV_CTL_TEST)
    {
        CeJoystickAxis* axis;
        uint16 showIndex = 0;
        axis = ceJoystickOp.getAxis(&ceJoystickLeft);   //获取左摇杆的位置，X轴－1000~+1000，Y轴-1000~+1000
        ceDebugOp.sprintf(temp,"LeftX:%d        ",axis->y);
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"LeftY:%d        ",-axis->x);
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        axis = ceJoystickOp.getAxis(&ceJoystickRight);
        ceDebugOp.sprintf(temp,"RightX:%d        ",-axis->y);
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"RightY:%d        ",axis->x);
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        
        showIndex++;
        ceDebugOp.sprintf(temp,"LeftBtn:%s        ",((ceJoystickOp.getBtnStatus(&ceJoystickLeft) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"RightBtn:%s        ",(ceJoystickOp.getBtnStatus(&ceJoystickRight) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        
        showIndex++;
        ceDebugOp.sprintf(temp,"S2A:%s        ",((ceBtnx1Op.getStatus(&ceBtnS2A) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"S2B:%s        ",((ceBtnx1Op.getStatus(&ceBtnS2B) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"S2C:%s        ",((ceBtnx1Op.getStatus(&ceBtnS2C) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"S2D:%s        ",((ceBtnx1Op.getStatus(&ceBtnS2D) == 0x01) ? "Down":"Up"));
        ceTft180Op.showString(&ceTft180,0,(showIndex++)*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
    }else
    {
        uint8 static showTime = 0;//TFT180刷新率只有10HZ，为避免长时间堵塞mainTask()函数，这里采用分断更新方式，确保mainTask()调用最大周期小于35ms

        ceDebugOp.sprintf(temp,"Send:%d        ",ceTMU.sendPackCount);
        ceTft180Op.showString(&ceTft180,0,0*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        ceDebugOp.sprintf(temp,"recv:%d        ",ceTMU.recvPackCount);
        ceTft180Op.showString(&ceTft180,0,1*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);

        if(showTime == 0)
        {
            showTime++;
            ceDebugOp.sprintf(temp,"acc:%f        ",(fp32)(cePackageRecv.accelerator));
            ceTft180Op.showString(&ceTft180,0,3*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);

            ceDebugOp.sprintf(temp,"Pitch:%f        ",(fp32)(cePackageRecv.pitchByFilter)/1000);
            ceTft180Op.showString(&ceTft180,0,5*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
            ceDebugOp.sprintf(temp,"Roll:%f        ",(fp32)(cePackageRecv.rollByFilter)/1000);
            ceTft180Op.showString(&ceTft180,0,6*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);

        }else if(showTime == 1)
        {
            showTime++;
            ceDebugOp.sprintf(temp,"Yaw:%f        ",(fp32)(cePackageRecv.yawByFilter)/1000);
            ceTft180Op.showString(&ceTft180,0,7*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);

            ceDebugOp.sprintf(temp,"temp:%f        ",(fp32)(cePackageRecv.temperature)/1000);
            ceTft180Op.showString(&ceTft180,0,9*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
            ceDebugOp.sprintf(temp,"alt:%f        ",(fp32)(cePackageRecv.altitude)/1000);
            ceTft180Op.showString(&ceTft180,0,10*8,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        }else  if(showTime == 2)
        {
            showTime = 0;

            ceDebugOp.sprintf(temp,"Vol:%f        ",(fp32)(cePackageRecv.batVoltage)/1000);
            ceTft180Op.showString(&ceTft180,0,12*8,CE_TFT180_COLOR_YELLOW,CE_TFT180_COLOR_BLACK,temp,CE_TFT180_EN_SIZE_F6X8);
        }
    }

}

/**
  * @brief  TMU数据传输管理模块接收数据后，调用的回调函数
  * @param  dataBuf:接收到数据的缓存地址
  * @param  dataCount:接收到数据的长度
  * @return 无
  */
void recvDataCallBack(uint8* dataBuf, uint16 dataCount)
{
    ceLed1COp.setOn(&ceLed1C);                                  //打开接收处理LED指示灯LED1C
    cePackageOp.dealRecv(&cePackageRecv,dataBuf,dataCount);     //对接收数据进行拆包操作
    ceLed1COp.setOff(&ceLed1C);                                 //关闭接收处理LED指示灯LED1C
}

/**
  * @brief  供CREELINKS平台系统调用，用以实现类似CMD窗口的功能
  * @return 无
  */
void appendString(const char* msg)
{
    ceTft180Op.appendString(&ceTft180,msg);
}

/**
  * @brief  S2C按钮的回调函数，用以控制TFT180显示屏的开关
  * @return 无
  */
void btnS2cCallBack()
{
    if(ceTft180Op.getShowStatus(&ceTft180) == 0x01)
        ceTft180Op.setOff(&ceTft180);   //关闭TFT180显示
    else 
        ceTft180Op.setOn(&ceTft180);    //开启TFT180显示
}

/**
  * @brief  初始化各功能模块
  * @return 无
  */
void initialModule(void)
{
    ceTft180Op.initial(&ceTft180,Spi2,PB9GI,PB10GIP,PB11GIP);       //使用一路Spi，三路GPIO资源初始化TFT180显示模块

    ceLed1COp.initialByGpio(&ceLed1A,PC10GI);                       //使用一路GPIO资源来初始LED1A
    ceLed1COp.initialByGpio(&ceLed1B,PC11GI);                       //使用一路GPIO资源来初始LED1B
    ceLed1COp.initialByGpio(&ceLed1C,PC12GI);                       //使用一路GPIO资源来初始LED1C
    ceLed1COp.initialByGpio(&ceLed1D,PC13GI);                       //使用一路GPIO资源来初始LED1D

    ceTft180Op.showString(&ceTft180,28,56,0x0b0f,CE_TFT180_COLOR_BLACK,"CREELINKS",CE_TFT180_EN_SIZE_F8X16);//显示2s CREELINKS平台的大LOGO
    ceSystemOp.delayMs(2000); 

    ceDebugOp.registerAppendString(appendString);                   //向CREELINKS系统注册一个用于实现CMD显示功能的函数

    ceDebugOp.printf("Initial LED...\n");    
    ceLed1COp.setFlash(&ceLed1A,80,500);                            //配置LED1A灯为闪烁状态

    ceDebugOp.printf("Initial Beep...\n");
    ceBeepNSrcOp.initialByGpio(&ceBeepNSrc,PA1AGIP);                //使用一路Gpio资源口初始化无源蜂鸣器

    ceDebugOp.printf("Initial PC33V...\n");    
    cePC33VOp.initial(&cePC33V,PA0ACGIP);                           //使用一路Ad资源口，初始化PC33V电压测量模块

    ceDebugOp.printf("Initial Joystick...\n");
    ceJoystickOp.initial(&ceJoystickLeft,PC0AGI,PC1AGI,PC2AGI);     //使用两路Ad和一路Gpio初始化左侧摇杆模块
    ceJoystickOp.initial(&ceJoystickRight,PC3AGI,PC4AGI,PC5AGI);    //使用两路Ad和一路Gpio初始化右侧摇杆模块

    ceDebugOp.printf("Initial Btn...\n");
    ceBtnx1Op.initialByGpio(&ceBtnS2A,PC6GIP,CE_NULL);              //使用一路Gpio口初始化单个按钮模块S2A，不用注册回调函数
    ceBtnx1Op.initialByGpio(&ceBtnS2B,PC7GIP,CE_NULL);              //使用一路Gpio口初始化单个按钮模块S2B，不用注册回调函数
    ceBtnx1Op.initialByGpio(&ceBtnS2C,PC8GIP,btnS2cCallBack);       //使用一路Gpio口初始化单个按钮模块S2C，注册回调函数以控制TFT屏的显示
    ceBtnx1Op.initialByGpio(&ceBtnS2D,PC9GIP,CE_NULL);              //使用一路Gpio口初始化单个按钮模块S2D，不用注册回调函数

    if(ceBtnx1Op.getStatus(&ceBtnS2D) == 0x01)                      //系统上电时，按下S2D按钮，则进入WIFI通讯模式
    {
        ceDebugOp.printf("Initial TMU By Wifi...\n");

        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,1);                    //响1声
        ceTMUOp.initialByWifi(recvDataCallBack);                    //对TMU数据传输模块进行初始化，耗时较长
    }
    else if(ceBtnx1Op.getStatus(&ceBtnS2C) == 0x01)                 //系统上电时，按下S2C按钮，则进入蓝牙通讯模式
    {
        ceDebugOp.printf("Initial TMU By BLUE...\n");
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,2);                    //响2声
        ceTMUOp.initialByBlue(recvDataCallBack);      
    }else if(ceBtnx1Op.getStatus(&ceBtnS2A) == 0x01)                //进入出厂测试状态
    {
        ceDebugOp.printf("IN TEST WORK TYPE...\n");
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,5);                    //响5声
        workModel = CE_UAV_CTL_TEST;                                //进入测试状态
    }else                                                           //其它情况则进入NRF24L01通讯方式            
    {
        ceDebugOp.printf("Initial TMU By NRF24L01+...\n");
        ceBeepNSrcOp.say(&ceBeepNSrc,100,200,3);
        ceTMUOp.initialByNrf(recvDataCallBack,sendStatusToCtl);               
    }

    ceDebugOp.printf("Initial Package...\n");        
    cePackageOp.initialSend(&cePackageSend);                        //对数据打包发送模块进行初始化
    cePackageOp.initialRecv(&cePackageRecv);                        //对数据接收拆包模块进行初始化

    ceLed1COp.setFlash(&ceLed1A,30,1000);                           //配置LED1A灯为工作正常状态

    ceDebugOp.printf("System Initial Finish...\n");    
    ceBeepNSrcOp.say(&ceBeepNSrc,1000,0,1);                         //蜂鸣器响1s以提示系统初始化完成

    ceDebugOp.unRegisterAppendString();                             //取消注册系统的CMD功能函数，此后可正常使用TFT显示屏
    ceTft180Op.fill(&ceTft180,CE_TFT180_COLOR_BLACK);               //清屏为BLACK
}

/**
  * @brief  CREELINKS平台主入口函数(裸奔)
  * @return 0
  */
int main()
{
    ceSystemOp.initial();                                           //Creelinks环境初始化
    ceDebugOp.initial(Uart4);                            //通过Uart串口输出Debug信息到上位机
    initialModule();                                                //初始化各功能模块
    while (1)
    {
        ceTaskOp.mainTask();                                        //Creelinks环境主循环任务，请保证此函数能够被周期调用
        showStatus();                                               //将无人机状态信息显示到TFT屏上
    }
}



