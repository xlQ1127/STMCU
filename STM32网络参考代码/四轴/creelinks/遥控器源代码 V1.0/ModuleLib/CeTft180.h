/**
  ******************************************************************************
  * @file    CeTft180.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeTft180模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_TFT180_H__
#define __CE_TFT180_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_TFT180_VERSION__ 1                                         /*!< 此驱动文件的版本号*/
#define __CE_TFT180_NEED_CREELINKS_VERSION__ 1                          /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_TFT180_NEED_CREELINKS_VERSION__)   /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeTft180.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else

//#define CE_TFT180_SHOW_HORIZONTAL         /*!< 如果需要横向显示，则定义此行 注：V1.0版本暂仅支持竖屏*/

#ifdef CE_TFT180_SHOW_HORIZONTAL
#define CE_TFT180_WIDTH     160
#define CE_TFT180_HIGHT     128
#else
#define CE_TFT180_WIDTH     128
#define CE_TFT180_HIGHT     160
#endif

/**
  * @brief  枚举，Tft180对象常用显示的颜色列表
  */
typedef enum
{
    CE_TFT180_COLOR_RED     = 0xF800,           /*!< 红色*/
    CE_TFT180_COLOR_GREEN   = 0x07E0,           /*!< 绿色*/
    CE_TFT180_COLOR_BLUE    = 0x001F,           /*!< 蓝色*/
    CE_TFT180_COLOR_WHITE   = 0xFFFF,           /*!< 白色*/
    CE_TFT180_COLOR_BLACK   = 0x0000,           /*!< 黑色*/
    CE_TFT180_COLOR_YELLOW  = 0xFFE0,           /*!< 黄色*/
    CE_TFT180_COLOR_GRAY0   = 0xEF7D,           /*!< 重灰色*/
    CE_TFT180_COLOR_GRAY1   = 0x8410,           /*!< 灰色*/
    CE_TFT180_COLOR_GRAY2   = 0x4208,           /*!< 浅灰色*/
} CE_TFT180_COLOR_LIST;
/**
  * @brief  枚举，Tft180对象显示英文字符的字模大小
  */
typedef enum
{
    CE_TFT180_EN_SIZE_F6X8,                     /*!< 字符宽占6个像素点，高占8个像素点*/
    CE_TFT180_EN_SIZE_F8X16,                    /*!< 字符宽占8个像素点，高占16个像素点*/
} CE_TFT180_EN_SIZE;

/*
 *CeTft180属性对像
 */
typedef struct
{
    CeSpiMaster ceSpi;                          /*!< 模块使用到的SpiMaster对象*/
    CeGpio      ceGpio0;
    CeGpio      ceGpio1;
    CeGpio      ceGpio2;
    char        asBuf[CE_TFT180_WIDTH / 6][CE_TFT180_HIGHT / 8];/*!< 用于打印调试信息的缓存*/
    uint16      asXIndex;                       /*!< 用于打印调试信息的光标x轴*/
    uint16      asYIndex;                       /*!< 用于打印调试信息的光标y轴*/
    uint8       isShow;
} CeTft180;
/*
 *CeTft180操作对像
 */
typedef struct
{
    CE_STATUS   (*initial)(CeTft180* ceTft180, CE_RESOURCE ceSpi, CE_RESOURCE ceGpio0, CE_RESOURCE ceGpio1,CE_RESOURCE ceGpio2);/*!<
                                                     @brief CeTft180模块初始化
                                                     @param ceTft180:CeTft180属性对象指针
                                                     @param ceSpi:CeTft180模块使用的Spi资源号
                                                     @param ceTg:CeTft180模块使用的Tg资源号*/

    void        (*setOn)(CeTft180* ceTft180);   /*!< @brief  打开显示
                                                     @param  ceTft180:CeTft180属性对象*/


    void        (*fill)(CeTft180* ceTft180, uint16 color);/*!<
                                                     @brief  CeTft180用指定数据(颜色)进行全屏填充
                                                     @param  ceTft180:CeTft180属性对象
                                                     @param  color:全屏填充的数据(颜色)*/

    void        (*drawPoint)(CeTft180* ceTft180, int16 x, int16 y, uint16 color);/*!<
                                                     @brief  CeTft180在指定位置画点
                                                     @param  ceTft180:CeTft180属性对象
                                                     @param  x:设置开始显示字符串的x轴坐标
                                                     @param  y:设置开始显示字符串的y轴坐标
                                                     @param  color:要显示的数据(颜色)*/

    void        (*drawRectangle)(CeTft180* ceTft180, uint16 startX, uint16 startY, uint16 endX, uint16 endY, int16 color);/*!<
                                                     @brief  CeTft180绘制矩形
                                                     @param  ceTft180:CeTft180属性对象
                                                     @param  startX:矩形左上角x坐标
                                                     @param  startY:矩形左上角y坐标
                                                     @param  endX:矩形右下角x坐标
                                                     @param  endY:矩形右下角y坐标
                                                     @param  color:要显示的数据(颜色)*/

    void        (*drawData)(CeTft180* ceTft180, uint16 x, uint16 y, const uint8* colorBuf, uint16 bufSizeWidth, uint16 bufSizeHight);/*!<
                                                     @brief  CeTft180绘制图片
                                                     @param  ceTft180:CeTft180属性对象
                                                     @param  x:图片的起始点x坐标
                                                     @param  y:图片的起始点y坐标
                                                     @param colorBuf:图片数据
                                                     @param bufSizeWidth:图片宽
                                                     @param bufSizeHight:图片高*/

    void        (*showInt)(CeTft180* ceTft180, int16 x, int16 y, uint16 foreColor, uint16 backColor, const int32 val, CE_TFT180_EN_SIZE showSize);/*!<
                                                     @brief  CeTft180显示32位有符号的数字，最大值0x7FFFFFFF，最小值0x80000001
                                                     @param  ceTft180:CeTft180属性对象
                                                     @param  x:设置开始显示32位有符号的数字的x轴坐标
                                                     @param  y:设置开始显示32位有符号的数字的y轴坐标
                                                     @param  foreColor:显示字体的前景色
                                                     @param  backColor:显示字体的背景色
                                                     @param  val:要显示的32位有符号的数字
                                                     @param  showSize:显示的字体大小，可选CE_LCD_EN_SIZE_F6X8或CE_LCD_EN_SIZE_F8X16*/


    void        (*showString)(CeTft180* ceTft180, int16 x, int16 y, uint16 foreColor, uint16 backColor, const char* msg, CE_TFT180_EN_SIZE showSize);/*!<
                                                     @brief  CeTft180显示字符串，不支持中文
                                                     @param  ceTft180:CeTft180属性对象
                                                     @param  x:设置开始显示字符串的x轴坐标
                                                     @param  y:设置开始显示字符串的y轴坐标
                                                     @param  foreColor:显示字体的前景色
                                                     @param  backColor:显示字体的背景色
                                                     @param  msg:要显示的字符串指针
                                                     @param  showSize:显示的字体大小，可选CE_LCD_EN_SIZE_F6X8或CE_LCD_EN_SIZE_F8X16*/

    void        (*showCN1616)(CeTft180* ceTft180, int16 x, int16 y, uint16 foreColor, uint16 backColor, const uint8* cn1616);/*!<
                                                     @brief  CeTft180显示16x16的中文字体，字模cn1616可从CeCN1616.h中获取
                                                     @param  ceTft180:CeTft180属性对象
                                                     @param  x:设置开始显示字符串的x轴坐标
                                                     @param  y:设置开始显示字符串的y轴坐标
                                                     @param  foreColor:显示字体的前景色
                                                     @param  backColor:显示字体的背景色
                                                     @param  cn1616:要显示的文字模，字模cn1616可从CeCN1616.h中获取*/

    void        (*appendString)(CeTft180* ceTft180, const char* msg);/*!<
                                                     @brief  一般用于代码调试，以控制台的方式打印调试信息
                                                     @param  ceTft180:CeTft180属性对象
                                                     @param  msg:要显示的信息*/

    void        (*setOff)(CeTft180* ceTft180);  /*!< @brief  关闭显示
                                                     @param  ceTft180:CeTft180属性对象*/

    uint8       (*getShowStatus)(CeTft180* ceTft180);
} CeTft180OpBase;
/*
 *CeTft180操作对象实例
 */
extern const CeTft180OpBase ceTft180Op;

#endif //(__CE_CREELINKS_VERSION__ < __CE_TFT180_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TFT180_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function 在屏上显示R、G、B三色，并显示中文与英文
******************************************************************************
#include "Creelinks.h"
#include "CeTft180.h"

const unsigned char CN1616_B4B4[] = //创
{ 0x0C, 0x06, 0x0C, 0x06, 0x1E, 0x06, 0x1B, 0x36, 0x31, 0xB6, 0x60, 0xF6, 0xFF, 0x36, 0x33, 0x36, 0x33, 0x36, 0x33, 0x36, 0x3F, 0x36, 0x36, 0x36, 0x30, 0xC6, 0x30, 0xC6, 0x1F, 0xDE, 0x00, 0x0C };
const unsigned char CN1616_A3E5[] = //ｅ
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x06, 0x60, 0x0C, 0x30, 0x0F, 0xF0, 0x0C, 0x00, 0x0C, 0x00, 0x06, 0x30, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char CN1616_C1AA[] = //联
{0x00, 0xCC, 0xFE, 0x6C, 0x6C, 0x78, 0x6C, 0x00, 0x7D, 0xFE, 0x6C, 0x30, 0x6C, 0x30, 0x7C, 0x30, 0x6F, 0xFF, 0x6C, 0x30, 0x6E, 0x78, 0x7C, 0x78, 0xEC, 0xCC, 0x0C, 0xCC, 0x0D, 0x86, 0x0F, 0x03};

const unsigned char CN1616_B1B1[] = //北
{0x06, 0x60, 0x06, 0x60, 0x06, 0x60, 0x06, 0x66, 0x06, 0x6C, 0x7E, 0x78, 0x06, 0x70, 0x06, 0x60, 0x06, 0x60, 0x06, 0x60, 0x06, 0x60, 0x06, 0x63, 0x1E, 0x63, 0xF6, 0x63, 0x66, 0x3F, 0x06, 0x00};
const unsigned char CN1616_BEA9[] = //京
{0x03, 0x00, 0x01, 0x80, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xF8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1F, 0xF8, 0x01, 0x80, 0x19, 0x98, 0x19, 0x8C, 0x31, 0x86, 0x67, 0x86, 0x03, 0x00};
const unsigned char CN1616_B4F3[] = //大
{0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0xFF, 0xFF, 0x01, 0x80, 0x01, 0x80, 0x03, 0xC0, 0x03, 0xC0, 0x06, 0x60, 0x06, 0x60, 0x0C, 0x30, 0x18, 0x18, 0x30, 0x0C, 0xE0, 0x07};
const unsigned char CN1616_D0C5[] = //信
{0x0C, 0x60, 0x0C, 0x30, 0x0F, 0xFF, 0x18, 0x00, 0x18, 0x00, 0x39, 0xFE, 0x38, 0x00, 0x78, 0x00, 0xD9, 0xFE, 0x18, 0x00, 0x18, 0x00, 0x19, 0xFE, 0x19, 0x86, 0x19, 0x86, 0x19, 0xFE, 0x19, 0x86};
const unsigned char CN1616_BFC6[] = //科
{0x0C, 0x18, 0x1F, 0x98, 0xF8, 0xD8, 0x18, 0xD8, 0x18, 0x18, 0xFF, 0x98, 0x18, 0xD8, 0x3C, 0xD8, 0x3E, 0x18, 0x78, 0x1F, 0x7B, 0xF8, 0xD8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
const unsigned char CN1616_BCBC[] = //技
{0x18, 0x30, 0x18, 0x30, 0x18, 0x30, 0x1B, 0xFF, 0xFE, 0x30, 0x18, 0x30, 0x18, 0x30, 0x1F, 0xFE, 0x1C, 0xC6, 0x38, 0xCC, 0xF8, 0x6C, 0x18, 0x78, 0x18, 0x30, 0x18, 0x78, 0x79, 0xCC, 0x37, 0x07};

CeTft180 myTft180;
int main(void)
{
    ceSystemOp.initial();                                   //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                              //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    ceTft180Op.initial(&myTft180, R12Spi, R2TI2c);
    ceTft180Op.setOn(&myTft180);
    while (1)
    {
        ceTaskOp.mainTask();                                //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操
        ceTft180Op.fill(&myTft180,CE_TFT180_COLOR_RED);
        ceSystemOp.delayMs(1000);
        ceTft180Op.fill(&myTft180,CE_TFT180_COLOR_GREEN);
        ceSystemOp.delayMs(1000);
        ceTft180Op.fill(&myTft180,CE_TFT180_COLOR_BLUE);
        ceSystemOp.delayMs(1000);
        ceTft180Op.fill(&myTft180,CE_TFT180_COLOR_BLACK);

        ceTft180Op.showCN1616(&myTft180, 40, 40,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_B4B4);
        ceTft180Op.showCN1616(&myTft180, 40 + 16, 40,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_A3E5);
        ceTft180Op.showCN1616(&myTft180, 40 + 32, 40, CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,CN1616_C1AA);

        ceTft180Op.showString(&myTft180, 28, 56, CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,"CREELINKS", CE_TFT180_EN_SIZE_F8X16);

        ceTft180Op.showCN1616(&myTft180, 16*1, 72, CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,CN1616_B1B1);
        ceTft180Op.showCN1616(&myTft180, 16*2, 72, CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK,CN1616_BEA9);
        ceTft180Op.showCN1616(&myTft180, 16*3, 72,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_B4F3);
        ceTft180Op.showCN1616(&myTft180, 16*4, 72,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_D0C5);
        ceTft180Op.showCN1616(&myTft180, 16*5, 72,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_BFC6);
        ceTft180Op.showCN1616(&myTft180, 16*6, 72,CE_TFT180_COLOR_WHITE,CE_TFT180_COLOR_BLACK, CN1616_BCBC);

        ceSystemOp.delayMs(2000);
    };
}
******************************************************************************
*/
