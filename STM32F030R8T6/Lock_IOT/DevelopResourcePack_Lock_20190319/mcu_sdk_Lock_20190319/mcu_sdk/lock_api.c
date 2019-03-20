/****************************************Copyright (c)*************************
**                               版权所有 (C), 2015-2020, 涂鸦科技
**
**                                 http://www.tuya.com
**
**--------------文件信息-------------------------------------------------------
**文   件   名: lock_api.c
**描        述: 门锁下发/上报数据处理函数
**使 用 说 明 : 直接在需要的地方调用门锁功能接口，在返回的数据中处理相关逻辑
**
**
**--------------当前版本修订---------------------------------------------------
** 版  本: v1.0.2
** 日　期: 2018年12月6日
** 描　述: 1:增加低功耗门锁api接口文件
           2:修改增加部分新功能兼容

** 版  本: v1.0.1
** 日　期: 2018年11月1日
** 描　述: 1:低功耗SDK上线正式版

** 版  本: v1.0
** 日　期: 2018年4月24日
** 描　述: 1:低功耗SDK首版，适配1.0.2协议
**
**-----------------------------------------------------------------------------
******************************************************************************/
#define LOCK_API_GLOBAL

#include "wifi.h"
#include "lock_api.h"

/*****************************************************************************
函数名称 : mcu_get_gelin_time
功能描述 : MCU获取格林时间,用于校对本地时钟
输入参数 : 无
返回参数 : 无
使用说明 : MCU主动调用完成后在 mcu_write_gltime 函数内记录并计算格林时间，
           用于门锁类时间戳校验
*****************************************************************************/
void mcu_get_gelin_time(void)
{
  wifi_uart_write_frame(GET_GL_TIME_CMD,0);
}

/*****************************************************************************
函数名称 : mcu_get_temp_pass
功能描述 : MCU请求获取云端临时密码
输入参数 : 无
返回参数 : 无
使用说明 : MCU主动调用完成后在 temp_pass_handle 函数内可获取临时密码和有效期
*****************************************************************************/
void mcu_get_temp_pass(void)
{ 
  wifi_uart_write_frame(TEMP_PASS_CMD, 0);
}

/*****************************************************************************
函数名称 : dynamic_pass_check
功能描述 : MCU请求动态密码校验
输入参数 : time:请求当前格林时间(从低到高6位，分别为年月日时分秒)
           user_pass:用户输入的动态密码(从低到高8位，密码用ASCLL传入，范围'0'-'9')
           admin_num:管理员密码组数(0~10，若为0则后面参数无效)
           admin_len:管理员密码长度(最长8个字节)
           admin_pass:多组管理员密码(多组密码依次排列，长度admin_num*admin_len，
                      用ASCLL传入，范围'0'-'9')
返回参数 : 无
使用说明 : 1:MCU主动调用
           2:成功后,在 pass_check_handle 函数内可获取动态密码校验结果
*****************************************************************************/
void dynamic_pass_check(unsigned char time[], unsigned char user_pass[], unsigned char admin_num,
                        unsigned char admin_len, unsigned char admin_pass[])
{ 
  unsigned char length = 0;
  unsigned char i = 0;

  length = set_wifi_uart_buffer(length,(unsigned char *)time, 6);
  length = set_wifi_uart_buffer(length,(unsigned char *)user_pass, 8);
  length = set_wifi_uart_byte(length, admin_num);

  if (admin_num > 0)
  {
    length = set_wifi_uart_byte(length, admin_len);

    for(i=0;i<admin_num;i++)
    {
      length = set_wifi_uart_buffer(length,(unsigned char *)(admin_pass+admin_len*i), admin_len);
    }
  }

  wifi_uart_write_frame(PASS_CHECK_CMD, length);
}

/*****************************************************************************
函数名称 : mcu_get_mul_temp_pass
功能描述 : MCU请求获取云端多组临时密码
输入参数 : 无
返回参数 : 无
使用说明 : MCU主动调用完成后在 mul_temp_pass_data 函数内可获取临时密码参数和有效期时限
*****************************************************************************/
void mcu_get_mul_temp_pass(void)
{ 
  wifi_uart_write_frame(MUL_TEMP_PASS_CMD, 0);
}

/*****************************************************************************
函数名称 : mcu_write_gltime
功能描述 : MCU查询格林时间返回函数
输入参数 : gl_time:返回的当前格林时间(从低到高7位，分别为年月日时分秒星期)

返回参数 : 无
使用说明 : 1:MCU主动调用 mcu_get_gelin_time 成功后,该函数内可获取格林时间
*****************************************************************************/
void mcu_write_gltime(unsigned char gl_time[])
{
  #error "请自行完成格林时间记录代码,并删除该行"
  /*
  gl_time[0]为是否获取时间成功标志，为 0 表示失败，为 1表示成功
  gl_time[1]为年份 , 0x00 表示 2000 年
  gl_time[2]为月份，从 1 开始到12 结束
  gl_time[3]为日期，从 1 开始到31 结束
  gl_time[4]为时钟，从 0 开始到23 结束
  gl_time[5]为分钟，从 0 开始到59 结束
  gl_time[6]为秒钟，从 0 开始到59 结束
  gl_time[7]为星期，从 1 开始到 7 结束，1代表星期一
 */
  if(gl_time[0] == 1)
  {
    //正确接收到wifi模块返回的本地时钟数据 
	 
  }
  else
  {
  	//获取本地时钟数据出错,有可能是当前wifi模块未联网
  }
}

/*****************************************************************************
函数名称 : temp_pass_handle
功能描述 : MCU请求临时密码返回函数
输入参数 : suc_flag:请求标志(1:成功;0:失败)
           gl_time:密码有效期格林时间(从低到高6位，分别为年月日时分秒)
           pass:临时密码数据(ascll码表示，长度pass_len)
返回参数 : 无
使用说明 : 1:MCU主动调用 mcu_get_temp_pass 成功后,该函数内可临时密码和有效期
*****************************************************************************/
void temp_pass_handle(unsigned char suc_flag, const unsigned char gl_time[], 
                      const unsigned char pass[], unsigned char pass_len)
{ 
  #error "请自行完成临时密码相关处理代码,并删除该行"

  /*
  suc_flag为是否获取密码成功标志，为 0 表示失败，为 1 表示成功
 */
  /*
  gl_time为密码有效期格林时间
  gl_time[0]为年份 , 0x00 表 示2000 年
  gl_time[1]为月份，从 1 开始到12 结束
  gl_time[2]为日期，从 1 开始到31 结束
  gl_time[3]为时钟，从 0 开始到23 结束
  gl_time[4]为分钟，从 0 开始到59 结束
  gl_time[5]为秒钟，从 0 开始到59 结束

  pass指向临时密码数据(ascll码)，长度pass_len
 */
  if (suc_flag == 1)
  {
    //获取临时密码数据成功
  }
  else
  {
    //获取临时密码数据出错
  }
}

/*****************************************************************************
函数名称 : pass_check_handle
功能描述 : MCU请求校验动态密码返回函数
输入参数 : status:校验标志(0:成功;1:失败;2:未激活;3:长度错误)
返回参数 : 无
使用说明 : 1:MCU主动调用 dynamic_pass_check 成功后,该函数内可获取校验结果
*****************************************************************************/
void pass_check_handle(unsigned char status)
{ 
  #error "请自行完成动态密码校验结果处理代码,并删除该行"

  switch (status)
  {
    case 0:
    {
      //密码核对通过
      break;
    }

    case 1:
    {
      //密码核对失败
      break;
    }

    case 2:
    {
      //设备未激活
      break;
    }

    case 3:
    {
      //数据长度错误
      break;
    }

    default:
      break;
  }
}

/*****************************************************************************
函数名称 : mul_temp_pass_data
功能描述 : MCU请求临时密码返回函数
输入参数 : suc_flag:请求标志(1:成功;0:失败)
           pass_ser:当前组密码编号(实际编号须+900)
           pass_valcnt:当前组密码有效次数(0:不限次数;1:一次性)
           pass_sta:密码当前状态(0:有效;1:被删除无效)
           gl_start:密码生效日期格林时间(从低到高6位，分别为年月日时分秒)
           gl_end:密码失效日期格林时间(从低到高6位，分别为年月日时分秒)
           pass:临时密码数据(ascll码表示，长度pass_len)
返回参数 : 无
使用说明 : 1:MCU主动调用 mcu_get_mul_temp_pass 成功后,
             该函数内可多次分别获取每组的临时密码与有效期限
*****************************************************************************/
static void mul_temp_pass_data(unsigned char suc_flag, unsigned char pass_ser, 
                               unsigned char pass_valcnt, unsigned char pass_sta, 
                               const unsigned char gl_start[], const unsigned char gl_end[], 
                               const unsigned char pass[], unsigned char pass_len)
{ 
  #error "请自行完成多组临时密码信息处理代码,并删除该行"

  /*
  suc_flag为是否获取密码成功标志，为 0 表示失败，为 1 表示成功
 */
  /*
  pass_ser为密码编号
  pass_valcnt为密码有效次数
  pass_sta为密码当前状态

  gl_start为密码生效日期格林时间
  gl_start[0]为年份 , 0x00 表 示2000 年
  gl_start[1]为月份，从 1 开始到12 结束
  gl_start[2]为日期，从 1 开始到31 结束
  gl_start[3]为时钟，从 0 开始到23 结束
  gl_start[4]为分钟，从 0 开始到59 结束
  gl_start[5]为秒钟，从 0 开始到59 结束
  gl_end为密码截至日期格林时间，同gl_start

  pass指向临时密码数据(ascll码)，长度pass_len
 */

  /*
  注：获取多组密码成功，该函数会进入多次，
      直至将多组临时密码全部获取完结束；
      若失败则只进入一次。
 */
  if (suc_flag == 1)
  {
    //获取多组临时密码数据成功
  }
  else
  {
    //获取多组临时密码数据出错
  }
}

/*****************************************************************************
函数名称 : mul_temp_pass_handle
功能描述 : MCU请求多组临时密码处理函数
输入参数 : data:返回数据
返回参数 : 无
使用说明 : 无
*****************************************************************************/
void mul_temp_pass_handle(const unsigned char data[])
{
  unsigned char i = 0;

  unsigned char suc_flag = data[0];
  unsigned char pass_num = data[1];
  unsigned char pass_len = data[2];
  unsigned char offset = 0;

  if (suc_flag == 1)
  {
    for (i=0;i<pass_num;i++)
    {
      offset = i*(15+pass_len);
      mul_temp_pass_data(suc_flag, data[offset+3], data[offset+4], data[offset+5], 
                         data+offset+6, data+offset+12, data+offset+18, pass_len);
    }
  }
  else
  {
    mul_temp_pass_data(suc_flag, 0, 0, 0, 0, 0, 0, 0);
  }
}

#ifdef SUPPORT_MCU_FIRM_UPDATE
/*****************************************************************************
函数名称 : mcu_update_request
功能描述 : MCU请求mcu固件升级
输入参数 : 无
返回参数 : 无
使用说明 : MCU主动调用完成后在 mcu_update_handle 函数内可获取升级当前状态
*****************************************************************************/
void mcu_update_request(void)
{ 
  wifi_uart_write_frame(MCU_UG_REQ_CMD, 0);
}

/*****************************************************************************
函数名称 : mcu_update_handle
功能描述 : MCU请求mcu固件升级返回函数
输入参数 : status:校验标志
返回参数 : 无
使用说明 : MCU主动调用 mcu_update_request 函数完成后该函数内可获取升级当前状态
*****************************************************************************/
void mcu_update_handle(unsigned char status)
{ 
  #error "请自行完成mcu升级状态处理代码,并删除该行"

  switch (status)
  {
    case 0:
    {
      //开始检查固件更新
      break;
    }

    case 1:
    {
      //已经是最新固件
      break;
    }

    case 2:
    {
      //正在更新固件
      break;
    }

    case 3:
    {
      //固件更新成功
      break;
    }

    case 4:
    {
      //固件更新失败
      break;
    }

    default:
      break;
  }
}
#endif


