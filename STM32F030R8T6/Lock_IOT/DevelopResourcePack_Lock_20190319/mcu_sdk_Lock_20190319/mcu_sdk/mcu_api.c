/****************************************Copyright (c)*************************
**                               版权所有 (C), 2015-2020, 涂鸦科技
**
**                                 http://www.tuya.com
**
**--------------文件信息-------------------------------------------------------
**文   件   名: mcu_api.c
**描        述: 下发/上报数据处理函数
**使 用 说 明 : 此文件下函数无须用户修改,用户需要调用的文件都在该文件内
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
#define MCU_API_GLOBAL

#include "wifi.h"

/*****************************************************************************
函数名称 : hex_to_bcd
功能描述 : hex转bcd
输入参数 : Value_H:高字节/Value_L:低字节
返回参数 : bcd_value:转换完成后数据
*****************************************************************************/
unsigned char hex_to_bcd(unsigned char Value_H,unsigned char Value_L)
{
  unsigned char bcd_value;
  
  if((Value_H >= '0') && (Value_H <= '9'))
    Value_H -= '0';
  else if((Value_H >= 'A') && (Value_H <= 'F'))
    Value_H = Value_H - 'A' + 10;
  else if((Value_H >= 'a') && (Value_H <= 'f'))
    Value_H = Value_H - 'a' + 10;
   
  bcd_value = Value_H & 0x0f;
  
  bcd_value <<= 4;
  if((Value_L >= '0') && (Value_L <= '9'))
    Value_L -= '0';
  else if((Value_L >= 'A') && (Value_L <= 'F'))
    Value_L = Value_L - 'a' + 10;
  else if((Value_L >= 'a') && (Value_L <= 'f'))
    Value_L = Value_L - 'a' + 10;
  
  bcd_value |= Value_L & 0x0f;

  return bcd_value;
}
/*****************************************************************************
函数名称 : my_strlen
功能描述 : 求字符串长度
输入参数 : src:源地址
返回参数 : len:数据长度
*****************************************************************************/
unsigned long my_strlen(unsigned char *str)  
{
  unsigned long len = 0;
  if(str == NULL)
  { 
    return 0;
  }
  
  for(len = 0; *str ++ != '\0'; )
  {
    len ++;
  }
  
  return len;
}
/*****************************************************************************
函数名称 : my_memset
功能描述 : 把src所指内存区域的前count个字节设置成字符c
输入参数 : src:源地址
           ch:设置字符
           count:设置数据长度
返回参数 : src:数据处理完后的源地址
*****************************************************************************/
void *my_memset(void *src,unsigned char ch,unsigned short count)
{
  unsigned char *tmp = (unsigned char *)src;
  
  if(src == NULL)
  {
    return NULL;
  }
  
  while(count --)
  {
    *tmp ++ = ch;
  }
  
  return src;
}
/*****************************************************************************
函数名称 : mymemcpy
功能描述 : 内存拷贝
输入参数 : dest:目标地址
           src:源地址
           count:数据拷贝数量
返回参数 : src:数据处理完后的源地址
*****************************************************************************/
void *my_memcpy(void *dest, const void *src, unsigned short count)  
{  
  unsigned char *pdest = (unsigned char *)dest;  
  const unsigned char *psrc  = (const unsigned char *)src;  
  unsigned short i;
  
  if(dest == NULL || src == NULL)
  { 
    return NULL;
  }
  
  if((pdest <= psrc) || (pdest > psrc + count))
  {  
    for(i = 0; i < count; i ++)
    {  
      pdest[i] = psrc[i];  
    }  
  }
  else
  {
    for(i = count; i > 0; i --)
    {  
      pdest[i - 1] = psrc[i - 1];  
    }  
  }  
  
  return dest;  
}
/*****************************************************************************
函数名称 : my_strcpy
功能描述 : 内存拷贝
输入参数 : s1:目标地址
           s2:源地址
返回参数 : 数据处理完后的源地址
*****************************************************************************/
char *my_strcpy(char *dest, const char *src)  
{
  char *p = dest;
  while(*src!='\0')
  {
    *dest++ = *src++;
  }
  *dest = '\0';
  return p;
}
/*****************************************************************************
函数名称 : my_strcmp
功能描述 : 内存拷贝
输入参数 : s1:字符串1
           s2:字符串2
返回参数 : 大小比较值，0:s1=s2; -1:s1<s2; 1:s1>s2
*****************************************************************************/
int my_strcmp(char *s1 , char *s2)
{
  while( *s1 && *s2 && *s1 == *s2 )
  {
    s1++;
    s2++;
  }
  return *s1 - *s2;
}
/*****************************************************************************
函数名称 : int_to_byte
功能描述 : 将int类型拆分四个字节
输入参数 : number:4字节原数据;value:处理完成后4字节数据
返回参数 :无
****************************************************************************/
void int_to_byte(unsigned long number,unsigned char value[4])
{
  value[0] = number >> 24;
  value[1] = number >> 16;
  value[2] = number >> 8;
  value[3] = number & 0xff;
}
/*****************************************************************************
函数名称 : byte_to_int
功能描述 : 将4字节合并为1个32bit变量
输入参数 : value:4字节数组
返回参数 : number:合并完成后的32bit变量
****************************************************************************/
unsigned long byte_to_int(const unsigned char value[4])
{
  unsigned long nubmer = 0;

  nubmer = (unsigned long)value[0];
  nubmer <<= 8;
  nubmer |= (unsigned long)value[1];
  nubmer <<= 8;
  nubmer |= (unsigned long)value[2];
  nubmer <<= 8;
  nubmer |= (unsigned long)value[3];
  
  return nubmer;
}
#ifndef WIFI_CONTROL_SELF_MODE
/*****************************************************************************
函数名称 : mcu_get_reset_wifi_flag
功能描述 : MCU获取复位wifi成功标志
输入参数 : 无
返回参数 : 复位标志:RESET_WIFI_ERROR:失败/RESET_WIFI_SUCCESS:成功
使用说明 : 1:MCU主动调用mcu_reset_wifi()后调用该函数获取复位状态
           2:如果为模块自处理模式,MCU无须调用该函数
*****************************************************************************/
unsigned char mcu_get_reset_wifi_flag(void)
{
  return reset_wifi_flag;
}
/*****************************************************************************
函数名称 : reset_wifi
功能描述 : MCU主动重置wifi工作模式
输入参数 : 无
返回参数 : 无
使用说明 : 1:MCU主动调用,通过mcu_get_reset_wifi_flag()函数获取重置wifi是否成功
           2:如果为模块自处理模式,MCU无须调用该函数
*****************************************************************************/
void mcu_reset_wifi(void)
{
  reset_wifi_flag = RESET_WIFI_ERROR;
  
  wifi_uart_write_frame(WIFI_RESET_CMD, 0);
}
/*****************************************************************************
函数名称 : mcu_get_wifimode_flag
功能描述 : 获取设置wifi状态成功标志
输入参数 : 无
返回参数 : SET_WIFICONFIG_ERROR:失败/SET_WIFICONFIG_SUCCESS:成功
使用说明 : 1:MCU主动调用mcu_set_wifi_mode()后调用该函数获取复位状态
           2:如果为模块自处理模式,MCU无须调用该函数
*****************************************************************************/
unsigned char mcu_get_wifimode_flag(void)
{
  return set_wifimode_flag;
}
/*****************************************************************************
函数名称 : mcu_set_wifi_mode
功能描述 : MCU设置wifi工作模式
输入参数 : mode:
          SMART_CONFIG:进入smartconfig模式
          AP_CONFIG:进入AP模式
返回参数 : 无
使用说明 : 1:MCU主动调用
           2:成功后,可判断set_wifi_config_state是否为TRUE;TRUE表示为设置wifi工作模式成功
           3:如果为模块自处理模式,MCU无须调用该函数
*****************************************************************************/
void mcu_set_wifi_mode(unsigned char mode)
{
  unsigned char length = 0;
  
  set_wifimode_flag = SET_WIFICONFIG_ERROR;
  
  length = set_wifi_uart_byte(length, mode);
  
  wifi_uart_write_frame(WIFI_MODE_CMD, length);
}
/*****************************************************************************
函数名称 : mcu_get_wifi_work_state
功能描述 : MCU主动获取当前wifi工作状态
输入参数 : 无
返回参数 : WIFI_WORK_SATE_E:
          SMART_CONFIG_STATE:smartconfig配置状态
          AP_STATE:AP 配置状态
          WIFI_NOT_CONNECTED:WIFI 配置成功但未连上路由器
          WIFI_CONNECTED:WIFI 配置成功且连上路由器
          WIFI_CONN_CLOUD:WIFI 已经连接上云服务器
使用说明 : 无
*****************************************************************************/
unsigned char mcu_get_wifi_work_state(void)
{
  return wifi_work_state;
}
#endif

#ifdef SUPPORT_MCU_RTC_CHECK
/*****************************************************************************
函数名称  : mcu_get_system_time
功能描述 : MCU获取系统时间,用于校对本地时钟
输入参数 : 无
返回参数 : 无
使用说明 : MCU主动调用完成后在mcu_write_rtctime函数内校对rtc时钟
*****************************************************************************/
void mcu_get_system_time(void)
{
  wifi_uart_write_frame(GET_LOCAL_TIME_CMD,0);
}
#endif

#ifdef WIFI_TEST_ENABLE


void mcu_start_wifitest(void)
{
  wifi_uart_write_frame(WIFI_TEST_CMD,0);
}
#endif

/*****************************************************************************
函数名称 : wifi_update_request
功能描述 : MCU请求wifi固件升级
输入参数 : 无
返回参数 : 无
使用说明 : MCU主动调用完成后在 wifi_update_handle 函数内可获取升级当前状态
*****************************************************************************/
void wifi_update_request(void)
{ 
  wifi_uart_write_frame(WIFI_UG_REQ_CMD, 0);
}

/*****************************************************************************
函数名称 : mcu_dp_raw_update
功能描述 : raw型dp数据上传
输入参数 : dpid:id号
           value:当前dp值指针
           len:数据长度
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_raw_update(unsigned char dpid,const unsigned char value[],unsigned short len)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  //
  length = set_wifi_uart_byte(length,dpid);
  length = set_wifi_uart_byte(length,DP_TYPE_RAW);
  //
  length = set_wifi_uart_byte(length,len / 0x100);
  length = set_wifi_uart_byte(length,len % 0x100);
  //
  length = set_wifi_uart_buffer(length,(unsigned char *)value,len);
  
  wifi_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}
/*****************************************************************************
函数名称 : mcu_dp_bool_update
功能描述 : bool型dp数据上传
输入参数 : dpid:id号
           value:当前dp值
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_bool_update(unsigned char dpid,unsigned char value)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  length = set_wifi_uart_byte(length,dpid);
  length = set_wifi_uart_byte(length,DP_TYPE_BOOL);
  //
  length = set_wifi_uart_byte(length,0);
  length = set_wifi_uart_byte(length,1);
  //
  if(value == FALSE)
  {
    length = set_wifi_uart_byte(length,FALSE);
  }
  else
  {
    length = set_wifi_uart_byte(length,1);
  }
  
  wifi_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}
/*****************************************************************************
函数名称 : mcu_dp_value_update
功能描述 : value型dp数据上传
输入参数 : dpid:id号
           value:当前dp值
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_value_update(unsigned char dpid,unsigned long value)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  length = set_wifi_uart_byte(length,dpid);
  length = set_wifi_uart_byte(length,DP_TYPE_VALUE);
  //
  length = set_wifi_uart_byte(length,0);
  length = set_wifi_uart_byte(length,4);
  //
  length = set_wifi_uart_byte(length,value >> 24);
  length = set_wifi_uart_byte(length,value >> 16);
  length = set_wifi_uart_byte(length,value >> 8);
  length = set_wifi_uart_byte(length,value & 0xff);
  
  wifi_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}
/*****************************************************************************
函数名称 : mcu_dp_string_update
功能描述 : rstring型dp数据上传
输入参数 : dpid:id号
           value:当前dp值指针
           len:数据长度
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_string_update(unsigned char dpid,const unsigned char value[],unsigned short len)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  //
  length = set_wifi_uart_byte(length,dpid);
  length = set_wifi_uart_byte(length,DP_TYPE_STRING);
  //
  length = set_wifi_uart_byte(length,len / 0x100);
  length = set_wifi_uart_byte(length,len % 0x100);
  //
  length = set_wifi_uart_buffer(length,(unsigned char *)value,len);
  
  wifi_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}
/*****************************************************************************
函数名称 : mcu_dp_enum_update
功能描述 : enum型dp数据上传
输入参数 : dpid:id号
           value:当前dp值
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_enum_update(unsigned char dpid,unsigned char value)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  length = set_wifi_uart_byte(length,dpid);
  length = set_wifi_uart_byte(length,DP_TYPE_ENUM);
  //
  length = set_wifi_uart_byte(length,0);
  length = set_wifi_uart_byte(length,1);
  //
  length = set_wifi_uart_byte(length,value);
  
  wifi_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}
/*****************************************************************************
函数名称 : mcu_dp_fault_update
功能描述 : fault型dp数据上传
输入参数 : dpid:id号
           value:当前dp值
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_fault_update(unsigned char dpid,unsigned long value)
{
  unsigned short length = 0;
   
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  length = set_wifi_uart_byte(length,dpid);
  length = set_wifi_uart_byte(length,DP_TYPE_FAULT);
  //
  length = set_wifi_uart_byte(length,0);
  
  if((value | 0xff) == 0xff)
  {
    length = set_wifi_uart_byte(length,1);
    length = set_wifi_uart_byte(length,value);
  }
  else if((value | 0xffff) == 0xffff)
  {
    length = set_wifi_uart_byte(length,2);
    length = set_wifi_uart_byte(length,value >> 8);
    length = set_wifi_uart_byte(length,value & 0xff);
  }
  else
  {
    length = set_wifi_uart_byte(length,4);
    length = set_wifi_uart_byte(length,value >> 24);
    length = set_wifi_uart_byte(length,value >> 16);
    length = set_wifi_uart_byte(length,value >> 8);
    length = set_wifi_uart_byte(length,value & 0xff);
  }    
  
  wifi_uart_write_frame(STATE_UPLOAD_CMD,length);

  return SUCCESS;
}
/*****************************************************************************
函数名称 : mcu_get_dp_download_bool
功能描述 : mcu获取bool型下发dp值
输入参数 : value:dp数据缓冲区地址
           length:dp数据长度
返回参数 : bool:当前dp值
*****************************************************************************/
unsigned char mcu_get_dp_download_bool(const unsigned char value[],unsigned short len)
{
  return(value[0]);
}
/*****************************************************************************
函数名称 : mcu_get_dp_download_enum
功能描述 : mcu获取enum型下发dp值
输入参数 : value:dp数据缓冲区地址
           length:dp数据长度
返回参数 : enum:当前dp值
*****************************************************************************/
unsigned char mcu_get_dp_download_enum(const unsigned char value[],unsigned short len)
{
  return(value[0]);
}
/*****************************************************************************
函数名称 : mcu_get_dp_download_value
功能描述 : mcu获取value型下发dp值
输入参数 : value:dp数据缓冲区地址
           length:dp数据长度
返回参数 : value:当前dp值
*****************************************************************************/
unsigned long mcu_get_dp_download_value(const unsigned char value[],unsigned short len)
{
  return(byte_to_int(value));
}
/*****************************************************************************
函数名称 : uart_receive_input
功能描述 : 收数据处理
输入参数 : value:串口收到字节数据
返回参数 : 无
使用说明 : 在MCU串口接收函数中调用该函数,并将接收到的数据作为参数传入
*****************************************************************************/
void uart_receive_input(unsigned char value)
{
  #error "请在串口接收中断中调用uart_receive_input(value),串口数据由MCU_SDK处理,用户请勿再另行处理,完成后删除该行" 
	
  if((queue_in > queue_out) && ((queue_in - queue_out) >= sizeof(wifi_queue_buf)))
  {
    //数据队列满
  }
  else if((queue_in < queue_out) && ((queue_out  - queue_in) == 0))
  {
    //数据队列满
  }
  else
  {
    //队列不满
    if(queue_in >= (unsigned char *)(wifi_queue_buf + sizeof(wifi_queue_buf)))
    {
      queue_in = (unsigned char *)(wifi_queue_buf);
    }
    
    *queue_in ++ = value;
  }
}
/*****************************************************************************
函数名称  : wifi_uart_service
功能描述  : wifi串口处理服务
输入参数 : 无
返回参数 : 无
使用说明 : 在MCU主函数while循环中调用该函数
*****************************************************************************/
void wifi_uart_service(void)
{
  #error "请直接在main函数的while(1){}中添加wifi_uart_service(),调用该函数不要加任何条件判断,完成后删除该行"
  static unsigned short rx_in = 0;
  unsigned short offset = 0;
  unsigned short rx_value_len = 0;             //数据帧长度
  
  while((rx_in < sizeof(wifi_uart_rx_buf)) && get_queue_total_data() > 0)
  {
    wifi_uart_rx_buf[rx_in ++] = Queue_Read_Byte();
  }
  
  if(rx_in < PROTOCOL_HEAD)
    return;
  
  while((rx_in - offset) >= PROTOCOL_HEAD)
  {
    if(wifi_uart_rx_buf[offset + HEAD_FIRST] != FRAME_FIRST)
    {
      offset ++;
      continue;
    }
    
    if(wifi_uart_rx_buf[offset + HEAD_SECOND] != FRAME_SECOND)
    {
      offset ++;
      continue;
    }  
    
    if(wifi_uart_rx_buf[offset + PROTOCOL_VERSION] != VERSION)
    {
      offset += 2;
      continue;
    }      
    
    rx_value_len = wifi_uart_rx_buf[offset + LENGTH_HIGH] * 0x100 + wifi_uart_rx_buf[offset + LENGTH_LOW] + PROTOCOL_HEAD;
    if(rx_value_len > sizeof(wifi_uart_rx_buf) + PROTOCOL_HEAD)
    {
      offset += 3;
      continue;
    }
    
    if((rx_in - offset) < rx_value_len)
    {
      break;
    }
    
    //数据接收完成
    if(get_check_sum((unsigned char *)wifi_uart_rx_buf + offset,rx_value_len - 1) != wifi_uart_rx_buf[offset + rx_value_len - 1])
    {
      //校验出错
      offset += 3;
      continue;
    }
    
    data_handle(offset);
    offset += rx_value_len;
  }//end while

  rx_in -= offset;
  if(rx_in > 0)
  {
    my_memcpy(wifi_uart_rx_buf,wifi_uart_rx_buf + offset,rx_in);
  }
}
/*****************************************************************************
函数名称 :  wifi_protocol_init
功能描述 : 协议串口初始化函数
输入参数 : 无
返回参数 : 无
使用说明 : 必须在MCU初始化代码中调用该函数
*****************************************************************************/
void wifi_protocol_init(void)
{
  #error " 请在main函数中添加wifi_protocol_init()完成wifi协议初始化,并删除该行"
  queue_in = (unsigned char *)wifi_queue_buf;
  queue_out = (unsigned char *)wifi_queue_buf;
  //
#ifndef WIFI_CONTROL_SELF_MODE
  wifi_work_state = WIFI_SATE_UNKNOW;
#endif
}


