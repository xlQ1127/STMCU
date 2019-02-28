#ifndef _ONENET_H_
#define _ONENET_H_





struct MODBUS_LIST
{

	unsigned short *buf;		//ModBus---传感器值缓存表
	unsigned short dataLen;		//ModBus---传感器值的个数
	
	struct MODBUS_LIST *next;	//下一个

};


typedef struct
{

    char serial[12];
    char pswd[12];
	char proID[12];
	
	char ip[16];
	char port[8];
	
	const unsigned char protocol;	//协议类型号		1-edp	2-nwx	3-jtext		4-Hiscmd
									//				5-jt808			6-modbus	7-mqtt
									//				8-gr20			9-reg		10-HTTP(自定义)
	
	unsigned char *cmd_ptr;			//平台下发的命令
	
	unsigned char s_addr;			//ModBus---本机地址
	unsigned char m_cmd;			//ModBus---收到的命令码
	unsigned short r_addr;			//ModBus---寄存器地址
	unsigned short r_len;			//ModBus---寄存器读取长度
	unsigned char rev_cmd_cnt;		//收到的命令个数
/*************************发送队列*************************/
	struct MODBUS_LIST *head, *end;
	
	unsigned char netWork : 1;		//1-OneNET接入成功		0-OneNET接入失败
	unsigned char sendData : 2;
	unsigned char errCount : 3;		//错误计数
	unsigned char heartBeat : 1;	//心跳
	unsigned char reverse : 1;		//保留

} ONETNET_INFO;

extern ONETNET_INFO onenet_info;





#define CHECK_CONNECTED			0	//已连接
#define CHECK_CLOSED			1	//已断开
#define CHECK_GOT_IP			2	//已获取到IP
#define CHECK_NO_DEVICE			3	//无设备
#define CHECK_INITIAL			4	//初始化状态
#define CHECK_NO_CARD			5	//没有sim卡
#define CHECK_NO_ERR			255 //

#define SEND_TYPE_OK			0	//
#define SEND_TYPE_DATA			1	//
#define SEND_TYPE_HEART			2	//




void OneNet_DevLink(const char *serial, const char *pswd, const char *devid);

unsigned char OneNet_SendData(unsigned short *value_table, unsigned short value_table_cnt);

unsigned char OneNet_SendData_Heart(void);

void OneNET_CmdHandle(void);

void OneNet_RevPro(unsigned char *cmd, unsigned short len);

_Bool OneNet_CheckListHead(void);

unsigned short *OneNet_GetListHeadBuf(void);

unsigned short OneNet_GetListHeadLen(void);

unsigned char OneNet_AddDataSendList(unsigned short *buf ,unsigned short dataLen);

_Bool OneNet_DeleteDataSendList(void);

#endif
