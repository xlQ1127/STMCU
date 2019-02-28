#ifndef _STMFLASH_H_
#define _STMFLASH_H_







#define SSID_ADDRESS		0x0807F800 //账号保存地址	 							地址必须是2的倍数
#define PSWD_ADDRESS		0x0807F8A0 //密码保存地址	长度偏移160字节(80个半字)	地址必须是2的倍数



_Bool Flash_NeedErase(void);

void Flash_Read(unsigned int addr, char *rBuf, unsigned short len);

void Flash_Write(unsigned int addr, char *wBuf, unsigned short len);


#endif
