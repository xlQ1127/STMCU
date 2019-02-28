#ifndef _M6311_H_
#define _M6311_H_







#define M6311_PWR_ON		GPIO_SetBits(GPIOA, GPIO_Pin_0)
#define M6311_PWR_OFF		GPIO_ResetBits(GPIOA, GPIO_Pin_0)

#define M6311_RST_ON		GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define M6311_RST_OFF		GPIO_ResetBits(GPIOA, GPIO_Pin_1)








void M6311_Init(void);

_Bool M6311_SendCmd(char *cmd, char *res);


#endif
