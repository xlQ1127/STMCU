#ifndef __DAC_H
#define __DAC_H
#include "sys.h"
//DHR registers offsets 
#define DHR12R1_Offset             ((unsigned int)0x00000008)
#define DHR12R2_Offset             ((unsigned int)0x00000014)
#define DHR12RD_Offset             ((unsigned int)0x00000020)
#define CR_CLEAR_Mask              ((unsigned int)0x00000FFE)
#define SWTRIGR_SWTRIG_Set         ((unsigned int)0x00000001)
#define CR_EN_Set                  ((unsigned int)0x00000001)

void MyDAC_Init(void);//DAC channel1 Configuration
void DAC1_SetData(u16 data);
void DAC2_SetData(u16 data);

#endif


	