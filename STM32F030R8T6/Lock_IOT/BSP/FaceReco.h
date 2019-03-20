#ifndef FACERECO_H
#define FACERECO_H

#include "stm32f0xx_hal.h"
#include "usart.h"

#define RecvBuffSize   40

typedef struct{
uint8_t Open;
int VisitorNum;
}FaceCheck_Typedef;

extern FaceCheck_Typedef FaceCheck;

void RecvFaceRecoData(uint8_t ch);
void FaceReco_Init(void);
void FaceCMDSemd(void);
int  FaceDataCheck(void);
#endif
