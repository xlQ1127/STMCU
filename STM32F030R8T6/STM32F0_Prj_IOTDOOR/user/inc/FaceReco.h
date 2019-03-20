#ifndef FACERECO_H
#define FACERECO_H


#include "common.h"


#define RecvBuffSize   40

void RecvFaceRecoData(uint8 ch);
void FaceReco_Init(void);
void FaceCMDSemd(void);
int  FaceDataCheck(void);
#endif