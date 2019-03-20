#include "common.h"
#include "FaceReco.h"


static uint8 FaceRecoBuffRecv[RecvBuffSize] = {0};
static uint8 * RecvDataPtr;
static uint8 RecvDataCount = 0;
static uint32 RecvSum = 0;

void RecvFaceRecoData(uint8 ch)
{
	if(RecvDataCount >= RecvBuffSize)
	{
		RecvDataPtr = FaceRecoBuffRecv;
		RecvDataCount = 0;
	}
	*RecvDataPtr++ = ch;
	RecvDataCount++;
        RecvSum++;
}


void FaceReco_Init(void)
{
	RecvDataPtr = FaceRecoBuffRecv;
	RecvDataCount = 0;
}


/////发送1对N比对
void FaceCMDSemd(void)
{
  uint8 cmd[20] = {0x4d, 0x58, 0x00, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x12, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                   0xB7, 0x00};
  uart1_sendbuff(cmd, sizeof(cmd));
}


int FaceDataCheck(void)
{
  uint8 i = 0;
  int Result = -2;
    if(RecvSum < RecvBuffSize)
      return Result;
  Result = -1;
    if(FaceRecoBuffRecv[i] == 0x6D && FaceRecoBuffRecv[i + 1] == 0x78 
       && FaceRecoBuffRecv[i + 8] == 0x12 && FaceRecoBuffRecv[i + 10] == 0x01
       && FaceRecoBuffRecv[i + 16] == 0xEF && FaceRecoBuffRecv[i + 17] == 0xEF)
    {  
      if(FaceRecoBuffRecv[i + 20] == 0x6D && FaceRecoBuffRecv[i + 21] == 0x78 
         && FaceRecoBuffRecv[i + 28] == 0x12 && FaceRecoBuffRecv[i + 30] == 0x01)
      {
        Result = FaceRecoBuffRecv[i + 32];
        RecvSum = 0;
      }
    }
  return Result;
}