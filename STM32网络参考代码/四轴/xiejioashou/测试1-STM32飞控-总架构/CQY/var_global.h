
#include "stm32f10x.h"

typedef struct int16_xyz{
				vs16 X;
				vs16 Y;
				vs16 Z;} int16_XYZ;  //有符号16位xyz

typedef struct float_xyz{
				float X;
				float Y;
				float Z;} float_XYZ; //浮点型xyz


typedef struct float_angle{
				float Rool;
				float Pitch;
				float Yaw;} float_RPY; //浮点型欧拉角

extern int16_XYZ ACC_RealData;
extern int16_XYZ GRY_RealData;
extern u8 TxDate[4];
extern float_XYZ ACC_F,GRY_F;
extern float_RPY Q_ANGLE;

extern int16_XYZ ACC_AVG;


