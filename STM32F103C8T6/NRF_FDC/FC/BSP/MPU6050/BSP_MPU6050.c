/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "BSP_MPU6050.h"

/* #define hI2CMPU hi2cx */

float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}



Posture_Typedef Posture;

                 
/**
  * ��������: I2Cͨ�Ŵ���������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: һ����I2Cͨ�ų�ʱʱ���øú���
  */
static void I2C_Error (void)
{
  /* ����ʼ��I2Cͨ������ */
  HAL_I2C_DeInit(&hI2CMPU);

  /* ���³�ʼ��I2Cͨ������*/
  MX_I2C1_Init();
}

/**
  * ��������: ͨ��I2Cд��һ��ֵ��ָ���Ĵ�����
  * �������: Addr��I2C�豸��ַ
  *           Reg��Ŀ��Ĵ���
  *           Value��ֵ
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void I2C_MPU6050_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hI2CMPU, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0xFFFF);
  
  /* ���I2Cͨ��״̬ */
  if(status != HAL_OK)
  {
    /* ����I2Cͨ�Ŵ��������� */
    I2C_Error();
  }
}



/**
  * ��������: ͨ��I2C��ȡһ�μĴ������ݴ�ŵ�ָ���Ļ�������
  * �������: Addr��I2C�豸��ַ
  *           Reg��Ŀ��Ĵ���
  *           RegSize���Ĵ����ߴ�(8λ����16λ)
  *           pBuffer��������ָ��
  *           Length������������
  * �� �� ֵ: HAL_StatusTypeDef���������
  * ˵    ��: ��
  */
HAL_StatusTypeDef I2C_MPU6050_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hI2CMPU, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, 0xFF);
  
  /* ���I2Cͨ��״̬ */
  if(status != HAL_OK)
  {
    /* ����I2Cͨ�Ŵ��������� */
    I2C_Error();
  }        
  return status;
}




/**
  * ��������: д���ݵ�MPU6050�Ĵ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */ 
void MPU6050_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
  I2C_MPU6050_WriteData(MPU6050_SLAVE_ADDRESS,reg_add,reg_dat);
}


HAL_StatusTypeDef I2C_MPU6050_WriteDatas(uint16_t Addr, uint8_t Reg,uint8_t len, uint8_t *Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hI2CMPU, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Value, len, 0xFFFF);
  
  /* ���I2Cͨ��״̬ */
  if(status != HAL_OK)
  {
    /* ����I2Cͨ�Ŵ��������� */
    I2C_Error();
  }
  return status;
}

uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{

   if( I2C_MPU6050_WriteDatas(addr,(uint16_t)reg,len,buf)!=HAL_OK )
   {return 1;}
   else
   return 0;
}

/**
  * ��������: ��MPU6050�Ĵ�����ȡ����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */ 
void MPU6050_ReadData(uint8_t reg_add,unsigned char *Read,uint8_t num)
{
  I2C_MPU6050_ReadBuffer(MPU6050_SLAVE_ADDRESS,reg_add,I2C_MEMADD_SIZE_8BIT,Read,num);
}

uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
  if(I2C_MPU6050_ReadBuffer(addr,reg,I2C_MEMADD_SIZE_8BIT,buf,len)!=HAL_OK)
  {return 1;}
  else 
  {return 0;}
}


/**
  * ��������: ��ʼ��MPU6050оƬ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */ 
void MPU6050_Init(void)
{
        HAL_Delay(200);
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	    //�������״̬
        HAL_Delay(200);
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //�����ǲ����ʣ�1KHz
        HAL_Delay(50);
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	        //��ͨ�˲��������ã���ֹƵ����1K��������5K
        HAL_Delay(50);
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x08);	  //���ü��ٶȴ�����������2Gģʽ�����Լ�
        HAL_Delay(50);
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x08);     //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
        
}

/**
  * ��������: ��ȡMPU6050��ID
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */ 
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
        MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //��������ַ
	if(Re != 0x68)
	{
		return 0;
	}
	else
	{
		return 1;
	}
		
}

/**
  * ��������: ��ȡMPU6050�ļ��ٶ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */ 
void MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * ��������: ��ȡMPU6050�Ľ��ٶ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */ 
void MPU6050ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

/**
  * ��������: ��ȡMPU6050��ԭʼ�¶�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */ 
void MPU6050ReadTemp(short *tempData)
{
	uint8_t buf[2];
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
    *tempData = (buf[0] << 8) | buf[1];
}

/**
  * ��������: ���I2C�豸�Ƿ���׼���ÿ���ͨ��״̬
  * �������: DevAddress��I2C�豸��ַ
  *           Trials�����Բ��Դ���
  * �� �� ֵ: HAL_StatusTypeDef���������
  * ˵    ��: ��
  */
HAL_StatusTypeDef I2C_MPU6050_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(&hI2CMPU, DevAddress, Trials, 0xFFFF));
}



Posture_Typedef IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
#define pi 3.14159265f                           
#define Kp 10.0f                        
#define Ki 0.008f                         
#define halfT 0.04f           
float  q0=1,q1=0,q2=0,q3=0;   
float  exInt=0,eyInt=0,ezInt=0; 

  
  float  norm;
  float  vx, vy, vz;
  float  ex, ey, ez;

  float  q0q0 = q0*q0;
  float  q0q1 = q0*q1;
  float  q0q2 = q0*q2;
 // float  q0q3 = q0*q3;
  
  float  q1q1 = q1*q1;
//  float  q1q2 = q1*q2;
  float  q1q3 = q1*q3;
  
  float  q2q2 = q2*q2;
  float  q2q3 = q2*q3;
  float  q3q3 = q3*q3;
  
  
  
    gx *= 0.017453f;  
    gy *= 0.017453f;
    gz *= 0.017453f;
  

  norm = sqrt(ax*ax + ay*ay + az*az);      
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;
           
  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								 
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  gx = gx + Kp*ex + exInt;					   							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							
					   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  Posture.Pitch=asin(2*(q0*q2-q1*q3 ))* 57.2957795f; 
  Posture.Roll=atan2(2*(q0*q1+q2*q3 ),1-2*(q1*q1+q2*q2) )* 57.2957795f; 
  
  return Posture;
}


