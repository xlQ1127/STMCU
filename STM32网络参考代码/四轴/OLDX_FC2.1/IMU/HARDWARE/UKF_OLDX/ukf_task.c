#include "ukf_task.h"
#include "ukf_oldx.h"
#include "my_math.h"
#include "usart_fc.h"
#include "gps.h"
#include "KF_OLDX_NAV.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "insgps.h"
#include "time.h"
#include "nav_ukf.h"
#include "matlib.h"
#define DELAY_GPS 0.1//s



void KF_OLDX4(float *X_f,float *P_f,float *Z_f,float *A_f
	,float *B_f,float *H_f,float U,float *Q_f,float *R_f)
{
    unsigned int S31[2] = {3,1};
    unsigned int S33[2] = {3,3};
    float I[3][3]= {{1,0,0}, {0, 1, 0}, {0, 0, 1}};
		float Temp31_1[3][1];
		float Temp31_2[3][1];
		float Temp31_3[3][1];
		float Temp31_4[3][1];
		
		float Temp33_1[3][3];
		float Temp33_2[3][3];
		float Temp33_3[3][3];
		float Temp33_4[3][3];
		float Temp33_5[3][3];
		float Temp33_6[3][3];
		//x_pre //X_pre=A*X+B*U;
    float X_pre[3][1] ;
	  MatMultiplication( (float *)A_f, S33, (float *)X_f, S33, (float *)Temp31_1);
	  MatScalarMult((float *)B_f, S31, U, (float *)Temp31_2);
    MatAddition( (float *)Temp31_1, S31, (float *)Temp31_2,(float *)X_pre);
		//p_pre //P_pre=A*P*A'+Q;
		float p_pre[3][3] ;
		MatMultiplication( (float *)A_f, S33, (float *)P_f, S33, (float *)Temp33_1);
		MatTranspose( (float *)A_f, S33, (float *)Temp33_2);
		MatMultiplication( (float *)Temp33_1, S33, (float *)Temp33_2, S33, (float *)Temp33_3);
		MatAddition( (float *)Temp33_3, S33, (float *)Q_f,(float *)p_pre);
		//cal K  K=P_pre*H'*inv(H*P_pre*H'+R);
		float k[3][3]; 
		MatMultiplication( (float *)p_pre, S33, (float *)H_f, S33, (float *)Temp33_1);
		MatMultiplication( (float *)H_f, S33, (float *)p_pre, S33, (float *)Temp33_2);
		MatMultiplication( (float *)Temp33_2, S33, (float *)H_f, S33, (float *)Temp33_3);
		MatAddition( (float *)Temp33_3, S33, (float *)R_f,(float *)Temp33_4);
		MatInv3by3((float *)Temp33_4, (float *)Temp33_5);
		MatMultiplication( (float *)Temp33_1, S33, (float *)Temp33_5, S33, (float *)k);
		//correct  X=X_pre+K*(H*Z-H*X_pre);
		MatMultiplication( (float *)H_f, S33, (float *)Z_f, S31, (float *)Temp31_1);
		MatMultiplication( (float *)H_f, S33, (float *)X_pre, S31, (float *)Temp31_2);
		MatSubAddition( (float *)Temp31_1, S33, (float *)Temp31_2,(float *)Temp31_3);
		MatMultiplication( (float *)k, S33, (float *)Temp31_3, S31, (float *)Temp31_4);
		MatAddition( (float *)Temp31_1, S31, (float *)Temp31_4,(float *)X_f);
		//P   //P=(I-K*H)*P_pre;
		MatMultiplication( (float *)k, S33, (float *)H_f, S33, (float *)Temp33_1);
		MatSubAddition( (float *)I, S33, (float *)Temp33_1,(float *)Temp33_2);
		MatMultiplication( (float *)Temp33_2, S33, (float *)p_pre, S33, (float *)P_f);
}
float test_k[3]={0.6,1.6,0.6};
u8 OLDX_KF3(float *measure,float tau,float *r_sensor,u8 *flag_sensor,double *state,double *state_correct,float T)
{
float PosDealt;	
float SpeedDealt;
float K_ACC_Z;
float K_VEL_Z;
float K_POS_Z;

if(!flag_sensor[0]&&!flag_sensor[1]&&!flag_sensor[2])	
	return 0;
K_ACC_Z =(5.0f / (tau * tau * tau));
K_VEL_Z =(3.0f / (tau * tau));
K_POS_Z =(3.0f / tau);
//d spd	
PosDealt=(measure[0]-state[0]);

if(flag_sensor[0]){
state_correct[3*0+0] += r_sensor[0]*PosDealt* K_POS_Z ;//pos correct
state_correct[3*0+1] += r_sensor[1]*PosDealt* K_VEL_Z ;//spd correct
//state_correct[3*0+2] += r_sensor[2]*PosDealt* K_ACC_Z ;//acc correct
}

if(flag_sensor[1])
{
state_correct[3*0+1] += r_sensor[0]*(measure[1]-state[1])*33* K_VEL_Z ;//spd correct
state_correct[3*0+2] += r_sensor[1]*(measure[1]-state[1])*33* K_ACC_Z ;//acc correct
}

//acc correct
//估计
state_correct[3*1+2]=0;
//修正
state[2]=measure[2]*flag_sensor[2]+state_correct[3*0+2];

//vel correct
//估计
state_correct[3*1+1]+=state[2]*T;
//修正
state[1]=state_correct[3*1+1]+state_correct[3*0+1];

//pos correct
//估计
state_correct[3*1+0]+=state[1]*T+0.5*state[2]*T*T;
//修正
state[0]=state_correct[3*1+0]+state_correct[3*0+0];

return 1;	
}


//For  Qr  mark
#define NAV_USE_KF 1
#if USE_FLOW_FLY_ROBOT
float q_flow[3]={0.005,0.005,0.005};//{1,1,1};//0.6;///1;
#else
float q_flow[3]={0.02,0.01,0.01};//{1,1,1};//0.6;///1;
#endif
float r_flow[3]={10,1,0.1};//{1,1,1};//0.6;///1;
float flow_gain=1;//2;
double X_ukf[6],X_ukf_global[6];
double X_ukf_baro[6];
int acc_flag_flow[2]={1,1};
float X_ukf_Pos[2];
float r1,r2;
float posNorth,posEast;
double local_Lat,local_Lon;//GPS局部坐标初始
float velEast,velNorth;
float GPS_J_F,GPS_W_F;//融合GPS
static void CalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

static void CalcGlobalDistance(double lat, double lon) {
    posNorth = (lat - local_Lat) * r1;
    posEast =  (lon - local_Lon) * r2;
}


static void CalcGlobalLocation(float posNorth,float posEast){ 
    GPS_W_F=(float)posNorth/(float)(r1+0.1)+local_Lat;
    GPS_J_F=(float)posEast/(float)(r2+0.1)+local_Lon;
}

static float acc_body_buf[2][40];
static void feed_acc_buf(float in1,float in2)
{
u8 i,j;	
float reg[2][40];	
static u8 cnt;
 for(i=0;i<2;i++)
	for(j=0;j<40;j++)
   reg[i][j]=acc_body_buf[i][j];
	
 for(i=0;i<2;i++)
	for(j=0;j<40-1;j++)
   acc_body_buf[i][j]=reg[i][j-1];	
	
	acc_body_buf[0][0]=in1;
	acc_body_buf[1][0]=in2;
}	

static float get_acc_delay(u8 sel,float delay,float dt)
{
u8 id[2];
id[0]=(int)(delay/dt);
id[1]=id[0]+1;	
if(delay>0)	
return acc_body_buf[sel][id[0]]/2+acc_body_buf[sel][id[1]]/2;
else
return acc_body_buf[sel][0];	
}	

u8 kf_data_sel=1;//0->flow 1->gps 2->flow global 
double X_KF_NAV[2][3],X_KF_NAV_HB[2][3];
double P_KF_NAV[2][9]={0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001};
float ga_nav= 0.1; 
float gwa_nav=0.1;
#if FLOW_USE_P5A
float g_pos_flow= 0.0086;//0.0051;
float g_spd_flow= 0.000368;//0.0006;
#else
float g_pos_flow= 0.0086*1.2;//0.0051;
float g_spd_flow= 0.000025;//0.0006;
#endif
float K_pos_qr=0.01;
float K_spd_flow=0.86;//1.2;//0.86;
//gps

float K_acc_gps=1;  
float K_pos_gps=1;
float K_spd_gps=1;
#if  USE_M100_IMU
float g_pos_gps= 0.001;//10;
float g_spd_gps= 0.001;//0.1;//0.1; 
#else
float g_pos_gps= 0.036;//10;
float g_spd_gps= 0.000052125;//0.1;//0.1; 
#endif
float g_pos_use,g_spd_use;
float velNorth_gps,velEast_gps;
int flag_kf1[2]={1,1};
u8 force_test;
float Posx,Posy;
u8 gps_data_vaild=0,gps_init;
float  r_flow_new[4]={0.03,0.05,0.0153,4.56};
float  r_gps_new[4]={0.03,0.05,0.0153,5};
double state_correct_posx[6];
double state_correct_posy[6];
double X_kf2_x[3];
double X_kf2_y[3];
float X_f[3][1],P_f[3][3];
void ukf_pos_task_qr(float Qr_x,float Qr_y,float Yaw,float flowx,float flowy,float accx,float accy,float T)
{
static int gps_h_off;	

float Sdpx,Accx;
float Sdpy,Accy;

u8 pos_vaild=0;
double A[9]=
			 {1,       0,    0,
				T,       1,    0,
				-T*T/2, -T,    1};

double B[3]={T*T/2,T,0}; 
double H[9]={
			 1,0,0,
       0,1,0,
       0,0,0}; 

#if USE_M100_IMU
gpsx.pvt.PVT_latitude=m100.Lat;
gpsx.pvt.PVT_longitude=m100.Lon;	
if(m100.connect&&m100.m100_data_refresh==1&&m100.Yaw!=0)	
{gpsx.pvt.PVT_numsv=3;gpsx.pvt.PVT_fixtype=3;}
#endif			 
			 
 if((gpsx.pvt.PVT_longitude!=0||force_test) && gps_init==0 && gpsx.pvt.PVT_numsv>=6&&gpsx.pvt.PVT_fixtype>=3){
 gps_init=1;
 local_Lat=gpsx.pvt.PVT_latitude;
 local_Lon=gpsx.pvt.PVT_longitude;
 gps_h_off=gpsx.pvt.PVT_height; 
 CalcEarthRadius(gpsx.pvt.PVT_latitude);

 }
#if USE_M100_IMU //<<----------------------------------------------------------------------
u8 kf_data_sel_temp=1;  
#else 
u8 kf_data_sel_temp=kf_data_sel; 
#endif 
 
if(module.gps&& gpsx.pvt.PVT_numsv>=1&&gpsx.pvt.PVT_fixtype>=3)
kf_data_sel_temp=1;	
else if(module.flow||module.flow_iic)
{kf_data_sel_temp=1;gps_init=0;}	
//kf_data_sel_temp=0;
#if NAV_USE_KF
//--------------------------------GPS_KF------------------------------------- 
if(kf_data_sel_temp==1){
   float ACCY=flow_matlab_data[1]*K_acc_gps;
   float ACCX=flow_matlab_data[0]*K_acc_gps;
	 float accEast=ACCY*sin(Yaw*0.0173)+ACCX*cos(Yaw*0.0173);
   float accNorth=ACCY*cos(Yaw*0.0173)-ACCX*sin(Yaw*0.0173);
	 #if USE_UKF_FROM_AUTOQUAD
	 feed_acc_buf(-accy,-accx);
	 #else
	 feed_acc_buf(accEast,accNorth);
	 #endif
   #if USE_M100_IMU
	 if(m100.connect&&m100.m100_data_refresh==1&&m100.Yaw!=0)
	 {CalcEarthRadius(gpsx.pvt.PVT_latitude); gps_data_vaild=1;}
	 #else
	 if(gpsx.pvt.PVT_numsv>=6&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0)
	 {CalcEarthRadius(gpsx.pvt.PVT_latitude); gps_data_vaild=1;}
	 #endif

	 CalcGlobalDistance(gpsx.pvt.PVT_latitude,gpsx.pvt.PVT_longitude); 
	 
	 static float dposEast,dposNorth;
	 static u8 cnt;
	 if(cnt++>0.2/T){cnt=0;
	 velNorth_gps=(posNorth-dposNorth)/(0.2+0.00001);
	 velEast_gps=(posEast-dposEast)/(0.2+0.00001);
	 dposEast=posEast;
	 dposNorth=posNorth;
	 }
	 
	 #if !USE_M100_IMU
   #if GPS_FROM_UBM
	   velEast=LIMIT(gpsx.ubm.velE,-6.3,6.3);//LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
		 velNorth=LIMIT(gpsx.ubm.velN,-6.3,6.3);//LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3);
		 #else
		 velEast=LIMIT(gpsx.pvt.PVT_East_speed,-6.3,6.3);//LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
		 velNorth=LIMIT(gpsx.pvt.PVT_North_speed,-6.3,6.3);//LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3);
		 #endif
	 #else
	 velEast=LIMIT(m100.spd[1],-3,3);//LIMIT(-gpsx.spd*sin((gpsx.angle-180)*0.0173),-3,3);
   velNorth=LIMIT(m100.spd[0],-3,3);//LIMIT(-gpsx.spd*cos((gpsx.angle-180)*0.0173),-3,3); 
	 #endif
   #if !USE_M100_IMU
	 Global_GPS_Sensor.NED_Pos[Zr]=gpsx.pvt.PVT_height-gps_h_off;
	 #else
   Global_GPS_Sensor.NED_Pos[Zr]=(float)(gpsx.altitude-gps_h_off)/10.;
	 #endif
	 Global_GPS_Sensor.NED_Vel[Zr]=gpsx.pvt.PVT_Down_speed;
   if((module.pi_flow&&pi_flow.insert)&&
		!(gpsx.pvt.PVT_numsv>=6&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0&&((gps_init&&gps_data_vaild)))) 
   ;
	 else{
   Global_GPS_Sensor.NED_Pos[Yr]=Posy=posNorth*K_pos_gps;//1-> west north
	 Global_GPS_Sensor.NED_Vel[Yr]=Sdpy=velNorth*K_spd_gps;
	 Global_GPS_Sensor.NED_Pos[Xr]=Posx=posEast*K_pos_gps;//0->  east
	 Global_GPS_Sensor.NED_Vel[Xr]=Sdpx=velEast*K_spd_gps;
	 }
	 
	 Global_GPS_Sensor.NED_Acc[Yr]=Accy=get_acc_delay(Yr,DELAY_GPS,T);
	 Global_GPS_Sensor.NED_Acc[Xr]=Accx=get_acc_delay(Xr,DELAY_GPS,T);
	 u8 flag_sensor[3]={1,0,1};
	
#if !USE_UKF_FROM_AUTOQUAD
	 double Zx[3]={Posx,Sdpx,Accx};
	 double Zy[3]={Posy,Sdpy,Accy};
	 if((gps_init&&gps_data_vaild)||force_test)//bei
   KF_OLDX_NAV( X_KF_NAV[Yr],  P_KF_NAV[Yr],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps,  T);

	 if((gps_init&&gps_data_vaild)||force_test)//dong 
	 KF_OLDX_NAV( X_KF_NAV[Xr],  P_KF_NAV[Xr],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_gps,  g_spd_gps,  T);

	 float X_KF_NAV_TEMP[2][3];//0->x->east  1->y->north
	 X_KF_NAV_TEMP[Xr][0]=X_KF_NAV[Xr][0]+DELAY_GPS*X_KF_NAV[Xr][1]+1/2*pow(DELAY_GPS,2)*(Accx+X_KF_NAV[Xr][2]);
	 X_KF_NAV_TEMP[Xr][1]=X_KF_NAV[Xr][1]+DELAY_GPS*(Accx+X_KF_NAV[Xr][2]);
	 X_KF_NAV_TEMP[Xr][2]=X_KF_NAV[Xr][2];
	 X_KF_NAV_TEMP[Yr][0]=X_KF_NAV[Yr][0]+DELAY_GPS*X_KF_NAV[Yr][1]+1/2*pow(DELAY_GPS,2)*(Accy+X_KF_NAV[Yr][2]);
	 X_KF_NAV_TEMP[Yr][1]=X_KF_NAV[Yr][1]+DELAY_GPS*(Accy+X_KF_NAV[Yr][2]);
	 X_KF_NAV_TEMP[Yr][2]=X_KF_NAV[Yr][2];						
#else//<<---------------------------------UKF-AUTOQUAD----------------------------------	
  static u16 init_ukf,init_ukf_cnt;	 
	float X_KF_NAV_TEMP[2][3];//0->x->east  1->y->north 
	if(init_ukf_cnt++>320)init_ukf=1;
		
	if(init_ukf)
  runTaskCode(Global_GPS_Sensor.NED_Pos[Yr],Global_GPS_Sensor.NED_Pos[Xr],Global_GPS_Sensor.NED_Pos[Zr],
	            Global_GPS_Sensor.NED_Vel[Yr],Global_GPS_Sensor.NED_Vel[Xr],Global_GPS_Sensor.NED_Vel[Zr],T );
		
	 double Zx_kf[3]={0,0,0};
	 double Zy_kf[3]={0,0,0};
	 u8 flag_sensor_flow_gps[3]={1,1,1};
	 

	 static u8 gps_flow_switch_flag=2,has_off_flag;
	 static u16 loss_check_cnt;
	 static u16 cnt_off_yaw_correct;
	  force_test=1;		
	 if(gpsx.pvt.PVT_numsv>4&&gpsx.pvt.PVT_fixtype>=1&&gpsx.pvt.PVT_latitude!=0)
	 { gps_flow_switch_flag=1;
		 Zx_kf[0]=Posx;
		 Zx_kf[1]=UKF_VELE_F;
		 Zy_kf[0]=Posy;
		 Zy_kf[1]=UKF_VELN_F; 
		 g_pos_use=	 g_pos_gps;
		 g_spd_use=	 g_spd_gps;	 
	 }
	 else 
	 { 
		 force_test=1;		
		 gps_flow_switch_flag=0;
		//convert qr to nez
		 if(loss_check_cnt>10/T)
		 {
		 has_off_flag=0;
		 }
		 
		 if(pi_flow.connect==1&&pi_flow.check==1){
		 loss_check_cnt=0;	
		 cnt_off_yaw_correct++;
			 if(cnt_off_yaw_correct>333)
			 { has_off_flag=1;cnt_off_yaw_correct=0; }			
			 if(has_off_flag==0&&pi_flow.yaw!=0){
			 pi_flow.yaw_off=Moving_Median(0,19,Yaw-pi_flow.sensor.yaw);
			 }
		
			 if(pi_flow.yaw_off!=0){
			 Global_GPS_Sensor.NED_Posf[Yr]=-pi_flow.sensor.y*K_pos_qr*cos(pi_flow.yaw_off*0.0173)-pi_flow.sensor.x*K_pos_qr*sin(pi_flow.yaw_off*0.0173);
			 Global_GPS_Sensor.NED_Posf[Xr]=pi_flow.sensor.x*K_pos_qr*cos(pi_flow.yaw_off*0.0173)-pi_flow.sensor.y*K_pos_qr*sin(pi_flow.yaw_off*0.0173);
			 }
       Global_GPS_Sensor.NED_Posf[Zr]=pi_flow.sensor.z*K_pos_qr;
			 if(Global_GPS_Sensor.NED_Posf[Yr]!=0&&Global_GPS_Sensor.NED_Posf[Xr]!=0){
			 Global_GPS_Sensor.NED_Posf_reg[Xr]=Global_GPS_Sensor.NED_Posf[Xr];
			 Global_GPS_Sensor.NED_Posf_reg[Yr]=Global_GPS_Sensor.NED_Posf[Yr];
			 }
		 }else 
		 loss_check_cnt++;
		 
		 
		//convert flow to nez		 
		 Global_GPS_Sensor.NED_Velf[Yr]=flowy*K_spd_flow*cos(Yaw*0.0173)-flowx*K_spd_flow*sin(Yaw*0.0173);
		 Global_GPS_Sensor.NED_Velf[Xr]=flowy*K_spd_flow*sin(Yaw*0.0173)+flowx*K_spd_flow*cos(Yaw*0.0173);
		 Zx_kf[0]=Global_GPS_Sensor.NED_Posf_reg[Xr];
		 Zx_kf[1]=Global_GPS_Sensor.NED_Velf[Xr];
		 Zy_kf[0]=Global_GPS_Sensor.NED_Posf_reg[Yr];
		 Zy_kf[1]=Global_GPS_Sensor.NED_Velf[Yr]; 
		 g_pos_use=g_pos_flow;
		 g_spd_use=g_spd_flow;	 

		 if(pi_flow.check==0||!pi_flow.insert||pi_flow.yaw_off==0||pi_flow.sensor.y==0||pi_flow.sensor.x==0)
		 {flag_sensor_flow_gps[0]=0;
		 H[0]=0;} 
	 }
	 static u8 gps_flow_switch_flag_reg;
	 if(gps_flow_switch_flag==0&&gps_flow_switch_flag_reg==1)//gps->flow
	 {
	 X_KF_NAV[Xr][0]= Global_GPS_Sensor.NED_Posf_reg[Xr];
	 X_KF_NAV[Yr][0]= Global_GPS_Sensor.NED_Posf_reg[Yr];	 
	 }
	 else if(gps_flow_switch_flag==1&&gps_flow_switch_flag_reg==0)//flow->gps
	 {
	 X_KF_NAV[Xr][0]= Global_GPS_Sensor.NED_Pos[Xr];
	 X_KF_NAV[Yr][0]= Global_GPS_Sensor.NED_Pos[Yr];	
	 }
	 static u16 cnt_outrange;
	 if(fabs(X_KF_NAV[Xr][0]-Zx_kf[0])>1.5||fabs(X_KF_NAV[Yr][0]-Zy_kf[0])>1.5)
	  cnt_outrange++;
	 if(cnt_outrange>2/T){
			if(gps_flow_switch_flag==0&&pi_flow.connect&&1)//flow
			{cnt_outrange=0;
			X_KF_NAV[Xr][0]= Global_GPS_Sensor.NED_Posf_reg[Xr];
			X_KF_NAV[Yr][0]= Global_GPS_Sensor.NED_Posf_reg[Yr];	 
			}
		}
	 gps_flow_switch_flag_reg=gps_flow_switch_flag;
		
	 //KF	global
	 static u8 cnt_kf;
		float Z_x[3]={Zx_kf[0],Zx_kf[1],Accx};
		float Z_y[3]={Zy_kf[0],Zy_kf[1],Accy};
		
//		float A_f[3][3]  = {{1, T, -pow(T,2)/2}, {0, 1, -T}, {0, 0, 1}};
//    float B_f[3][1]  = {pow(T,2)/2,T,1};
//		float H_f[3][3]  = {{0,0,0}, {0, 1, 0}, {0, 0, 0}};
//		
//		float Z_f[3][1]= {Zx_kf[0],Zx_kf[1],0};
//		float U=Accx;
//		float Q_f[3][3]  ={
//{2.50011111111111e-11	,5.00016666666667e-09,	-3.33333333333333e-11},
//{5.00016666666667e-09,	1.00002500000000e-06	,-5.00000000000000e-07},
//{-3.33333333333333e-11,	-5.00000000000000e-09	,1.00000000000000e-06}};
//		float R_f[3][3]  = {{g_pos_use,0,0}, {0, g_spd_use, 0}, {0, 0, 0.00001}};	
//	//	KF_OLDX4((float *)X_f,(float *)P_f,(float *)Z_f,(float *) A_f, (float *)B_f, (float *)H_f, Z_f[2][0],(float *)Q_f,(float *)R_f);
//    unsigned int S31[2] = {3,1};
//    unsigned int S33[2] = {3,3};
//    float I[3][3]= {{1,0,0}, {0, 1, 0}, {0, 0, 1}};
//		float Temp31_1[3][1]={0};
//		float Temp31_2[3][1]={0};
//		float Temp31_3[3][1]={0};
//		float Temp31_4[3][1]={0};
//		
//		float Temp33_1[3][3]={0};
//		float Temp33_2[3][3]={0};
//		float Temp33_3[3][3]={0};
//		float Temp33_4[3][3]={0};
//		float Temp33_5[3][3]={0};
//		float Temp33_6[3][3]={0};
//		//x_pre //X_pre=A*X+B*U;
//		if(init_ukf&&U!=0){
//    float X_pre[3][1]={0} ;
//	  MatMultiplication( (float *)A_f, S33, (float *)X_f, S31, (float *)Temp31_1);
//	  MatScalarMult((float *)B_f, S31, U, (float *)Temp31_2);
//    MatAddition( (float *)Temp31_1, S31, (float *)Temp31_2,(float *)X_pre);
//		//p_pre //P_pre=A*P*A'+Q;
//		float P_pre[3][3]={0};
//		MatTranspose( (float *)A_f, S33, (float *)Temp33_1);	
//		MatMultiplication( (float *)A_f, S33, (float *)P_f, S33, (float *)Temp33_2);
//		MatMultiplication( (float *)Temp33_2, S33, (float *)Temp33_1, S33, (float *)Temp33_3);
//		MatAddition( (float *)Temp33_3, S33, (float *)Q_f,(float *)P_pre);
//		//cal K  K=P_pre*H'*inv(H*P_pre*H'+R);
//		float K[3][3]={0}; 
//		MatMultiplication( (float *)P_pre, S33, (float *)H_f, S33, (float *)Temp33_1);
//		MatMultiplication( (float *)H_f, S33, (float *)P_pre, S33, (float *)Temp33_2);
//		MatMultiplication( (float *)Temp33_2, S33, (float *)H_f, S33, (float *)Temp33_3);
//		MatAddition( (float *)Temp33_3, S33, (float *)R_f,(float *)Temp33_4);
//		MatInv3by3((float *)Temp33_4, (float *)Temp33_5);
//		MatMultiplication( (float *)Temp33_1, S33, (float *)Temp33_5, S33, (float *)K);
//		//correct  X=X_pre+K*(H*Z-H*X_pre);
//		MatMultiplication( (float *)H_f, S33, (float *)Z_f, S31, (float *)Temp31_1);
//		MatMultiplication( (float *)H_f, S33, (float *)X_pre, S31, (float *)Temp31_2);
//		MatSubAddition( (float *)Temp31_1, S31, (float *)Temp31_2,(float *)Temp31_3);
//		MatMultiplication( (float *)K, S33, (float *)Temp31_3, S31, (float *)Temp31_4);
//		MatAddition( (float *)Temp31_1, S31, (float *)Temp31_4,(float *)X_f);
//		//P   //P=(I-K*H)*P_pre;
//		MatMultiplication( (float *)K, S33, (float *)H_f, S33, (float *)Temp33_1);
//		MatSubAddition( (float *)I, S33, (float *)Temp33_1,(float *)Temp33_2);
//		MatMultiplication( (float *)Temp33_2, S33, (float *)P_pre, S33, (float *)P_f);
//	}
	 if(((gps_init&&gps_data_vaild)||force_test)&&init_ukf&&cnt_kf++>0){cnt_kf=0;
	 OLDX_KF3(Z_x,r_flow_new[3],r_flow_new,flag_sensor_flow_gps,X_KF_NAV[Xr],state_correct_posx,T);
	 OLDX_KF3(Z_y,r_flow_new[3],r_flow_new,flag_sensor_flow_gps,X_KF_NAV[Yr],state_correct_posy,T);
  // KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy_kf,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_use,  g_spd_use,  T); 
	// KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx_kf,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_use,  g_spd_use,  T);
   }
	 // NAN check 
  	if(isnan(X_KF_NAV[Xr][0]))
			X_KF_NAV[Xr][0]=Zx_kf[0];
    if(isnan(X_KF_NAV[Xr][1]))
			X_KF_NAV[Xr][1]=Zx_kf[1];	
	  if(isnan(X_KF_NAV[Xr][2]))
			X_KF_NAV[Xr][2]=Zx_kf[2];
    if(isnan(X_KF_NAV[Yr][0]))
			X_KF_NAV[Yr][0]=Zy_kf[0];
	  if(isnan(X_KF_NAV[Yr][1]))
			X_KF_NAV[Yr][1]=Zy_kf[1];
    if(isnan(X_KF_NAV[Yr][2]))
			X_KF_NAV[Yr][2]=Zy_kf[2];	 		
		
	 //0->x->east  1->y->north
	 X_KF_NAV_TEMP[Xr][0]=X_KF_NAV[Xr][0]+DELAY_GPS*X_KF_NAV[Xr][1];//+1/2*pow(DELAY_GPS,2)*(Accx-X_KF_NAV[Xr][2]);
	 X_KF_NAV_TEMP[Xr][1]=X_KF_NAV[Xr][1];//+DELAY_GPS*(Accx+X_KF_NAV[0][2]);
	 X_KF_NAV_TEMP[Xr][2]=X_KF_NAV[Xr][2];
	 X_KF_NAV_TEMP[Yr][0]=X_KF_NAV[Yr][0]+DELAY_GPS*X_KF_NAV[Yr][1];//+1/2*pow(DELAY_GPS,2)*(Accy-X_KF_NAV[Yr][2]);
	 X_KF_NAV_TEMP[Yr][1]=X_KF_NAV[Yr][1];//+DELAY_GPS*(Accy+X_KF_NAV[1][2]);
	 X_KF_NAV_TEMP[Yr][2]=X_KF_NAV[Yr][2];						

	// X_KF_NAV_TEMP[Yr][0]=Global_GPS_Sensor.NED_Posf_reg[Yr];
	// X_KF_NAV_TEMP[Xr][0]=Global_GPS_Sensor.NED_Posf_reg[Xr];
#endif

	 X_ukf[0]=X_KF_NAV_TEMP[Yr][0];//North pos	
	 X_ukf[2]=X_KF_NAV_TEMP[Yr][2];
	 X_ukf[3]=X_KF_NAV_TEMP[Xr][0];//East  pos
	 X_ukf[5]=X_KF_NAV_TEMP[Xr][2];
	//global
	 X_ukf_global[1]=X_ukf[1]=X_KF_NAV_TEMP[Yr][1];//North vel
	 X_ukf_global[4]=X_ukf[4]=X_KF_NAV_TEMP[Xr][1];//East  vel
	//turn to body frame
	 X_ukf[4]=X_KF_NAV_TEMP[Yr][1]*cos(Yaw*0.0173)+X_KF_NAV_TEMP[Xr][1]*sin(Yaw*0.0173);//Y
	 X_ukf[1]=-X_KF_NAV_TEMP[Yr][1]*sin(Yaw*0.0173)+X_KF_NAV_TEMP[Xr][1]*cos(Yaw*0.0173);//X
	//output to fc 
	 X_ukf_Pos[1]=X_ukf[0];//North Pos
   X_ukf_Pos[0]=X_ukf[3];//East Pos  
	 if((gps_init&&gps_data_vaild))
   CalcGlobalLocation(X_ukf[0],X_ukf[3]);
  }
//--------=-----------------flow in global---------------------------------------------------------
	else if(kf_data_sel_temp==2){	 
	 static int qr_yaw_init;	
	 #if SENSOR_FORM_PI_FLOW	
	 float Yaw_qr=To_180_degrees(Yaw);
	 #else
	 float Yaw_qr=To_180_degrees(Yaw+yaw_qr_off);	
   #endif		
	 float ACCY=flow_matlab_data[1];
   float ACCX=flow_matlab_data[0];
	 float accEast=ACCY*sin(Yaw_qr*0.0173)+ACCX*cos(Yaw_qr*0.0173);
   float accNorth=ACCY*cos(Yaw_qr*0.0173)-ACCX*sin(Yaw_qr*0.0173);
   float SPDY=flowy*K_spd_flow;
	 float SPDX=flowx*K_spd_flow;
	 float acc_bias[2]={0};

	 velNorth=SPDY*cos(Yaw_qr*0.0173)-SPDX*sin(Yaw_qr*0.0173);
   velEast=SPDY*sin(Yaw_qr*0.0173)+SPDX*cos(Yaw_qr*0.0173);
	 #if SENSOR_FORM_PI_FLOW
	 if(pi_flow.check==0&&pi_flow.connect)
	 H[0]=0; 
	 #else
	 if(qr.check==0&&qr.connect)
	 H[0]=0; 
	 #endif
	 static float pos_reg[2];
	 #if SENSOR_FORM_PI_FLOW
	 Qr_y=-pi_flow.sensor.y;
	 Posy=Qr_y*K_pos_qr;
	 Qr_x=pi_flow.sensor.x;
	 Posx=Qr_x*K_pos_qr;
	 #else
   Qr_y=-qr.y;
	 Posy=Qr_y*K_pos_qr;
	 Qr_x=qr.x;
	 Posx=Qr_x*K_pos_qr;
	 #endif
	 Sdpy=velNorth*K_spd_gps;
	 Accy=accNorth*flag_kf1[1];
	 Sdpx=velEast*K_spd_gps;
	 Accx=accEast*flag_kf1[0];
	 static u8 state_init_flow_pos;
	 switch(state_init_flow_pos)
	 {
		 case 0:
			  if(qr.check&&qr.connect)
				{
				state_init_flow_pos=1;
				X_KF_NAV[1][0]=Posy;X_KF_NAV[0][0]=Posx;
				}
			 break;
		 case 1:
			 if(ALT_POS_SONAR2<0.15||!fly_ready)
				state_init_flow_pos=0;
			break; 
	 }
	 double Zy[3]={Posy,Sdpy,acc_bias[1]};
	 if(1)//bei 
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 double Zx[3]={Posx,Sdpx,acc_bias[0]};
	 if(1)//dong
   KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 X_ukf[0]=X_KF_NAV[0][0]+X_ukf[2]*T*15;//East pos
	 //X_ukf[1]=X_KF_NAV[0][1];//East vel
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0]+X_ukf[5]*T*15;//North  pos
	 //X_ukf[4]=X_KF_NAV[1][1];//North  vel
	 X_ukf[5]=X_KF_NAV[1][2];							
	 X_ukf[1]=-X_KF_NAV[1][1]*sin(Yaw_qr*0.0173)+X_KF_NAV[0][1]*cos(Yaw_qr*0.0173);//X
	 X_ukf[4]= X_KF_NAV[1][1]*cos(Yaw_qr*0.0173)+X_KF_NAV[0][1]*sin(Yaw_qr*0.0173);//Y
   if(fabs( X_ukf[0]-pos_reg[0])>1.5||fabs( X_ukf[3]-pos_reg[1])>1.5){
		  if(qr.connect)
			{X_KF_NAV[1][0]=Posy;X_KF_NAV[0][0]=Posx;}
			else
			{X_KF_NAV[1][0]= pos_reg[1];X_KF_NAV[0][0]= pos_reg[0];}	
		}
	 pos_reg[0]=X_ukf[0];
	 pos_reg[1]=X_ukf[3];	
 	 X_ukf_Pos[0]=X_ukf[0];//East Pos
   X_ukf_Pos[1]=X_ukf[3];//North Pos
	}else{
//-------------------------------------------flow in body-------------------------------------------------------------------------	
	 if(qr.check==0)
	 H[0]=0; 
   Qr_y=-qr.y;
	 Qr_x=qr.x;  
   Posx=Qr_x*K_pos_qr;
	 Sdpx=flowx*K_spd_flow;
	 Accx=accx*acc_flag_flow[0];
	 double Zx[3]={Posx,Sdpx,0};
   KF_OLDX_NAV( X_KF_NAV[0],  P_KF_NAV[0],  Zx,  Accx, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 Posy=Qr_y*K_pos_qr;
   Sdpy=flowy*K_spd_flow;
	 Accy=accy*acc_flag_flow[1];
	 double Zy[3]={Posy,Sdpy,0};
   KF_OLDX_NAV( X_KF_NAV[1],  P_KF_NAV[1],  Zy,  Accy, A,  B,  H,  ga_nav,  gwa_nav, g_pos_flow,  g_spd_flow,  T);
	 X_ukf[0]=X_KF_NAV[0][0];
	 X_ukf[1]=X_KF_NAV[0][1];
	 X_ukf[2]=X_KF_NAV[0][2];
	 X_ukf[3]=X_KF_NAV[1][0];
	 X_ukf[4]=X_KF_NAV[1][1];
	 X_ukf[5]=X_KF_NAV[1][2];
	 
 	 X_ukf_Pos[0]=X_ukf[0];//X
   X_ukf_Pos[1]=X_ukf[3];//Y
 }
	#endif
}


