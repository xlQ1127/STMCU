// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "main.h"

unsigned int  TestInt = 0;
#define ARRAYGROESSE 10
unsigned char Array[ARRAYGROESSE] = {1,2,3,4,5,6,7,8,9,10};
char DisplayBuff[80] = "Hallo Welt";
unsigned char DispPtr = 0; 
unsigned char RemoteTasten = 0;

#define KEY1    0x01
#define KEY2    0x02
#define KEY3    0x04
#define KEY4    0x08
#define KEY5    0x10

void LcdClear(void)
{
 unsigned char i; 
 for(i=0;i<80;i++) DisplayBuff[i] = ' '; 
}

void Menu(void)
 {
  static unsigned char MaxMenue = 11,MenuePunkt=0; 
 
  if(RemoteTasten & KEY1) { if(MenuePunkt) MenuePunkt--; else MenuePunkt = MaxMenue; LcdClear(); RemotePollDisplayLine = -1; }
  if(RemoteTasten & KEY2) { MenuePunkt++; LcdClear(); RemotePollDisplayLine = -1;}
  if((RemoteTasten & KEY1) && (RemoteTasten & KEY2)) MenuePunkt = 0;
  if(MenuePunkt < 10) {LCD_printfxy(17,0,"[%i]",MenuePunkt);} else {LCD_printfxy(16,0,"[%i]",MenuePunkt);};
  switch(MenuePunkt)
   {
    case 0: 
           LCD_printfxy(0,0,"+ MikroKopter +");
           LCD_printfxy(0,1,"HW:V%d.%d SW:%d.%d%c",PlatinenVersion/10,PlatinenVersion%10,VERSION_HAUPTVERSION, VERSION_NEBENVERSION,VERSION_INDEX+'a');
           LCD_printfxy(0,2,"Setting: %d ",GetActiveParamSetNumber());
           LCD_printfxy(0,3,"(c) Holger Buss");
//           if(RemoteTasten & KEY3) TestInt--;
//           if(RemoteTasten & KEY4) TestInt++;
           break;
    case 1: 
          if(EE_Parameter.GlobalConfig & CFG_HOEHENREGELUNG)
           {         
           LCD_printfxy(0,0,"Hoehe:     %5i",HoehenWert);
           LCD_printfxy(0,1,"SollHoehe: %5i",SollHoehe);
           LCD_printfxy(0,2,"Luftdruck: %5i",MessLuftdruck);
           LCD_printfxy(0,3,"Off      : %5i",DruckOffsetSetting);
           }
           else 
           {         
           LCD_printfxy(0,1,"Keine ");
           LCD_printfxy(0,2,"Höhenregelung");
           }
           
           break;
    case 2: 
           LCD_printfxy(0,0,"akt. Lage");
           LCD_printfxy(0,1,"Nick:      %5i",IntegralNick/1024);
           LCD_printfxy(0,2,"Roll:      %5i",IntegralRoll/1024);
           LCD_printfxy(0,3,"Kompass:   %5i",KompassValue);
           break;
    case 3: 
           LCD_printfxy(0,0,"K1:%4i  K2:%4i ",PPM_in[1],PPM_in[2]);
           LCD_printfxy(0,1,"K3:%4i  K4:%4i ",PPM_in[3],PPM_in[4]);
           LCD_printfxy(0,2,"K5:%4i  K6:%4i ",PPM_in[5],PPM_in[6]);
           LCD_printfxy(0,3,"K7:%4i  K8:%4i ",PPM_in[7],PPM_in[8]);
           break;
    case 4: 
           LCD_printfxy(0,0,"Ni:%4i  Ro:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_NICK]],PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]]);
           LCD_printfxy(0,1,"Gs:%4i  Gi:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_GAS]],PPM_in[EE_Parameter.Kanalbelegung[K_GIER]]);
           LCD_printfxy(0,2,"P1:%4i  P2:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_POTI1]],PPM_in[EE_Parameter.Kanalbelegung[K_POTI2]]);
           LCD_printfxy(0,3,"P3:%4i  P4:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_POTI3]],PPM_in[EE_Parameter.Kanalbelegung[K_POTI4]]);
           break;
    case 5: 
           LCD_printfxy(0,0,"Gyro - Sensor");
          if(PlatinenVersion == 10)
          {
           LCD_printfxy(0,1,"Nick %4i (%3i)",AdWertNick - AdNeutralNick, AdNeutralNick);
           LCD_printfxy(0,2,"Roll %4i (%3i)",AdWertRoll - AdNeutralRoll, AdNeutralRoll);
           LCD_printfxy(0,3,"Gier %4i (%3i)",MesswertGier, AdNeutralGier);
          }
          else  
          {
           LCD_printfxy(0,1,"Nick %4i (%3i)",AdWertNick - AdNeutralNick, AdNeutralNick/2);
           LCD_printfxy(0,2,"Roll %4i (%3i)",AdWertRoll - AdNeutralRoll, AdNeutralRoll/2);
           LCD_printfxy(0,3,"Gier %4i (%3i)",MesswertGier, AdNeutralGier/2);
          }
           break;
    case 6: 
           LCD_printfxy(0,0,"ACC - Sensor");
           LCD_printfxy(0,1,"Nick %4i (%3i)",AdWertAccNick,NeutralAccX);
           LCD_printfxy(0,2,"Roll %4i (%3i)",AdWertAccRoll,NeutralAccY);
           LCD_printfxy(0,3,"Hoch %4i (%3i)",Mittelwert_AccHoch/*accumulate_AccHoch / messanzahl_AccHoch*/,(int)NeutralAccZ);
           break;
    case 7: 
           LCD_printfxy(0,1,"Spannung:  %5i",UBat);
           LCD_printfxy(0,2,"Empf.Pegel:%5i",SenderOkay);
           break;
    case 8: 
           LCD_printfxy(0,0,"Kompass       ");
           LCD_printfxy(0,1,"Richtung:  %5i",KompassRichtung);
           LCD_printfxy(0,2,"Messwert:  %5i",KompassValue);
           LCD_printfxy(0,3,"Start:     %5i",KompassStartwert);
           break;
    case 9: 
           LCD_printfxy(0,0,"Poti1:  %3i",Poti1);
           LCD_printfxy(0,1,"Poti2:  %3i",Poti2);
           LCD_printfxy(0,2,"Poti3:  %3i",Poti3);
           LCD_printfxy(0,3,"Poti4:  %3i",Poti4);
           break;
    case 10: 
           LCD_printfxy(0,0,"Servo  " );
           LCD_printfxy(0,1,"Setpoint  %3i",Parameter_ServoNickControl);
           LCD_printfxy(0,2,"Stellung: %3i",ServoValue);
           LCD_printfxy(0,3,"Range:%3i-%3i",EE_Parameter.ServoNickMin,EE_Parameter.ServoNickMax);
           break;
    case 11: 
           LCD_printfxy(0,0,"ExternControl  " );
           LCD_printfxy(0,1,"Ni:%4i  Ro:%4i ",ExternControl.Nick,ExternControl.Roll);
           LCD_printfxy(0,2,"Gs:%4i  Gi:%4i ",ExternControl.Gas,ExternControl.Gier);
           LCD_printfxy(0,3,"Hi:%4i  Cf:%4i ",ExternControl.Hight,ExternControl.Config);
           break;
    default: MaxMenue = MenuePunkt - 1;
             MenuePunkt = 0; 
           break;
    }
 RemoteTasten = 0;
}
