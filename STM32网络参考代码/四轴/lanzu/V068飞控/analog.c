// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
++/
#include "main.h"

volatile int  Aktuell_Nick,Aktuell_Roll,Aktuell_Gier,Aktuell_ax, Aktuell_ay,Aktuell_az, UBat = 100;
volatile int  AdWertNick = 0, AdWertRoll = 0, AdWertGier = 0;
volatile int  AdWertAccRoll = 0,AdWertAccNick = 0,AdWertAccHoch = 0;
volatile char MessanzahlNick = 0, MessanzahlRoll = 0, MessanzahlGier = 0;
volatile char messanzahl_AccNick = 0, messanzahl_AccRoll = 0, messanzahl_AccHoch = 0;
volatile long Luftdruck = 32000;
volatile int  StartLuftdruck;
volatile unsigned int  MessLuftdruck = 1023;
unsigned char DruckOffsetSetting;
volatile int HoeheD = 0;
volatile char messanzahl_Druck;
volatile int  tmpLuftdruck;
volatile unsigned int ZaehlMessungen = 0;

//#######################################################################################
//
void ADC_Init(void)
//#######################################################################################
{ 
    ADMUX = 0;//Referenz ist extern
    ADCSRA=(1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE);
    //Free Running Mode, Division Factor 128, Interrupt on
}

void SucheLuftruckOffset(void)
{
 unsigned int off;
 off = eeprom_read_byte(&EEPromArray[EEPROM_ADR_LAST_OFFSET]);
 if(off > 20) off -= 10;
 OCR0A = off;
 Delay_ms_Mess(100);
 if(MessLuftdruck < 850) off = 0;
 for(; off < 250;off++)
  {
  OCR0A = off;
  Delay_ms_Mess(50);
  printf(".");  
  if(MessLuftdruck < 900) break;
  }
 eeprom_write_byte(&EEPromArray[EEPROM_ADR_LAST_OFFSET], off);
 DruckOffsetSetting = off;
 Delay_ms_Mess(300);
}


//#######################################################################################
//
SIGNAL(SIG_ADC)
//#######################################################################################
{
    static unsigned char kanal=0,state = 0;
    static unsigned int gier1, roll1, nick1;
    ANALOG_OFF;
    switch(state++)
        {
        case 0:
            gier1 = ADC;
            kanal = 1;
            ZaehlMessungen++;
            break;
        case 1:
            roll1 = ADC;
            kanal = 2;
            break;
        case 2:
            nick1 = ADC;
            kanal = 4;
            break;
        case 3:
            UBat = (3 * UBat + ADC / 3) / 4;//(UBat + ((ADC * 39) / 256) + 19)  / 2;
            kanal = 6;
            break;
        case 4:
            Aktuell_ay = NeutralAccY - ADC;
            AdWertAccRoll = Aktuell_ay;
            kanal = 7;
            break;
        case 5:
            Aktuell_ax = ADC - NeutralAccX;
            AdWertAccNick =  Aktuell_ax;
		    kanal = 0;
            break;
        case 6:
            if(PlatinenVersion == 10)  AdWertGier = (ADC + gier1) / 2;
			else 					   AdWertGier = ADC + gier1;
            kanal = 1;
            break;
        case 7:
            if(PlatinenVersion == 10)  AdWertRoll = (ADC + roll1) / 2;
			else 					   AdWertRoll = ADC + roll1;
            kanal = 2;
            break;
        case 8:
            if(PlatinenVersion == 10)  AdWertNick = (ADC + nick1) / 2;
			else 					   AdWertNick = ADC + nick1;
//AdWertNick = 0;
//AdWertNick += Poti2;            
            kanal = 5;
            break;
       case 9:
            AdWertAccHoch =  (signed int) ADC - NeutralAccZ;
            AdWertAccHoch += abs(Aktuell_ay) / 4 + abs(Aktuell_ax) / 4;
            if(AdWertAccHoch > 1) 
             {
              if(NeutralAccZ < 800) NeutralAccZ+= 0.02; 
             }  
             else if(AdWertAccHoch < -1)
             {
              if(NeutralAccZ > 600) NeutralAccZ-= 0.02;
             } 
            messanzahl_AccHoch = 1;
            Aktuell_az = ADC;
            Mess_Integral_Hoch += AdWertAccHoch;      // Integrieren
            Mess_Integral_Hoch -= Mess_Integral_Hoch / 1024; // dämfen
 	        kanal = 3;
            break;
        case 10:
            tmpLuftdruck += ADC;
            if(++messanzahl_Druck >= 5) 
                {
                MessLuftdruck = ADC;
                messanzahl_Druck = 0;
				HoeheD = (int)(StartLuftdruck - tmpLuftdruck - HoehenWert);  // D-Anteil = neuerWert - AlterWert
                Luftdruck = (tmpLuftdruck + 3 * Luftdruck) / 4;
                HoehenWert = StartLuftdruck - Luftdruck;
                tmpLuftdruck = 0;
                } 
            kanal = 0;
            state = 0;
            break;
        default: 
            kanal = 0;
            state = 0;
            break;
        } 
    ADMUX = kanal;
    if(state != 0) ANALOG_ON;
}
