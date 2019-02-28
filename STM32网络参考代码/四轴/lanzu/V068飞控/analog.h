/*#######################################################################################

#######################################################################################*/

extern volatile int UBat;
extern volatile int  AdWertNick, AdWertRoll, AdWertGier;
extern volatile int  AdWertAccRoll,AdWertAccNick,AdWertAccHoch;
extern volatile int  Aktuell_Nick,Aktuell_Roll,Aktuell_Gier,Aktuell_ax, Aktuell_ay,Aktuell_az;
extern volatile long  Luftdruck;
extern volatile char messanzahl_Druck;
extern volatile unsigned int ZaehlMessungen;
extern unsigned char DruckOffsetSetting;
extern volatile int HoeheD;
extern volatile unsigned int  MessLuftdruck;
extern volatile int  StartLuftdruck;
extern volatile char MessanzahlNick;

unsigned int ReadADC(unsigned char adc_input);
void         ADC_Init(void);
void SucheLuftruckOffset(void);


#define ANALOG_OFF ADCSRA=0
#define ANALOG_ON ADCSRA=(1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE)
