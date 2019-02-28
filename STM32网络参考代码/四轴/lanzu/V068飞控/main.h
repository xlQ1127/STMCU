#ifndef _MAIN_H
 #define _MAIN_H

//Hier die Quarz Frequenz einstellen
#if defined (__AVR_ATmega32__)
#define SYSCLK	20000000L	//Quarz Frequenz in Hz
#endif

#if defined (__AVR_ATmega644__)
#define SYSCLK	20000000L	//Quarz Frequenz in Hz
//#define SYSCLK	16000000L	//Quarz Frequenz in Hz
#endif

// neue Hardware
#define ROT_OFF   {if(PlatinenVersion == 10) PORTB &=~0x01; else  PORTB |= 0x01;}
#define ROT_ON    {if(PlatinenVersion == 10) PORTB |= 0x01; else  PORTB &=~0x01;}
#define ROT_FLASH PORTB ^= 0x01
#define GRN_OFF   PORTB &=~0x02 
#define GRN_ON    PORTB |= 0x02 
#define GRN_FLASH PORTB ^= 0x02



#define F_CPU SYSCLK
//#ifndef F_CPU
//#error ################## F_CPU nicht definiert oder ungültig #############
//#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define EEPROM_ADR_VALID            1
#define EEPROM_ADR_ACTIVE_SET       2
#define EEPROM_ADR_LAST_OFFSET      3

#define EEPROM_ADR_ACC_NICK         4
#define EEPROM_ADR_ACC_ROLL         6
#define EEPROM_ADR_ACC_Z            8

#define EEPROM_ADR_PARAM_BEGIN      100

#define CFG_HOEHENREGELUNG       0x01
#define CFG_HOEHEN_SCHALTER      0x02
#define CFG_HEADING_HOLD         0x04
#define CFG_KOMPASS_AKTIV        0x08
#define CFG_KOMPASS_FIX          0x10
#define CFG_GPS_AKTIV            0x20
#define CFG_ACHSENKOPPLUNG_AKTIV 0x40
#define CFG_DREHRATEN_BEGRENZER  0x80

#define CFG_LOOP_OBEN       0x01
#define CFG_LOOP_UNTEN      0x02
#define CFG_LOOP_LINKS      0x04
#define CFG_LOOP_RECHTS     0x08

//#define  SYSCLK  
//extern unsigned long SYSCLK;
extern volatile int i_Nick[20],i_Roll[20],DiffNick,DiffRoll;
extern volatile unsigned char SenderOkay;
extern unsigned char CosinusNickWinkel, CosinusRollWinkel;
extern unsigned char PlatinenVersion;

void ReadParameterSet (unsigned char number, unsigned char *buffer, unsigned char length);
void WriteParameterSet(unsigned char number, unsigned char *buffer, unsigned char length);
extern unsigned char GetActiveParamSetNumber(void);
extern unsigned char EEPromArray[];

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <avr/wdt.h>

#include "old_macros.h"

#include "_Settings.h"
#include "printf_P.h"
#include "timer0.h"
#include "uart.h"
#include "analog.h"
#include "twimaster.h"
#include "menu.h"
#include "rc.h"
#include "fc.h"
#include "gps.h"
#include "spi.h"


#ifndef EEMEM
#define EEMEM __attribute__ ((section (".eeprom")))
#endif

#define DEBUG_DISPLAY_INTERVALL  123 // in ms


#define DELAY_US(x)	((unsigned int)( (x) * 1e-6 * F_CPU ))
	
#endif //_MAIN_H






