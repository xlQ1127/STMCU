// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+UART
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "main.h"
#include "uart.h"

unsigned char DebugGetAnforderung = 0,DebugDisplayAnforderung = 0,DebugDataAnforderung = 0,GetVersionAnforderung = 0;
unsigned volatile char SioTmp = 0;
unsigned volatile char SendeBuffer[MAX_SENDE_BUFF];
unsigned volatile char RxdBuffer[MAX_EMPFANGS_BUFF];
unsigned volatile char NMEABuffer[MAX_EMPFANGS_BUFF];
unsigned volatile char NeuerDatensatzEmpfangen = 0;
unsigned volatile char NeueKoordinateEmpfangen = 0;
unsigned volatile char UebertragungAbgeschlossen = 1;
unsigned volatile char CntCrcError = 0;
unsigned volatile char AnzahlEmpfangsBytes = 0;
unsigned volatile char PC_DebugTimeout = 0;
unsigned char RemotePollDisplayLine = 0;
unsigned char NurKanalAnforderung = 0;
unsigned char DebugTextAnforderung = 255;
unsigned char PcZugriff = 100;
unsigned char MotorTest[4] = {0,0,0,0};
unsigned char DubWiseKeys[4] = {0,0,0,0}; 
unsigned char MeineSlaveAdresse;
unsigned char ConfirmFrame;
struct str_DebugOut    DebugOut;
struct str_ExternControl  ExternControl;
struct str_VersionInfo VersionInfo;
int Debug_Timer;

const unsigned char ANALOG_TEXT[32][16] =
{
   //1234567890123456 
    "IntegralNick    ", //0
    "IntegralRoll    ",
    "AccNick         ", 
    "AccRoll         ",
    "GyroGier        ",
    "HoehenWert      ", //5
    "AccZ            ",
    "Gas             ",
    "KompassValue    ",
    "Spannung        ",
    "Empfang         ", //10
    "Analog11        ",
    "Motor_Vorne     ",
    "Motor_Hinten    ",
    "Motor_Links     ",
    "Motor_Rechts    ", //15
    "Acc_Z           ",
    "MittelAccNick   ",
    "MittelAccRoll   ",
    "IntegralErrNick ",
    "IntegralErrRoll ", //20
    "MittelIntNick   ",
    "MittelIntRoll	 ",
    "NeutralNick     ",
    "RollOffset      ",
    "IntRoll*Faktor  ", //25
    "Analog26        ",
    "DirektAusglRoll ",
    "MesswertRoll    ",
    "AusgleichRoll   ",
    "I-LageRoll      ", //30
    "StickRoll       "
};



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++ Sende-Part der Datenübertragung
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
SIGNAL(INT_VEC_TX)
{
 static unsigned int ptr = 0;
 unsigned char tmp_tx;
 if(!UebertragungAbgeschlossen)  
  {
   ptr++;                    // die [0] wurde schon gesendet
   tmp_tx = SendeBuffer[ptr];  
   if((tmp_tx == '\r') || (ptr == MAX_SENDE_BUFF))
    {
     ptr = 0;
     UebertragungAbgeschlossen = 1;
    }
   UDR = tmp_tx; 
  } 
  else ptr = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++ Empfangs-Part der Datenübertragung, incl. CRC-Auswertung
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
SIGNAL(INT_VEC_RX)
{
 static unsigned int crc;
 static unsigned char crc1,crc2,buf_ptr;
 static unsigned char UartState = 0;
 unsigned char CrcOkay = 0;

 SioTmp = UDR; 
 if(buf_ptr >= MAX_EMPFANGS_BUFF)    UartState = 0;
 if(SioTmp == '\r' && UartState == 2) 
  {
   UartState = 0;
   crc -= RxdBuffer[buf_ptr-2];
   crc -= RxdBuffer[buf_ptr-1];
   crc %= 4096;
   crc1 = '=' + crc / 64;
   crc2 = '=' + crc % 64;
   CrcOkay = 0;
   if((crc1 == RxdBuffer[buf_ptr-2]) && (crc2 == RxdBuffer[buf_ptr-1])) CrcOkay = 1; else { CrcOkay = 0; CntCrcError++;};
   if(!NeuerDatensatzEmpfangen && CrcOkay) // Datensatz schon verarbeitet
    {
     NeuerDatensatzEmpfangen = 1; 
	 AnzahlEmpfangsBytes = buf_ptr;
     RxdBuffer[buf_ptr] = '\r';
	 if(RxdBuffer[2] == 'R') wdt_enable(WDTO_250MS); // Reset-Commando
	}				  
  }
  else
  switch(UartState)
  {
   case 0:
          if(SioTmp == '#' && !NeuerDatensatzEmpfangen) UartState = 1;  // Startzeichen und Daten schon verarbeitet
		  buf_ptr = 0;
		  RxdBuffer[buf_ptr++] = SioTmp;
		  crc = SioTmp;
          break;
   case 1: // Adresse auswerten
		  UartState++;
		  RxdBuffer[buf_ptr++] = SioTmp;
		  crc += SioTmp;
		  break;
   case 2: //  Eingangsdaten sammeln
		  RxdBuffer[buf_ptr] = SioTmp;
		  if(buf_ptr < MAX_EMPFANGS_BUFF) buf_ptr++; 
		  else UartState = 0;
		  crc += SioTmp;
		  break;
   default: 
          UartState = 0; 
          break;
  }
}


// --------------------------------------------------------------------------
void AddCRC(unsigned int wieviele)
{
 unsigned int tmpCRC = 0,i; 
 for(i = 0; i < wieviele;i++)
  {
   tmpCRC += SendeBuffer[i];
  }
   tmpCRC %= 4096;
   SendeBuffer[i++] = '=' + tmpCRC / 64;
   SendeBuffer[i++] = '=' + tmpCRC % 64;
   SendeBuffer[i++] = '\r';
  UebertragungAbgeschlossen = 0;
  UDR = SendeBuffer[0];
}



// --------------------------------------------------------------------------
void SendOutData(unsigned char cmd,unsigned char modul, unsigned char *snd, unsigned char len)
{
 unsigned int pt = 0;
 unsigned char a,b,c;
 unsigned char ptr = 0;

 SendeBuffer[pt++] = '#';               // Startzeichen
 SendeBuffer[pt++] = modul;             // Adresse (a=0; b=1,...)
 SendeBuffer[pt++] = cmd;		        // Commando

 while(len)
  {
   if(len) { a = snd[ptr++]; len--;} else a = 0;
   if(len) { b = snd[ptr++]; len--;} else b = 0;
   if(len) { c = snd[ptr++]; len--;} else c = 0;
   SendeBuffer[pt++] = '=' + (a >> 2);
   SendeBuffer[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
   SendeBuffer[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
   SendeBuffer[pt++] = '=' + ( c & 0x3f);
  }
 AddCRC(pt);
}


// --------------------------------------------------------------------------
void Decode64(unsigned char *ptrOut, unsigned char len, unsigned char ptrIn,unsigned char max)  // Wohin mit den Daten; Wie lang; Wo im RxdBuffer
{
 unsigned char a,b,c,d;
 unsigned char ptr = 0;
 unsigned char x,y,z;
 while(len)
  {
   a = RxdBuffer[ptrIn++] - '=';
   b = RxdBuffer[ptrIn++] - '=';
   c = RxdBuffer[ptrIn++] - '=';
   d = RxdBuffer[ptrIn++] - '=';
   if(ptrIn > max - 2) break;     // nicht mehr Daten verarbeiten, als empfangen wurden

   x = (a << 2) | (b >> 4);
   y = ((b & 0x0f) << 4) | (c >> 2);
   z = ((c & 0x03) << 6) | d;

   if(len--) ptrOut[ptr++] = x; else break;
   if(len--) ptrOut[ptr++] = y; else break;
   if(len--) ptrOut[ptr++] = z;	else break;
  }

}

// --------------------------------------------------------------------------
void BearbeiteRxDaten(void)
{
 if(!NeuerDatensatzEmpfangen) return;

// unsigned int tmp_int_arr1[1];
// unsigned int tmp_int_arr2[2];
// unsigned int tmp_int_arr3[3];
 unsigned char tmp_char_arr2[2];
// unsigned char tmp_char_arr3[3];
// unsigned char tmp_char_arr4[4];
 //if(!MotorenEin) 
 PcZugriff = 255;
  switch(RxdBuffer[2])
  {
   case 'a':// Texte der Analogwerte
            Decode64((unsigned char *) &tmp_char_arr2[0],sizeof(tmp_char_arr2),3,AnzahlEmpfangsBytes);
            DebugTextAnforderung = tmp_char_arr2[0];
			break;
   case 'b':
			Decode64((unsigned char *) &ExternControl,sizeof(ExternControl),3,AnzahlEmpfangsBytes);
			RemoteTasten |= ExternControl.RemoteTasten;
            ConfirmFrame = ExternControl.Frame;
            break;
   case 'c':
			Decode64((unsigned char *) &ExternControl,sizeof(ExternControl),3,AnzahlEmpfangsBytes);
			RemoteTasten |= ExternControl.RemoteTasten;
            ConfirmFrame = ExternControl.Frame;
            DebugDataAnforderung = 1;
            break;
   case 'h':// x-1 Displayzeilen
            Decode64((unsigned char *) &tmp_char_arr2[0],sizeof(tmp_char_arr2),3,AnzahlEmpfangsBytes);
            RemoteTasten |= tmp_char_arr2[0];
			if(tmp_char_arr2[1] == 255) NurKanalAnforderung = 1; else NurKanalAnforderung = 0; // keine Displaydaten
			DebugDisplayAnforderung = 1;
			break;
   case 't':// Motortest
            Decode64((unsigned char *) &MotorTest[0],sizeof(MotorTest),3,AnzahlEmpfangsBytes);
			break;
   case 'k':// Keys von DubWise
            Decode64((unsigned char *) &DubWiseKeys[0],sizeof(DubWiseKeys),3,AnzahlEmpfangsBytes);
			ConfirmFrame = DubWiseKeys[3];
			break;
   case 'v': // Version-Anforderung	und Ausbaustufe
            GetVersionAnforderung = 1;
            break;								  
   case 'g':// "Get"-Anforderung für Debug-Daten 
            // Bei Get werden die vom PC einstellbaren Werte vom PC zurückgelesen
            DebugGetAnforderung = 1;
            break;
   case 'q':// "Get"-Anforderung für Settings
            // Bei Get werden die vom PC einstellbaren Werte vom PC zurückgelesen
            Decode64((unsigned char *) &tmp_char_arr2[0],sizeof(tmp_char_arr2),3,AnzahlEmpfangsBytes);
            if(tmp_char_arr2[0] != 0xff)
             {
			  if(tmp_char_arr2[0] > 5) tmp_char_arr2[0] = 5;
	          ReadParameterSet(tmp_char_arr2[0], (unsigned char *) &EE_Parameter.Kanalbelegung[0], STRUCT_PARAM_LAENGE);			
	          SendOutData('L' + tmp_char_arr2[0] -1,MeineSlaveAdresse,(unsigned char *) &EE_Parameter.Kanalbelegung[0],STRUCT_PARAM_LAENGE);
             } 
             else 
	          SendOutData('L' + GetActiveParamSetNumber()-1,MeineSlaveAdresse,(unsigned char *) &EE_Parameter.Kanalbelegung[0],STRUCT_PARAM_LAENGE);
             
            break;
	
   case 'l':
   case 'm':
   case 'n':
   case 'o':
   case 'p': // Parametersatz speichern
            Decode64((unsigned char *) &EE_Parameter.Kanalbelegung[0],STRUCT_PARAM_LAENGE,3,AnzahlEmpfangsBytes);
			WriteParameterSet(RxdBuffer[2] - 'l' + 1, (unsigned char *) &EE_Parameter.Kanalbelegung[0], STRUCT_PARAM_LAENGE);
            eeprom_write_byte(&EEPromArray[EEPROM_ADR_ACTIVE_SET], RxdBuffer[2] - 'l' + 1);  // aktiven Datensatz merken
            Umschlag180Nick = (long) EE_Parameter.WinkelUmschlagNick * 2500L;
            Umschlag180Roll = (long) EE_Parameter.WinkelUmschlagRoll * 2500L;
            Piep(GetActiveParamSetNumber());
         break;
		
         
  }
// DebugOut.AnzahlZyklen =  Debug_Timer_Intervall;
 NeuerDatensatzEmpfangen = 0;
}

//############################################################################
//Routine für die Serielle Ausgabe
int uart_putchar (char c)
//############################################################################
{
	if (c == '\n')
		uart_putchar('\r');
	//Warten solange bis Zeichen gesendet wurde
	loop_until_bit_is_set(USR, UDRE);
	//Ausgabe des Zeichens
	UDR = c;
	
	return (0);
}

// --------------------------------------------------------------------------
void WriteProgramData(unsigned int pos, unsigned char wert)
{
  //if (ProgramLocation == IN_RAM) Buffer[pos] = wert;
  // else eeprom_write_byte(&EE_Buffer[pos], wert);
  // Buffer[pos] = wert;
}

//############################################################################
//INstallation der Seriellen Schnittstelle
void UART_Init (void)
//############################################################################
{
	//Enable TXEN im Register UCR TX-Data Enable & RX Enable

	UCR=(1 << TXEN) | (1 << RXEN);
    // UART Double Speed (U2X)
	USR   |= (1<<U2X);           
	// RX-Interrupt Freigabe
	UCSRB |= (1<<RXCIE);           
	// TX-Interrupt Freigabe
	UCSRB |= (1<<TXCIE);           

	//Teiler wird gesetzt 
	UBRR=(SYSCLK / (BAUD_RATE * 8L) - 1);
	//UBRR = 33;
	//öffnet einen Kanal für printf (STDOUT)
	//fdevopen (uart_putchar, 0);
	//sbi(PORTD,4);
  Debug_Timer = SetDelay(200);  
}

//---------------------------------------------------------------------------------------------
void DatenUebertragung(void)  
{
 if(!UebertragungAbgeschlossen) return;

   if(DebugGetAnforderung && UebertragungAbgeschlossen) 	      // Bei Get werden die vom PC einstellbaren Werte vom PC zurückgelesen
   { 
      SendOutData('G',MeineSlaveAdresse,(unsigned char *) &ExternControl,sizeof(ExternControl));
	  DebugGetAnforderung = 0;
   }

    if((CheckDelay(Debug_Timer) || DebugDataAnforderung) && UebertragungAbgeschlossen) 	 
    	 {
    	  SendOutData('D',MeineSlaveAdresse,(unsigned char *) &DebugOut,sizeof(DebugOut));
       	  DebugDataAnforderung = 0;
          Debug_Timer = SetDelay(MIN_DEBUG_INTERVALL);  
    	 } 
    if(DebugTextAnforderung != 255) // Texte für die Analogdaten
     {
      SendOutData('A',DebugTextAnforderung + '0',(unsigned char *) ANALOG_TEXT[DebugTextAnforderung],16);
      DebugTextAnforderung = 255;
	 }
     if(ConfirmFrame && UebertragungAbgeschlossen)   // Datensatz ohne CRC bestätigen 
	 {
      SendeBuffer[0] = '#';
      SendeBuffer[1] = ConfirmFrame;
      SendeBuffer[2] = '\r';
      UebertragungAbgeschlossen = 0;
      ConfirmFrame = 0;
      UDR = SendeBuffer[0];
     }
     if(DebugDisplayAnforderung && UebertragungAbgeschlossen)
	 {
      Menu();
 	  DebugDisplayAnforderung = 0;
      if(++RemotePollDisplayLine == 4 || NurKanalAnforderung) 
      {
       SendOutData('4',0,(unsigned char *)&PPM_in,sizeof(PPM_in));   // DisplayZeile übertragen
       RemotePollDisplayLine = -1;
      } 
      else  SendOutData('0' + RemotePollDisplayLine,0,(unsigned char *)&DisplayBuff[20 * RemotePollDisplayLine],20);   // DisplayZeile übertragen
	 } 
    if(GetVersionAnforderung && UebertragungAbgeschlossen) 
     { 
      SendOutData('V',MeineSlaveAdresse,(unsigned char *) &VersionInfo,sizeof(VersionInfo));
	  GetVersionAnforderung = 0;
     }

}

