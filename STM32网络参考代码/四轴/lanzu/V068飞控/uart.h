 #ifndef _UART_H
 #define _UART_H

#define MAX_SENDE_BUFF     150
#define MAX_EMPFANGS_BUFF  150
#define DUB_KEY_UP     4
#define DUB_KEY_DOWN   8
#define DUB_KEY_RIGHT  32
#define DUB_KEY_LEFT   16
#define DUB_KEY_FIRE   64

void BearbeiteRxDaten(void);

extern unsigned char DebugGetAnforderung;
extern unsigned volatile char SendeBuffer[MAX_SENDE_BUFF];
extern unsigned volatile char RxdBuffer[MAX_EMPFANGS_BUFF];
extern unsigned volatile char UebertragungAbgeschlossen;
extern unsigned volatile char PC_DebugTimeout;
extern unsigned volatile char NeueKoordinateEmpfangen;
extern unsigned char MeineSlaveAdresse;
extern unsigned char PcZugriff;
extern unsigned char RemotePollDisplayLine;
extern int Debug_Timer;
extern void UART_Init (void);
extern int uart_putchar (char c);
extern void boot_program_page (uint32_t page, uint8_t *buf);
extern void DatenUebertragung(void);
extern void DecodeNMEA(void);
extern void BearbeiteRxDaten(void);
extern unsigned char MotorTest[4];
extern unsigned char DubWiseKeys[4]; 
struct str_DebugOut
{
 unsigned char Digital[2];
 unsigned int Analog[32];    // Debugwerte
};

extern struct str_DebugOut    DebugOut;

struct str_ExternControl
{
 unsigned char Digital[2];
 unsigned char RemoteTasten;
 signed char   Nick;
 signed char   Roll;
 signed char   Gier;
 unsigned char Gas;
 signed char   Hight;
 unsigned char free;
 unsigned char Frame;
 unsigned char Config;
}; 
extern struct str_ExternControl   ExternControl;

struct str_VersionInfo
{
  unsigned char Hauptversion;
  unsigned char Nebenversion;
  unsigned char PCKompatibel;
  unsigned char Rserved[7]; 
};   
extern struct str_VersionInfo VersionInfo;

//Die Baud_Rate der Seriellen Schnittstelle ist 9600 Baud
//#define BAUD_RATE 9600		//Baud Rate für die Serielle Schnittstelle	
//#define BAUD_RATE 14400		//Baud Rate für die Serielle Schnittstelle	
//#define BAUD_RATE 28800		//Baud Rate für die Serielle Schnittstelle	
//#define BAUD_RATE 38400		//Baud Rate für die Serielle Schnittstelle	
#define BAUD_RATE 57600		//Baud Rate für die Serielle Schnittstelle	

//Anpassen der seriellen Schnittstellen Register wenn ein ATMega128 benutzt wird
#if defined (__AVR_ATmega128__)
#	define USR UCSR0A
#	define UCR UCSR0B
#	define UDR UDR0
#	define UBRR UBRR0L
#	define EICR EICRB
#endif

#if defined (__AVR_ATmega32__)
#	define USR UCSRA
#	define UCR UCSRB
#	define UBRR UBRRL
#	define EICR EICRB
#   define INT_VEC_RX  SIG_UART_RECV
#   define INT_VEC_TX  SIG_UART_TRANS
#endif

#if defined (__AVR_ATmega644__)
#	define USR  UCSR0A
#	define UCR  UCSR0B
#	define UDR  UDR0
#	define UBRR UBRR0L
#	define EICR EICR0B
#   define TXEN TXEN0
#   define RXEN RXEN0
#   define RXCIE RXCIE0
#   define TXCIE TXCIE0
#   define U2X  U2X0
#   define UCSRB UCSR0B
#   define UDRE UDRE0
#   define INT_VEC_RX  SIG_USART_RECV
#   define INT_VEC_TX  SIG_USART_TRANS
#endif


#endif //_UART_H
