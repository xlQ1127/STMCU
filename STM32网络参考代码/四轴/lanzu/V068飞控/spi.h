// ######################## SPI - FlightCtrl ###################
#ifndef _SPI_H
#define _SPI_H

//#define USE_SPI_COMMUNICATION 

#define SPI_PROTOCOL_COMP   1

//-----------------------------------------
#define DDR_SPI DDRB
#define DD_SS   PB4
#define DD_SCK  PB7
#define DD_MOSI PB5
#define DD_MISO PB6 

// for compatibility reasons gcc3.x <-> gcc4.x 
#ifndef SPCR  
#define SPCR   SPCR0
#endif    
#ifndef SPE  
#define SPE   SPE0
#endif    
#ifndef MSTR  
#define MSTR   MSTR0
#endif    
#ifndef SPR1  
#define SPR1   SPR01
#endif    
#ifndef SPR0  
#define SPR0   SPR00
#endif    
#ifndef SPIE  
#define SPIE   SPIE0
#endif    
#ifndef SPDR  
#define SPDR   SPDR0
#endif    
#ifndef SPIF  
#define SPIF   SPIF0
#endif    
#ifndef SPSR  
#define SPSR   SPSR0
#endif    
// -------------------------

#define SLAVE_SELECT_DDR_PORT   DDRC
#define SLAVE_SELECT_PORT       PORTC
#define SPI_SLAVE_SELECT        PC5

struct str_ToNaviCtrl
{
 unsigned char Sync1, Sync2;
 unsigned char Comp;
 int IntegralNick;
 int IntegralRoll;
 char StickNick,StickRoll,StickGier;
 unsigned char ChkSum;
}; 

struct str_FromNaviCtrl
{
 unsigned int Dummy;
 unsigned char Comp;
 int GPS_Nick;
 int GPS_Roll;
 int CompassValue;
}; 

#ifdef USE_SPI_COMMUNICATION
extern struct str_ToNaviCtrl   ToNaviCtrl;
extern struct str_FromNaviCtrl   FromNaviCtrl;


extern void SPI_MasterInit(void);
extern void SPI_StartTransmitPacket(void);
extern void UpdateSPI_Buffer(void);
extern void SPI_TransmitByte(void);
#else


// -------------------------------- Dummy -----------------------------------------
#define  SPI_MasterInit() ; 
#define  SPI_StartTransmitPacket() ;
#define  UpdateSPI_Buffer() ;
#define  SPI_TransmitByte() ;
#endif


#endif
