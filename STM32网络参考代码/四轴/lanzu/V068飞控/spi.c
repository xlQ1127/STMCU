// ######################## SPI - FlightCtrl ###################
#include "main.h"


struct str_ToNaviCtrl      ToNaviCtrl;
struct str_FromNaviCtrl    FromNaviCtrl;
unsigned char              SPI_BufferIndex;
volatile unsigned char     SPI_Buffer[sizeof(FromNaviCtrl)];
unsigned char *Ptr_buffer = (unsigned char *) &ToNaviCtrl;

unsigned char SPITransferCompleted, SPI_ChkSum;
#ifdef USE_SPI_COMMUNICATION
//------------------------------------------------------
void SPI_MasterInit(void)
{
  DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK);    // Set MOSI and SCK output, all others input 
  SLAVE_SELECT_DDR_PORT |= (1 << SPI_SLAVE_SELECT);
    
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(0<<SPIE);   // Enable SPI, Master, set clock rate fck/64 
  SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT); 
  SPITransferCompleted = 1;
  
  ToNaviCtrl.Sync1 = 0x81;
  ToNaviCtrl.Sync2 = 0x55;
  
  ToNaviCtrl.Comp = 12;
  ToNaviCtrl.IntegralNick = 12345;
  ToNaviCtrl.IntegralRoll = 56789;
  ToNaviCtrl.StickNick = 100;
  ToNaviCtrl.StickRoll = 150;//(char) StickRoll;
  ToNaviCtrl.StickGier = 200;//(char) StickGier;
  
}

//------------------------------------------------------
void SPI_StartTransmitPacket(void)
{
   //if ((SLAVE_SELECT_PORT & (1 << SPI_SLAVE_SELECT)) == 0) return;    // transfer of prev. packet not completed
   if (!SPITransferCompleted) return;
   
   SLAVE_SELECT_PORT &=  ~(1 << SPI_SLAVE_SELECT);  // SelectSlave
   
   SPITransferCompleted = 0;
   UpdateSPI_Buffer();                              // update buffer
   SPI_BufferIndex = 1;
   DebugOut.Analog[16]++; 
   // -- Debug-Output ---
/*	   DebugOut.Analog[20] = FromNaviCtrl.Comp;
       DebugOut.Analog[21] = FromNaviCtrl.GPS_Nick;
       DebugOut.Analog[22] = FromNaviCtrl.GPS_Roll;
       DebugOut.Analog[23] = FromNaviCtrl.CompassValue;
*/  
   //----
   SPDR = ToNaviCtrl.Sync1;                  // Start transmission 
   ToNaviCtrl.ChkSum = ToNaviCtrl.Sync1;
}

//------------------------------------------------------
//SIGNAL(SIG_SPI)
void SPI_TransmitByte(void)
{
   if (!(SPSR & (1 << SPIF))) return;
  SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT);   // DeselectSlave
    
	  if (SPI_BufferIndex < sizeof(FromNaviCtrl)) 
      { 
	    SPI_Buffer[SPI_BufferIndex]= SPDR;             // get data 
		
//		if (SPI_BufferIndex < 32 ) DebugOut.Analog[26+SPI_BufferIndex] = SPI_Buffer[SPI_BufferIndex];
        
        //if(SPDR!= 0x00) DebugOut.Analog[19]++; ;             
	  }
  
   if (SPI_BufferIndex < sizeof(ToNaviCtrl))  
     { 
	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
 	   SLAVE_SELECT_PORT &=  ~(1 << SPI_SLAVE_SELECT);  // SelectSlave
 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
	   
	   SPDR = Ptr_buffer[SPI_BufferIndex];
	   ToNaviCtrl.ChkSum += Ptr_buffer[SPI_BufferIndex];
 	 }
   else
     {
	   unsigned char *ptr = (unsigned char *)&FromNaviCtrl;
	   
       SPITransferCompleted = 1;  
       memcpy(ptr, (unsigned char *) SPI_Buffer,  sizeof(SPI_Buffer));
	 }
	 
	 SPI_BufferIndex++;
}

//------------------------------------------------------
void UpdateSPI_Buffer(void)
{
  /*static unsigned char i =0;
  cli();
  ToNaviCtrl.Comp = SPI_PROTOCOL_COMP;
  ToNaviCtrl.IntegralNick = (int) (IntegralNick >> 4);
  ToNaviCtrl.IntegralRoll = (int) (IntegralRoll >> 4);
  ToNaviCtrl.StickNick = 4;
  ToNaviCtrl.StickRoll = 5;//(char) StickRoll;
  ToNaviCtrl.StickGier = 6;//(char) StickGier;
  sei();
  */
}

#endif


