#include "ccd.h"

#define CCD_CLK1 CLK_SetVal() 
#define CCD_CLK0 CLK_ClrVal()

#define CCD_SI1  SI_SetVal() 
#define CCD_SI0  SI_ClrVal() 


word ccd_data[128] = {0};


word i = 0;
word t = 0;
void ccd_collect(void)
{
    
    
   CCD_CLK1;
    CCD_SI0;
    

    CCD_SI1;
    CCD_CLK0;
    

    CCD_CLK1;
    CCD_SI0;

    for(i=0;i<128;i++)
    {
        CCD_CLK0;
        (void)AD1_MeasureChan(1,0);
        (void)AD1_GetValue16(&t);
        ccd_data[i] = t>>4;
        CCD_CLK1;
        
    }
}
