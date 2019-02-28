#include "BSP_Leb_Delay.h"

#if defined(USE_FULL_LL_DRIVER)
void Leb_LLmDelay(uint32_t ms)
{
    LL_mDelay(ms);
}
#endif

void Leb_HALmDelay(uint32_t ms)
{

    HAL_Delay(ms);
}

#if USE_OS
void Leb_OSmDelay(uint32_t ms)
{
    osDelay(ms);
}
#endif
