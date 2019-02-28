#ifndef __BSP_Leb_Delay_H
#define __BSP_Leb_Delay_H

#include "stm32f1xx_hal.h"

#if USE_OS
#include "cmsis_os.h"
#endif

#if defined(USE_FULL_LL_DRIVER)
#include "stm32f1xx_ll_utils.h"
#endif

void Leb_LLmDelay(uint32_t ms);
void Leb_HALmDelay(uint32_t ms);
void Leb_OSmDelay(uint32_t ms);

#endif
