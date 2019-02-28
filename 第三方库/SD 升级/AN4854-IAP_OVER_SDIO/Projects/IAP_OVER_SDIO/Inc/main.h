/**
  ******************************************************************************
  * @file    Inc/main.h
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    4-April-2016
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F072xB

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"

/* EVAL includes component */
#include "stm32072b_eval.h"
#include "stm32072b_eval_lcd.h"

#else /* STM32F072xB */

#include "stm32l4xx_hal.h"

/* EVAL includes component */
#include "stm32l476g_eval.h"
#include "stm32l476g_eval_io.h"
#include "stm32l476g_eval_lcd.h"

#endif /* STM32F072xB */

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"


/* Exported variables --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef  void (*pFunction)(void);
typedef  UINT         uint_t;
typedef  FILINFO      fileinfo_t;   
typedef  FRESULT      fresult_t;

/* Exported constants --------------------------------------------------------*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifdef STM32F072xB

#define FLASH_SIZE                         ((uint32_t)0x20000)  /* 128 KBytes */
#define IAP_SIZE                           ((uint32_t)0x8000)   /* 32Kbytes as IAP size */

#else /* STM32F072xB */

#define IAP_SIZE                           ((uint32_t)0xE000)

#endif /* STM32F072xB */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Exported variables ------------------------------------------------------- */
extern const char DownloadFile[];

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
