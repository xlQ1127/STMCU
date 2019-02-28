/**
  ******************************************************************************
  * @file       Applications/Virtual_COM_Port/Src/gnss_data_if.c
  * @author     AST/CL
  * @version    V1.0.0
  * @date       Mar-2018
  * @brief      Implements interface of LibGNSS module middleware
  *
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"

#include "gnss_data_if.h"
#include "x_nucleo_gnss1a1.h"

#include "gnss_geofence.h"

/* Global variables ----------------------------------------------------------*/
extern osMutexId consoleMutexHandle;

/* Puts a string to console */
void GNSS_DATA_IF_ConsoleWrite(uint8_t *pBuffer)
{
  osMutexWait(consoleMutexHandle, osWaitForever);

  GNSS_IO_Transmit(pBuffer);

  osMutexRelease(consoleMutexHandle);
}

void GNSS_DATA_IF_ConsoleWriteChar(uint8_t *pCh){
  osMutexWait(consoleMutexHandle, osWaitForever);

  GNSS_IO_Transmit(pCh);

  osMutexRelease(consoleMutexHandle);
}

/* Puts a character to console */
void GNSS_DATA_IF_ConsoleRead(uint8_t *pBuffer, uint16_t size, uint32_t timeout)
{
  osMutexWait(consoleMutexHandle, osWaitForever);

  GNSS_IO_Receive(pBuffer, size, timeout);

  osMutexRelease(consoleMutexHandle);
}

/* Checks the console UART read status */
int8_t GNSS_DATA_IF_ConsoleReadable(void)
{
  int status;

  osMutexWait(consoleMutexHandle, osWaitForever);

  status = GNSS_IO_Readable();

  osMutexRelease(consoleMutexHandle);

  return (int8_t)status;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

