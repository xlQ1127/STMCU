/**
*******************************************************************************
* @file    gnss_data_if.h
* @author  AST/CL
* @version V2.0.0
* @date    Mar-2018
* @brief   LibGNSS module middleware.
*
*******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        www.st.com/software_license_agreement_liberty_v2
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
********************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GNSS_DATA_IF_H
#define GNSS_DATA_IF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gnss_parser.h"

/** @addtogroup MIDDLEWARES
 *  @{
 */

/** @addtogroup ST
 *  @{
 */

/** @addtogroup STM32_GNSS
 *  @{
 */
 
/** @addtogroup LibGNSS
 *  @{
 */

/** @defgroup GNSS_DATA_FUNCTIONS GNSS DATA FUNCTIONS
 *  @brief Prototypes of the API allowing the application to interface the driver
 *  and interact with GNSS module (sending commands, retrieving parsed NMEA info, etc.).
 *  The implementation is up to the application according to specific needs.
 *  @{
 */

/**	
 * @brief  This function puts a string on the console (via UART).
 * @param  pBuffer The string that contains the data to be written on the console
 * @retval None
 */
void GNSS_DATA_IF_ConsoleWrite(uint8_t *pBuffer);

/**
 * @brief  This function puts a single character on the console (via UART).
 * @param  pCh Charatcer to be printed
 * @retval None
 */
void GNSS_DATA_IF_ConsoleWriteChar(uint8_t *pCh);

/**	
 * @brief  This function gets data from the console (via UART).
 * @param  pBuffer The buffer where the received data are stored
 * @retval None
 */
void GNSS_DATA_IF_ConsoleRead(uint8_t *pBuffer, uint16_t size, uint32_t timeout);

/**	
 * @brief  Check if the console status is readable or not and set its status
 * @param  None
 * @retval 1 if the the console status is readable, 0 otherwise
 */
int8_t GNSS_DATA_IF_ConsoleReadable(void);

/**
 * @brief  Function that retrieves data from correctly parsed GPGGA sentence.
 * @param  pGNSSParser_Data The parsed GPGGA sentence
 * @retval None
 */
void GNSS_DATA_IF_GetValidInfo(GNSSParser_Data_t *pGNSSParser_Data);

/**	
 * @brief  This function retrieves the tracking data using the result of parsed GPGGA sentence.
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @param  how_many         The number of the position to track
 * @param  time             Time delay between the tracking of two different positions
 * @retval how_many number of actual acquired positions if the process went good, 0 if didn't
 */
int32_t GNSS_DATA_IF_TrackGotPos(GNSSParser_Data_t *pGNSSParser_Data, uint32_t how_many, uint32_t time);

/**	
 * @brief  This function prints on the console all the position got by a tracking position process.
 * @param  None
 * @retval None
 */
void GNSS_DATA_IF_PrintTrackedPositions(int how_many);

/**	
 * @brief  This function prints on the console the info about Fix data for single 
 *         or combined satellite navigation system.
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetGNSInfo(GNSSParser_Data_t *pGNSSParser_Data);

/**	
 * @brief  This function prints on the console the info about GPS Pseudorange.
 *         Noise Statistics.
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetGPGSTInfo(GNSSParser_Data_t *pGNSSParser_Data);

/**	
 * @brief  This function prints on the console the info about Recommended Minimum Specific
 *         GPS/Transit data got by the most recent reception process.
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetGPRMCInfo(GNSSParser_Data_t *pGNSSParser_Data);

/**	
 * @brief  This function prints on the console the info about GSA satellites got 
 *         by the most recent reception process.
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetGSAInfo(GNSSParser_Data_t *pGNSSParser_Data);

/**	
 * @brief  This function prints on the console the info about GSV satellites got
 *         by the most recent reception process.
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetGSVInfo(GNSSParser_Data_t *pGNSSParser_Data);

/**	
 * @brief  This function prints on the console the info about FW version.
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetPSTMVerInfo(GNSSParser_Data_t *pGNSSParser_Data);

/**	
 * @brief  This function prints on the console the geofence infos each time an alarm is received.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetGeofenceInfo(void *pHandle, GNSSParser_Data_t *pGNSSParser_Data);

/** 
 * @brief  This function prints on the console the info about Odometer.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetOdometerInfo(void *pHandle, GNSSParser_Data_t *pGNSSParser_Data);

/** 
 * @brief  This function prints on the console the info about Datalog.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetDatalogInfo(void *pHandle, GNSSParser_Data_t *pGNSSParser_Data);

/**
 * @brief  This function gets the ACK from the GPS data.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  pGNSSParser_Data Handler of the GNSS data
 * @retval None
 */
void GNSS_DATA_IF_GetMsglistAck(void *pHandle, GNSSParser_Data_t *pGNSSParser_Data);

/**
  * @brief  This function gets the ACK from the GNSS data.
  * @param  pHandle Handler of the GNSS object
  * @retval None
  */
void GNSS_DATA_IF_GetGNSSAck(void *pHandle, GNSSParser_Data_t *pGNSSParser_Data);

/**
 * @brief  This function sends a command to the GNSS module.
 * @param  pHandle  Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  pCommand  The string with NMEA command to be sent to the GNSS module
 * @retval None
 */
void GNSS_DATA_IF_SendCommand(void *pHandle, char *pCommand);

/**
 * @brief  This function configures the message list.
 * @param  pHandle  Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  highMask This HIGH_BITS Mask for configuring message list
 * @retval None
 */
void GNSS_DATA_IF_CfgMessageList(void *pHandle, int highMask);

/**
 * @brief  This function sends a command to enable/disable geofence.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  toggle  The toggle to enable/disable geofence
 * @retval None
 */
void GNSS_DATA_IF_EnableGeofence(void *pHandle, int toggle);

/**	
 * @brief  This function sends a command to config a geofence region 
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  gnss_geofence The geofence region to be configured
 * @retval None
 */
void GNSS_DATA_IF_ConfigGeofence(void *pHandle, void* gnss_geofence);

/**	
 * @brief  This function sends a command to enable/disable odometer.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  toggle The toggle to enable/disable odometer
 * @retval None
 */
void GNSS_DATA_IF_EnableOdo(void *pHandle, int toggle);

/**
 * @brief  This function sends a command to start odometer.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  alarmDistance The distance to raise an alarm
 * @retval None
 */
void GNSS_DATA_IF_StartOdo(void *pHandle, unsigned alarmDistance);

/**
 * @brief  This function sends a command to stop odometer.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @retval None
 */
void GNSS_DATA_IF_StopOdo(void *pHandle);

/**	
 * @brief  This function sends a command to enable/disable datalogging.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  toggle The toggle to enable/disable datalogging
 * @retval None
 */
void GNSS_DATA_IF_EnableDatalog(void *pHandle, int toggle);

/**	
 * @brief  This function sends a command to config datalogging.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @param  gnss_datalog The datalog configuration
 * @retval None
 */
void GNSS_DATA_IF_ConfigDatalog(void *pHandle, void *gnss_datalog);

/**	
 * @brief  This function sends a command to start datalogging.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @retval None
 */
void GNSS_DATA_IF_StartDatalog(void *pHandle);

/**	
 * @brief  This function sends a command to stop datalogging.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @retval None
 */
void GNSS_DATA_IF_StopDatalog(void *pHandle);

/**	
 * @brief  This function sends a command to erase datalogging.
 * @param  pHandle Handler of the GNSS object. Is set to void * to guarantee the generality of the method
 * @retval None
 */
void GNSS_DATA_IF_EraseDatalog(void *pHandle);
/**
 * @}
 */

/**
 * @}
 */
  
/**
 * @}
 */
  
/**
 * @}
 */ 

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* GNSS_DATA_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
