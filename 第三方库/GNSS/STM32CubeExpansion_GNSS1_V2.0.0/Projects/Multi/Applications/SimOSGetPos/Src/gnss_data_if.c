/**
  ******************************************************************************
  * @file       Applications/SimOSGetPos/Src/gnss_data_if.c
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

#include "gnss_data_if.h"
#include "x_nucleo_gnss1a1.h"

#include "gnss_geofence.h"

/* Private defines -----------------------------------------------------------*/
#define MSG_SZ (256)
#define CMD_SZ (90)

/* Private variables ---------------------------------------------------------*/
static char msg[MSG_SZ];
static char gnssCmd[CMD_SZ];

static char *geofenceCirclePosition[] = {
  "Unknown",
  "Outside",
  "Boundary",
  "Inside"
};

/* Puts a string to console */
void GNSS_DATA_IF_ConsoleWrite(uint8_t *pBuffer)
{
  GNSS_IO_Transmit(pBuffer);
}

/* Puts to console data of correctly parsed GPGGA sentence */
void GNSS_DATA_IF_GetValidInfo(GNSSParser_Data_t *pGNSSParser_Data)
{
  
  if(pGNSSParser_Data->gpgga_data.valid == VALID)
  {    
    snprintf(msg, MSG_SZ,  "UTC:\t\t\t[ %02ld:%02ld:%02ld ]\n\r",
            pGNSSParser_Data->gpgga_data.utc.hh, 
            pGNSSParser_Data->gpgga_data.utc.mm, 
            pGNSSParser_Data->gpgga_data.utc.ss);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
    snprintf(msg, MSG_SZ, "Latitude:\t\t[ %.0f' %d'' %c ]\n\r",
            (pGNSSParser_Data->gpgga_data.xyz.lat - ((int)pGNSSParser_Data->gpgga_data.xyz.lat % 100)) / 100, 
            ((int)pGNSSParser_Data->gpgga_data.xyz.lat % 100), 
            pGNSSParser_Data->gpgga_data.xyz.ns);          
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
    snprintf(msg, MSG_SZ, "Longitude:\t\t[ %.0f' %d'' %c ]\n\r",
            (pGNSSParser_Data->gpgga_data.xyz.lon - ((int)pGNSSParser_Data->gpgga_data.xyz.lon % 100)) / 100, 
            ((int)pGNSSParser_Data->gpgga_data.xyz.lon % 100),
            pGNSSParser_Data->gpgga_data.xyz.ew);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
    snprintf(msg, MSG_SZ, "Satellites locked:\t[ %ld ]\n\r",
            pGNSSParser_Data->gpgga_data.sats);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
    snprintf(msg, MSG_SZ, "Position accuracy:\t[ %.1f ]\n\r",
            pGNSSParser_Data->gpgga_data.acc);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
    snprintf(msg, MSG_SZ, "Altitude:\t\t[ %.2f%c ]\n\r",
            pGNSSParser_Data->gpgga_data.xyz.alt, 
            (pGNSSParser_Data->gpgga_data.xyz.mis + 32));
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
    snprintf(msg, MSG_SZ, "Geoid infos:\t\t[ %ld%c ]\n\r",
            pGNSSParser_Data->gpgga_data.geoid.height, 
            pGNSSParser_Data->gpgga_data.geoid.mis);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
    snprintf(msg, MSG_SZ, "Diff update:\t\t[ %ld ]\n\r",
            pGNSSParser_Data->gpgga_data.update);  
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
  }
  else
  {
    snprintf(msg, MSG_SZ,  "Last position wasn't valid.\n\n\r");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
  }
  
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\n\r>");
}

/* Puts to console the geofence infos each time an alarm is received. */
void GNSS_DATA_IF_GetGeofenceInfo(void *pHandle, GNSSParser_Data_t *pGNSSParser_Data)
{  
  if(pGNSSParser_Data->geofence_data.op == GNSS_FEATURE_EN_MSG)
  {
    if(pGNSSParser_Data->geofence_data.result == GNSS_OP_OK)
    {
      snprintf(msg, MSG_SZ,  "Enabling Geofence:\t[ %s ] (Saving params...)\t",
              "EN GEOFENCE OK");
      GNSS_DATA_IF_SendCommand(pHandle, "$PSTMSAVEPAR");
    }
    else
    {
      snprintf(msg, MSG_SZ,  "Enabling Geofence:\t[ %s ]\t",
              "EN GEOFENCE ERROR");
    }
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
  }
  
  if(pGNSSParser_Data->geofence_data.op == GNSS_GEOFENCE_CFG_MSG)
  {
    if(pGNSSParser_Data->geofence_data.result == GNSS_OP_OK)
    {
      snprintf(msg, MSG_SZ,  "Geofence Configuration:\t[ %s ]\t",
              "GEOFENCE CFG OK");
    }
    else
    {
      snprintf(msg, MSG_SZ,  "Geofence Configuration:\t[ %s ]\t",
              "GEOFENCE CFG ERROR");
    }
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
  }
  
  if(pGNSSParser_Data->geofence_data.op == GNSS_GEOFENCE_STATUS_MSG)
  {
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\r\n");
    snprintf(msg, MSG_SZ,  "Time/Date:\t\t%02ld:%02ld:%02ld %02ld/%02ld/%04ld\n",
            pGNSSParser_Data->geofence_data.timestamp.hh,
            pGNSSParser_Data->geofence_data.timestamp.mm,
            pGNSSParser_Data->geofence_data.timestamp.ss,
            pGNSSParser_Data->geofence_data.timestamp.day,
            pGNSSParser_Data->geofence_data.timestamp.month,
            pGNSSParser_Data->geofence_data.timestamp.year);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
    for(uint8_t i = 0; i<MAX_GEOFENCES_NUM; i++)
    {
      snprintf(msg, MSG_SZ,  "Position circle[%d]:\t%s\n",
              i, geofenceCirclePosition[pGNSSParser_Data->geofence_data.status[i]]);
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    }
  }
  
  if(pGNSSParser_Data->geofence_data.op == GNSS_GEOFENCE_ALARM_MSG)
  {
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\r\nGeofence status:\r\n");
    // Print ID - Status
    int idAlarm = pGNSSParser_Data->geofence_data.idAlarm;
    snprintf(msg, MSG_SZ,  "[Circle %d - %s]\t", idAlarm,
            geofenceCirclePosition[pGNSSParser_Data->geofence_data.status[idAlarm]]);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
    
    // Print time date
    snprintf(msg, MSG_SZ,  "%02ld:%02ld:%02ld %02ld/%02ld/%04ld\n",
            pGNSSParser_Data->geofence_data.timestamp.hh,
            pGNSSParser_Data->geofence_data.timestamp.mm,
            pGNSSParser_Data->geofence_data.timestamp.ss,
            pGNSSParser_Data->geofence_data.timestamp.day,
            pGNSSParser_Data->geofence_data.timestamp.month,
            pGNSSParser_Data->geofence_data.timestamp.year);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);
  }
  
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r>");
}

/* Sends a command to the GNSS module. */
void GNSS_DATA_IF_SendCommand(void *pHandle, char *pCommand)
{
  if (pCommand[0] != 36) /* An NMEA command must begin with '$' */
  {
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Invalid command (an NMEA command begins with '$').\n\r");
  }
  else
  {
    snprintf(msg, CMD_SZ,  "%s\n\r", pCommand);
    if (GNSS_Bus_Write(pHandle, (uint8_t *)gnssCmd, strlen(gnssCmd), MAX_DURATION) != GNSS_OK)
    {
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Error sending command\n\n");
    }
  }
}

/* Sends a command to enable/disable Geofence */
void GNSS_DATA_IF_EnableGeofence(void *pHandle, int toggle)
{
  //$PSTMCFGGEOFENCE,<en>,<tol>*<checksum><cr><lf>
  snprintf(msg, CMD_SZ,  "$PSTMCFGGEOFENCE,%d,%d",toggle,1);
  
  GNSS_DATA_IF_SendCommand(pHandle, gnssCmd);
}

/* Configures Geofence */
void GNSS_DATA_IF_ConfigGeofence(void *pHandle, void* gnss_geofence)
{
  snprintf(msg, CMD_SZ,  "$PSTMGEOFENCECFG,%d,%d,%d,%lf,%lf,%lf",
          ((GNSSGeofence_t*)gnss_geofence)->id,
          ((GNSSGeofence_t*)gnss_geofence)->enabled,
          ((GNSSGeofence_t*)gnss_geofence)->tolerance,
          ((GNSSGeofence_t*)gnss_geofence)->lat,
          ((GNSSGeofence_t*)gnss_geofence)->lon,
          ((GNSSGeofence_t*)gnss_geofence)->radius);
  
  GNSS_DATA_IF_SendCommand(pHandle, gnssCmd);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

