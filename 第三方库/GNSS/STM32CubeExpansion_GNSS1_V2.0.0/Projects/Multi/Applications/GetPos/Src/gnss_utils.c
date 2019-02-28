/**
  ******************************************************************************
  * @file       Applications/GetPos/Src/gnss_utils.c
  * @author     ADG ADD Application Software
  * @version    V1.0.0
  * @date       13-Mar-2017
  * @brief      This file contains application specific utilities
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

/* Inlcudes ------------------------------------------------------------------*/
#include "gnss_utils.h"
#include "gnss_data_if.h"
#include "gnss_app_cfg.h"

/* Exported fucntions --------------------------------------------------------*/

void printHelp(void){
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 0) FWUPG:\r\n\tUpgrade the fw of the GNSS device.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 1) GETPOS:\r\n\tGets and decode the first useful $GPGGA NMEA string with the global position information.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 2) LASTPOS:\r\n\tPrints the last saved position from the GPS reception process.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 3) WAKESTATUS:\r\n\tGets the activity state of the GPS datas reception.\r\n\tWill be printed one of the two states: 0 / 1.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 4) HELP:\r\n\tPrints command menu.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 5) DEBUG:\r\n\tChanges the debug state of the application (default is ON).\r\n\tIf debug is ON, when the getpos command is sent, the just decoded position will be printed.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 6) TRACK:\r\n\tBegins the tracking position process.\r\n\tYou have to choose the number of positions that you want to track and the delay between two\r\n\treceptions.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 7) LASTTRACK:\r\n\tIf last tracking process went good, prints last tracked positions on the console.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 8) GETFWVER:\r\n\tSends the ST proprietary $PSTMGETSWVER NMEA command (to be written on serial terminal) and decode the answer with all info about the FW version.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 9) GETGNSMSG:\r\n\tGets and decode the first useful NMEA string (the $--GNS one) with fix data for single or combined satellite navigation system information.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r10) GETGPGST:\r\n\tGets and decode the first useful $GPGST NMEA string with the GPS Pseudorange Noise Statistics.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r11) GETGPRMC:\r\n\tGets and decode the first useful $GPRMC NMEA string with the Recommended Minimum Specific GPS/Transit data.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r12) GETGSAMSG:\r\n\tGets and decode the first useful NMEA string (the $--GSA one) with GNSS DOP and active satellites information.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r13) GETGSVMSG:\r\n\tGets and decode the first useful NMEA string (the $--GSV one) with GNSS Satellites in View information.\n\n\r");
#if (configUSE_FEATURE == 1)
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r14) EN-FEATURE:\r\n\tSends a proprietary ST NMEA command to enable/disable feature and returns the answer.\n");
#endif /* configUSE_FEATURE */

#if (configUSE_GEOFENCE == 1)
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r15) CONF-GEOFENCE:\r\n\tSends a proprietary ST NMEA command to config a geofence circle and returns the answer.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r16) REQ-GEOFENCE:\r\n\tSends a proprietary ST NMEA command to request geofence status and returns the answer.\n");
#endif /* configUSE_GEOFENCE */

#if (configUSE_ODOMETER == 1)
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r17) ODOMETER-OP:\r\n\tSends a proprietary ST NMEA command to start or stop odometer and returns the answer.\n");
#endif /* configUSE_ODOMETER */

#if (configUSE_DATALOG == 1)
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r18) DATALOG-OP:\r\n\tSends a proprietary ST NMEA command to config/start/stop/erase datalog and returns the answer.\n");
#endif /* configUSE_DATALOG */

  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r19) EXT-HELP:\r\n\tPrints this extended help.\n");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r> ");
  //GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r 0) ENDAPP:\r\n\tEnds the application.\r\n\tTo restart it, press CTRL+B command on the console or restart the Nucleo.\n\n\r");
}

void showCmds(void){
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Select a command:\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 0 - fwupg\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 1 - getpos\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 2 - lastpos\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 3 - wakestatus\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 4 - help\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 5 - debug\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 6 - track\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 7 - lasttrack\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 8 - getfwver\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)" 9 - getgnsmsg\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"10 - getgpgst\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"11 - getgprmc\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"12 - getgsamsg\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"13 - getgsvmsg\n\r");
#if (configUSE_FEATURE == 1)  
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"14 - en-feature\n\r");
#endif /* configUSE_FEATURE */

#if (configUSE_GEOFENCE == 1)
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"15 - conf-geofence\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"16 - req-geofence\n\r");
#endif /* configUSE_GEOFENCE == 1 */

#if (configUSE_ODOMETER == 1)
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"17 - odometer-op\n\r");
#endif /* configUSE_ODOMETER */
  
#if (configUSE_DATALOG == 1)
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"18 - datalog-op\n\r");
#endif

  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"19 - ext-help\n\r");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\rSave configuration (y/n)? ");
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r> ");
}

void showPrompt(void){
  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"> ");
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


