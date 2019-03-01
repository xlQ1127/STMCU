/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2017  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.44 - Graphical user interface for embedded applications **
emWin is protected by international copyright laws.   Knowledge of the
source code may not be used to write a similar product.  This file may
only  be used  in accordance  with  a license  and should  not be  re-
distributed in any way. We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : MOVIE_ShowFromFS.c
Purpose     : Shows how to play a movie directly from a file system.
Requirements: WindowManager - ( )
              MemoryDevices - ( )
              AntiAliasing  - ( )
              VNC-Server    - ( )
              PNG-Library   - ( )
              TrueTypeFonts - ( )

              Requires either a MS Windows environment or emFile.
----------------------------------------------------------------------
*/
#include "GUI.h"
#include "fatfs.h"
/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
//
// Recommended memory to run the sample with adequate performance
//
#define RECOMMENDED_MEMORY (1024L * 100)

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _cbNotify
*
* Function description
*   Callback function for movie player. Uses multiple buffering if
*   available to avoid tearing effects.
*/
void _cbNotify(GUI_HMEM hMem, int Notification, U32 CurrentFrame) {
  switch (Notification) {
  case GUI_MOVIE_NOTIFICATION_PREDRAW:
    GUI_MULTIBUF_Begin();
    break;
  case GUI_MOVIE_NOTIFICATION_POSTDRAW:
    GUI_MULTIBUF_End();
    break;
  case GUI_MOVIE_NOTIFICATION_STOP:
    break;
  }
}

/*********************************************************************
*
*       _GetData
*
* Function description
*   Reading data directly from file system
*/
int _GetData(void * p, const U8 ** ppData, unsigned NumBytes, U32 Off) {
  FRESULT Fres;
  UINT bw;
  FIL * pFile;
  pFile = (FIL *)p;
  
  static int FileAddress = 0;
  
  if(Off == 1) FileAddress = 0;
  else FileAddress = Off;

  Fres=f_lseek(pFile,FileAddress);
  if(Fres != FR_OK) return 0;
  
  Fres=f_read(pFile,ppData,NumBytes,&bw);
  if(Fres != FR_OK) return 0;
  
  return bw;
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       _DrawSmilie
*/
static void _DrawSmilie(int xPos, int yPos, int r) {
  int d;

  GUI_SetColor(GUI_YELLOW);
  GUI_FillCircle(xPos, yPos, r);
  GUI_SetColor(GUI_BLACK);
  GUI_SetPenSize(1);
  GUI_DrawCircle(xPos, yPos, r);
  d = (r * 2) / 5;
  GUI_FillCircle(xPos - d, yPos - d, r / 10);
  GUI_FillCircle(xPos + d, yPos - d, r / 10);
  GUI_DrawVLine(xPos, yPos - d / 2, yPos + d / 2);
  GUI_DrawArc(xPos, yPos + r + d, r, r, 70, 110);
}

/*********************************************************************
*
*       MainTask
*/
void MainTask(void) {
 // GUI_MOVIE_INFO   Info;
  GUI_MOVIE_HANDLE hMovie;
  int              xSize, ySize;
  GUI_RECT         Rect;
  int              FontDistY;

  FIL *    pFile;
  const char       acFileName[] = "TWD.emf";

  FRESULT FsRes;

  GUI_Init();
  //
  // Check if recommended memory for the sample is available
  //
  if (GUI_ALLOC_GetNumFreeBytes() < RECOMMENDED_MEMORY) {
    GUI_ErrorOut("Not enough memory available."); 
    return;
  }
  //
  // Get display size
  //
  xSize = LCD_GetXSize();
  ySize = LCD_GetYSize();
  //
  // Create file handle
  //
  FsRes = f_mount(&SDFatFS,(const TCHAR *)SDPath , 1);
  if(FsRes != FR_OK) return;
  
  FsRes = f_open(&SDFile,acFileName, FA_OPEN_EXISTING|FA_READ|FA_OPEN_ALWAYS);
  if(FsRes != FR_OK) return;
  //
  // Get physical size of movie
  //
  pFile =  &SDFile;

      hMovie = GUI_MOVIE_CreateEx(_GetData, pFile , _cbNotify);
      if (hMovie) {
        GUI_MOVIE_Show(hMovie, (xSize - 320) / 2, (ySize - 240) / 2, 1);
      
  } else {
    GUI_SetBkColor(GUI_RED);
    GUI_Clear();
    GUI_SetFont(GUI_FONT_16_ASCII);
    FontDistY = GUI_GetFontDistY();
    Rect.x0 = 0;
    Rect.y0 = ySize / 2;
    Rect.x1 = xSize - 1;
    Rect.y1 = Rect.y0 + FontDistY * 3 - 1;
    GUI_DispStringInRect("Error opening the movie file.\n"
                         "Please make sure that the given\n"
                         "movie file exists on the specified path:",
                         &Rect, GUI_TA_HCENTER | GUI_TA_TOP);
    Rect.y0 = Rect.y1 + 1;
    Rect.y1 = Rect.y0 + FontDistY;
    GUI_DispStringInRect(acFileName, &Rect, GUI_TA_HCENTER | GUI_TA_TOP);
    Rect.y0 = Rect.y1 + 1;
    Rect.y1 = ySize - 1;
    GUI_DispStringInRect("A sample video can be downloaded\non www.segger.com.", &Rect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    _DrawSmilie(xSize / 2, ySize / 4, ySize / 8);
  }
  while (1) {
    GUI_Exec();
    GUI_X_Delay(1);
  }
}



/*************************** End of file ****************************/
