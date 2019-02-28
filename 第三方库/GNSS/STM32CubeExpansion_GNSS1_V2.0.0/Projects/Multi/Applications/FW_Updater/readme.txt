/**
  @page GNSS Expansion Board for STM32 Nucleo Boards FW_Updater Application
  
  @verbatim
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    readme.txt  
  * @author  CL/AST
  * @version V2.0.0
  * @date    Apr-2018
  * @brief   Application to be loaded on an STM32 Nucleo board in order to 
  *          use the tool for updating the GNSS Teseo-LIV3F firmware on the 
  *          X-NUCLEO-GNSS1A1 expansion board.
  ******************************************************************************
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
  @endverbatim

  
@par Example Description 

FW_Updater is the application to be loaded in order to use the tool for
updating the GNSS Teseo-LIV3F firmware on the X-NUCLEO-GNSS1A1 expansion boards
(see readme.txt in folder Utilities/PC_Software/FirmwareUpdaterTool).

Plug the X-NUCLEO-GNSS1A1 expansion board on top of a STM32 Nucleo-F401RE.
Connect the STM32 Nucleo board to your PC.
Drag and drop FW_Updater-*.bin (in Binary folder) on Nucleo drive.
Run the java tool FWUPG.jar (in Utilities/PC_Software/FirmwareUpdaterTool folder).
From the java GUI, after selecting the right serial port, click Open to start a 
connection with your STM32 Nucleo and X-NUCLEO-GNSS1A1 boards.
If the FW version on the Teseo-LIV3F module is not the latest one, click the 
Update FW >>> button to start the firmware upgrading process.

  
@par Hardware and Software environment

  - This example runs on STM32 Nucleo devices with GNSS STM32 expansion board
    (X-NUCLEO-GNSS1A1)
  - This example has been tested with STMicroelectronics:
    - NUCLEO-F401RE RevC board
    and can be easily tailored to any other supported device and development board.
    This example runs also on the NUCLEO-F411RE RevC board, even if the chip could
    be not exploited at its best since the projects are configured for the
    NUCLEO-F401RE target board.

  
@par How to use it? 

In order to make the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - Open IAR toolchain (this firmware has been successfully tested with
   Embedded Workbench V7.80.4).
   Alternatively you can use the Keil uVision toolchain (this firmware
   has been successfully tested with V5.24) or the System Workbench for
   STM32 (this firmware has been successfully tested with Version 2.3.1).
 - Rebuild all files and load your image into target memory.
 - Run the example.
 - Alternatively, you can download the pre-built binaries in "Binary" 
   folder included in the distributed package.


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
