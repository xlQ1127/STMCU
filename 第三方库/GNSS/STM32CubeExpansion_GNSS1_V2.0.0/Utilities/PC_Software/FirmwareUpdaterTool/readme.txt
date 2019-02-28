/**
  @page Firmware Updater Tool for STM32 X-NUCLEO-GNSS1A1 Expansion Board
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    readme.txt  
  * @author  CL/AST  
  * @version V2.0.1
  * @date    20-April-2017
  * @brief   This application allows the user to update the
  *          firmware of the X-NUCLEO-GNSS1A1 expansion board.
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

@par Tool Description 

	- FWUPG is a Java based graphical tool allowing the user to update the 
	firmware of the X-NUCLEO-GNSS1A1 expansion board connected with an STM32 
    Nucleo board.
	- FWUPG is platform independent and supports Windows, Mac OS X and Linux
    
@par Version 2.0.0

	- FWUPG allows the user to upgrade the X-NUCLEO-GNSS1A1 firmware to 
	version 4.6.8_1 
	
@par Hardware and Software environment

	- FWUPG requires JRE (Java Runtime) 6+
	- FWUPG uses jSerialComm (jSerialComm-2.0.2), a platform-independent serial port access library for Java (http://fazecast.github.io/jSerialComm/)
	- FWUPG requires a X-NUCLEO-GNSS1A1 expansion board connected with an STM32 NUCLEO-F401RE board which, in turn, is plugged to the PC via a USB cable
	- The Virtual_COM_Port application should run on the STM32 Nucleo board
	- The tool has been tested with NUCLEO-F401RE RevC

@par How to use it ? 

In order to make the program work, you must do the following:
	- Windows/Mac: The tool can be launched by double-clicking the FWUPG.jar
	- Linux: Due to serial port access permission, the "user" should be added to "dialout" group before launching the tool.
	The tool can be launched by typing the following command line:
	$ java -jar FWUPG.jar	
	- The user should select the relevant serial port and then push the "Open" button.
		- NOTE: On Windows, the active serial port is typically the one highest numbered. 
	- The user can start the upgrading procedure by pushing the "Update FW >>>" button or using the "Tools" menu.
	- A progress bar shall inform the user about the upgrading status.
	- WARNING:
		- The user is not allowed to select an arbitrary firmware image browsing the file system, since
        only the latest firmware image is embedded in the deployable jar.
		- The user is strongly advised not to unplug the STM32 Nucleo board during the upgrading procedure.

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
