/**
  @page AN4854 In-Application Programming using the SDIO Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    IAP_over _SDIO/readme.txt 
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    5-April-2016
  * @brief   Description of implementation of the AN4854 (in-application programming
  *          using the USART (IAP)) using SD card
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

This directory contains a set of sources files and pre-configured projects that 
describes how to build an application to be loaded into Flash memory using
In-Application Programming (IAP, through USART).

@par Directory contents
  - "AN4854-IAP_OVER_SDIO/Projects/IAP_OVER_SDIO/MDK-ARM": contains pre-configured project for MDK-ARM toolchain

  - "AN4854-IAP_OVER_SDIO/Projects/IAP_OVER_SDIO/EWARM": contains pre-configured project for EWARM toolchain
 
  - "AN4854-IAP_OVER_SDIO/Projects/IAP_OVER_SDIO/SW4STM32": contains pre-configured project for SW4STM32 toolchain
										  
  - "AN4854-IAP_OVER_SDIO/Projects/IAP_OVER_SDIO/Inc": contains the IAP firmware header files 
    - main.h               The main include file of the project.
    - command.h            This file provides all the headers of the command functions.
	- memory_card.h        This file provides all the headers of the memory card functions.
    - flash_if.h           This file provides all the headers of the flash memory manipulations. 
	- ffconf.h             File system Configuration file
    - stm32l4xx_hal_conf.h Library Configuration file
    - stm32l4xx_it.h       Header for stm32l4xx_it.c
    - stm32f0xx_hal_conf.h Library Configuration file
    - stm32f0xx_it.h   Header for stm32f0xx_it.c

 - "AN4854-IAP_OVER_SDIO/Projects/IAP_OVER_SDIO/Src": contains the IAP firmware source files
    - main.c                Main program
    - stm32l4xx_it.c        Interrupt handlers
    - stm32l4xx_flash_if.c  The file contains write, erase and disable
                            write protection of the internal Flash memory.
    - stm32f0xx_it.c        Interrupt handlers
    - stm32f0xx_flash_if.c  The file contains write, erase and disable 
	                        write protection of the internal Flash memory.
    - command.c             The file provides selectable command functions
    - memory_card.c			The file provides functions for memory card manipulations
    - system_stm32l4xx.c    STM32L4xx system source file
	- system_stm32f0xx.c    STM32F0xx system source file

 - "AN4854-IAP_OVER_SDIO/Projects/IAP_OVER_SDIO/MDK-ARM": contains pre-configured project for MDK-ARM toolchain

 - "AN4854-IAP_OVER_SDIO/Projects/IAP_OVER_SDIO/EWARM": contains pre-configured project for EWARM toolchain	
 
 - "AN4854-IAP_OVER_SDIO/Projects/IAP_OVER_SDIO/SW4STM32": contains pre-configured project for SW4STM32 toolchain	
	
@par Hardware and Software environment

  - These examples run on STM32L4xx Ultra Low Power Devices and on STM32F072VB devices.
  - These examples has been tested with STMicroelectronics STM32L476G-EVAL and STM32072B-EVAL evaluation boards 
    and can be easily tailored to any other supported device and development board.
  
Table 1. IAP implementation on STM32L476G-EVAL
/*** Platform ***|************* Implementation **************************|***** Configuration *****\
****************************************************************************************************
|    Firmware    | The IAP program is located at 0x08000000. The Flash   |                         |
|                | routines (program/erase) are executed from the Flash  |                         |
|                | memory.                                               |                         |
|                | The size of this program is about 13 Kbytes and       |                         |
|                | programmed on:                                        | Page 0                  |  
|                | ------------------------------------------------------|-------------------------|
|                | The user application (image to be downloaded with the |                         | 
|                | IAP) will be programmed starting from address         |                         |
|                | (uint32_t)0x0800E000(1).                              | (Page 28 - Page 511)    | 
|                | The maximum size of the image to be loaded is:        | (176 Kbytes             | 
|                | ------------------------------------------------------|-------------------------|
|                | The image is uploaded with the IAP from the STM32L4xx | 4 Kbytes                | 
|                | internal Flash.                                       | (Page 28 - Page 29)     |
|                | The size of the image to be uploaded is:              |                         |
|----------------|-------------------------------------------------------|-------------------------|
|    Hardware    | Push-button (active level: high)                      | Tamper push-button      |   |                |                                                                        |  connected to pin PC13  |
|                | ------------------------------------------------------|-------------------------| 
|                | SDIO used                                             |  SDIO                   |
\**************************************************************************************************/ 
(1) User application location address is defined in the flash_if.h file as: 
#define APPLICATION_ADDRESS           ((uint32_t)0x0800E000)
To modify it, change the default value to the desired one. Note that the application must be linked
relatively to the new address too.

Following picture illustrates the situation in program memory:
Figure 1. Flash memory usage

 Top Flash Memory address /-------------------------------------------\  0x08100000
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |          Bank 2                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |-------------------------------------------|  0x08080000
                          |                                           |
                          |          Bank 1                           |
 User code ceiling may    |-------------------------------------------|
 be in Bank 2 in some     |                                           |
 case.                    |          User code                        |
                          |                                           |
                          |- - - - - - - - - - - - - - - - - - - - - -|
 It is possible to con-   |          Vector table                     |
 figure the user code     |-------------------------------------------|  0x0800E000
 starting in Bank 2.      |          IAP code                         |
                          |- - - - - - - - - - - - - - - - - - - - - -|
                          |          Vector table                     |
                          \-------------------------------------------/	 0x08000000					  
   

Table 2. IAP implementation on STM32072B-EVAL
/*** Platform ***|************* Implementation **************************|***** Configuration *****\
****************************************************************************************************
|    Firmware    | The IAP program is located at 0x08000000. The Flash   |                         |
|                | routines (program/erase) are executed from the Flash  |                         |
|                | memory.                                               |                         |
|                | The maximum size of this program (wo optimization)    |                         |
|                | is about 20 Kbytes and                                |                         |
|                | programmed on:                                        | Page 0 - Page 21         |  
|                | ------------------------------------------------------|-------------------------|
|                | The user application (image to be downloaded with the |                         | 
|                | IAP) will be programmed starting from address         |                         |
|                | (uint32_t)0x0800B000(1).                              | (Page 22 - Page 127)    | 
|                | The maximum size of the image to be loaded is:        | (240 Kbytes             | 
|                | ------------------------------------------------------|-------------------------|
|                | The image is uploaded with the IAP from the STM32F0xx | 4 Kbytes               | 
|                | internal Flash.                                       | (Page 22 - Page 23)     |
|                | The size of the image to be uploaded is:              |                         |
|----------------|-------------------------------------------------------|-------------------------|
|    Hardware    | Push-button (active level: low)                       | Tamper push-button      |                |                |                                                       | connected to pin PC13   |
|                | ------------------------------------------------------|-------------------------| 
|                | SPI card used                                         | SPI                     |
\**************************************************************************************************/ 
(1) User application location address is defined in the flash_if.h file as: 
#define APPLICATION_ADDRESS           ((uint32_t)0x0800B000)
To modify it, change the default value to the desired one. Note that the application must be linked
relatively to the new address too.

Following picture illustrates the situation in program memory:
Figure 2. Flash memory usage

 Top Flash Memory address /-------------------------------------------\  0x0803FFFF
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |          Page 22 - Page 127               |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |                                           |
                          |          User code                        |
                          |                                           |
                          |- - - - - - - - - - - - - - - - - - - - - -|
                          |          Vector table                     |
                          |-------------------------------------------|  0x0800B000
                          |          IAP code                         |
                          |- - - - - - - - - - - - - - - - - - - - - -|
                          |          Vector table                     |
                          \-------------------------------------------/	 0x08000000	



@par How to use it? 

In order to make the program work, you must do the following:

In order to make the program work, you must do the following:
	1. Generate a binary image for the program provided in the 
		"IAP/IAP_Main/IAP_Binary_Template" project directory. 
	2. Program the internal Flash with the IAP (see below) 
	3. To run the IAP driver, keep the Tamper push-button pressed at Reset. 
		- Open your preferred toolchain
		- Rebuild all files and load your image into target memory
		- Run the example


    
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
