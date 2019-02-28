/**
  @page AN4854 Binary Template Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    IAP_Binary_Template/readme.txt 
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    5-April-2016
  * @brief   Description of preparation of binary file for the AN4854 (in-application programming
  *          using the SDMMC (IAP)).
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

This directory contains a set of sources files that build the application to be
loaded into Flash memory using In-Application Programming (IAP) using the SDIO..

To build such application, some special configuration has to be performed:

STM32F0xx devices:
1. Set the application load address at 0x0800B000, using your toolchain linker file
2. To be able to serve the application interrupts, you need to relocate the vector 
   table (which contains the interrupt handlers). However, unlike CortexM3 and CortexM4, 
   the CortexM0 processor do not support vector table relocation (it is fixed at 
   address 0x00000000).
   A solution will be to relocate by software the vector table to the internal SRAM:  
    - Copy the vector table from the Flash (mapped at the base of the application load
      address 0x0800B000) to the base address of the SRAM at 0x20000000.
    - Remap SRAM at address 0x00000000, using __HAL_SYSCFG_REMAPMEMORY_SRAM() macro
    - Then once an interrupt occurs, the CortexM0 processor will fetch the interrupt 
      handler start address from the relocated vector table in SRAM, then it will 
      jump to execute the interrupt handler located in the Flash.
   This operation should be done at the initialization phase of the application.    

STM32L4xx devices:
1. Set the program load address at 0x0800E000, using your toolchain linker file
2. Relocate the vector table at address 0x0800E000, changing the value of SCB->VTOR.

The SysTick example provided within the STM32F0xx and STM32L4xx HAL Cube examples is used 
as illustration.
This example configures the SysTick to generate a time base equal to 1 ms.
The system clock is set to the maximum system frequency, the SysTick is clocked by 
the AHB clock (HCLK). A "Delay" function is implemented based on the SysTick 
end-of-count event.
Four LEDs are toggled with a timing defined by the Delay function.


@par Directory contents 

 - AN4854-IAP_OVER_SDIO/Projects/IAP_Binary_Template/EWARM: 
										  This folder contains a pre-configured project 
                                          file that produces a binary image of SysTick 
                                          example to be loaded with IAP.
                                          
 - AN4854-IAP_OVER_SDIO/Projects/IAP_Binary_Template/MDK-ARM:
                                          This folder contains a pre-configured project 
                                          file that produces a binary image of SysTick 
                                          example to be loaded with Keil.
 
 - AN4854-IAP_OVER_SDIO/Projects/IAP_Binary_Template/SW4STM32: 
										  This folder contains a SW4STM32 pre-configured project 
                                          file that produces a binary image of SysTick 
                                          example that could be loaded using IAP.										  

 - "AN4854-IAP_OVER_SDIO/Projects/IAP_Binary_Template/Inc": 
   contains the binary_template firmware header files 
    - stm32l4xx_hal_conf.h  Library Configuration file
    - stm32l4xx_it.h        Header for stm32l4xx_it.c
    - stm32f0xx_hal_conf.h  Library Configuration file
    - stm32f0xx_it.h        Header for stm32f0xx_it.c	
    - main.h                Header for main.c

 - "AN4854-IAP_OVER_SDIO/Projects/IAP_Binary_Template/Src": 
   contains the binary_template firmware source files 
     - main.c               Main program
     - stm32l4xx_it.c       Interrupt handlers
     - system_stm32l4xx.c   STM32L4xx system source file
     - stm32f0xx_it.c       Interrupt handlers
     - system_stm32f0xx.c   STM32F0xx system source file
	 
@note The "system_stm32l4xx.c" and "system_stm32f0xx.c" are generated by an automatic clock configuration 
      system and can be easily customized to your own configuration. 

@par Hardware and Software environment

  - This example runs on STM32L4xx Ultra Low Power Devices and STM32F0xx devices.
  
  - This example has been tested with STMicroelectronics STM32L476G-EVAL and STM32072B-EVAL evaluation boards
    and can be easily tailored to any other supported device and development board.

@par How to use it ?  

In order to make the program work, you must do the following:

 - EWARM:
    - Open the Project.eww workspace
    - Rebuild all files: Project->Rebuild all
	- A binary file will be generated 
		- "L4source.bin" under "STM32L476G_EVAL/Exe" folder.  
		- "F0source.bin" under "STM32F072_EVAL/Exe" folder.  
    - Finally load this image with IAP application

 - MDK-ARM:
    - Open the Project.uvproj project
    - Rebuild all files: Project->Rebuild all target files
    - A binary file will be generated  
		- "L4source.bin" under "STM32L476G_EVAL" folder.
		- "F0source.bin" under "STM32F072_EVAL" folder.
    - Finally load this image with IAP application

- SW4STM32:
	- Open the Project.uvproj project
	- Browse to the SW4STM32 workspace directory, select the project 
		- project file in \IAP_Binary_Template\SW4STM32\STM32L476G_EVAL directory).
		- project file in \IAP_Binary_Template\SW4STM32\STM32F072_EVAL directory).	
	- Rebuild all files: Project->Rebuild all target files
	- A binary file will be generated 
		- "L4source.bin" under "STM32L476G_EVAL" folder.
		- "F0source.bin" under "STM32F072_EVAL" folder.
	- Finally load this image with IAP application


 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
 */
