/**
  ******************************************************************************
  * @file    Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    4-April-2016
  * @brief   Main program body
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

/** @addtogroup STM32L4xx_IAP_Main
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "memory_card.h"
#include "flash_if.h"
#include "command.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_MENU_INDEX    ((uint32_t)4)
#define MENU_ACTIVE       ((uint32_t)1)
#define MENU_NOT_ACTIVE   ((uint32_t)0)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pFunction   JumpToApplication;
uint32_t    JumpAddress;
uint32_t  volatile  FlashProtection = 0;
uint32_t  volatile  MenuIndex = 0;
uint32_t  volatile  SelButtonPressed = 0;
uint32_t  volatile  MainMenuActive = MENU_ACTIVE;
#ifdef STM32F072xB
const char DownloadFile[] = "F0source.bin";
const char UploadFile[] = "F0upload.bin";
#else  /* STM32F072xB */
const char DownloadFile[] = "L4source.bin";
const char UploadFile[] = "L4upload.bin";
#endif /* STM32F072xB */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void RunApplication(void);
static void Main_Menu(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);

  /* Configure the system clock to 48 MHz */
  SystemClock_Config();

  /* Init the user interface */
  BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_GPIO);
  BSP_JOY_Init(JOY_MODE_EXTI);
  BSP_LCD_Init();

  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  /* Set the LCD Back Color */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  /* Set the font size */
  BSP_LCD_SetFont(&Font20);

  /* Test if Key push-button on STM3210C-EVAL Board is pressed */
#ifdef STM32F072xB
  if ( BSP_PB_GetState(BUTTON_TAMPER) == RESET)
#else /* STM32F072xB */
  if ( BSP_PB_GetState(BUTTON_TAMPER) == SET)
#endif /* STM32F072xB */
  {
    /* Initialize Flash */
    FLASH_If_Init();

    /* Initialize CARD */
    if (Card_Init() == FR_OK)
    {
      /* Run main menu */
      Main_Menu();
    }
    else
    {
      Card_Unlink();
      /* Clear the LCD */
      BSP_LCD_Clear(LCD_COLOR_WHITE);
      /*  Error message */
      BSP_LCD_DisplayStringAtLine(1, (uint8_t*)" The card is missing");
      BSP_LCD_DisplayStringAtLine(2, (uint8_t*)" or not functional");
    }
  }
  /* Keep the user application running */
  else
  {
    RunApplication();
  }

  /* Infinite loop */
  while (1)
  {}
}


/**
  * @brief  Display the Main Menu on Display
  * @param  None
  * @retval None
  */
void Main_Menu(void)
{
  while (1)
  {
    /* Clear the LCD */
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAtLine(1, (uint8_t*)"      Main menu ");
    BSP_LCD_DisplayStringAtLine(3, (uint8_t*)"> Download to flash ");
    BSP_LCD_DisplayStringAtLine(4, (uint8_t*)"  Upload from flash ");
    BSP_LCD_DisplayStringAtLine(5, (uint8_t*)"  Erase user flash ");
    BSP_LCD_DisplayStringAtLine(6, (uint8_t*)"  Execute the app ");

    /* Test if any sector of Flash memory where user application will be loaded is write protected */
    FlashProtection = FLASH_If_GetWriteProtectionStatus();
    
    if ((FlashProtection & FLASHIF_PROTECTION_WRPENABLED) != 0)
    {
      BSP_LCD_DisplayStringAtLine(7, (uint8_t*)"  Disable write protect ");
    }
    else
    {
      BSP_LCD_DisplayStringAtLine(7, (uint8_t*)"  Enable write protection ");
    }

    /* Main menu loop */
    MainMenuActive = MENU_ACTIVE;
    MenuIndex = 0;

    while (MainMenuActive == MENU_ACTIVE)
    {
      if (SelButtonPressed == 1)
      {

        MainMenuActive = MENU_NOT_ACTIVE;

        switch (MenuIndex)
        {
            /* From card to FLASH */
          case 0:
            if (f_open(&MyFile, DownloadFile, FA_READ) != FR_OK)
            {
              BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Open file error           ");
            }
            else
            {
              BSP_LCD_DisplayStringAtLine(9, (uint8_t *)" Download ongoing         ");
              if (COMMAND_DOWNLOAD() != DOWNLOAD_OK)
              {
                BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Flash download error       ");
              }
              else
              {
                BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Download done            ");
              }
            }
            break;
            /* From FLASH to card */
          case 1:
            
            if ((FlashProtection & FLASHIF_PROTECTION_RDPENABLED) == 0)
            {  
              /* Remove UPLOAD file if exists on flash disk */
              f_unlink(UploadFile);
              /* Open new file to be wrriten with flash data */
              if (f_open(&MyFile, UploadFile, FA_WRITE | FA_OPEN_ALWAYS) != FR_OK)
              {
                BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Open file error       ");
              }
              else
              {
                /* Display LCD message */
                BSP_LCD_DisplayStringAtLine(9, (uint8_t *)" Upload ongoing         ");
                COMMAND_UPLOAD();
                BSP_LCD_DisplayStringAtLine(9, (uint8_t *)" Upload done            ");
              }
            }
            else
            {
              BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Read protection active   ");
            }
            break;
            /* Erase FLASH */
          case 2:
            BSP_LCD_DisplayStringAtLine(9, (uint8_t *)" Erase ongoing         ");
            if (FLASH_If_Erase (APPLICATION_ADDRESS) != 0)
            {
              /* Clear the LCD */
              BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Flash erase error          ");
            }
            else
            {
              /* Clear the LCD */
              BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" User Flash erased        ");
            }
            break;
            /* Run user application */
          case 3:
            RunApplication();
            break;
            /* Disable/Enable FLASH protection */
          case 4:
            if (FlashProtection != FLASHIF_PROTECTION_NONE)
            {
              /* Disable the write protection */
              if (FLASH_If_WriteProtectionConfig(FLASHIF_WRP_DISABLE) == FLASHIF_OK)
              {
                BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Write Protection disabled   ");
                BSP_LCD_DisplayStringAtLine(10, (uint8_t*)" Press tamper button   ");
                BSP_LCD_DisplayStringAtLine(11, (uint8_t*)" System will now restart    ");
#ifdef STM32F072xB                                
                while (BSP_PB_GetState(BUTTON_TAMPER) == SET);
#else   /* STM32F072xB */
                while (BSP_PB_GetState(BUTTON_TAMPER) == RESET);
#endif  /* STM32F072xB */                
                /* Launch the option byte loading */
                HAL_FLASH_OB_Launch();
              }
              else
              {
                BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Error: Flash write     ");
                BSP_LCD_DisplayStringAtLine(10, (uint8_t*)"  un-protection failed       ");

              }
            }
            else
            {
              if (FLASH_If_WriteProtectionConfig(FLASHIF_WRP_ENABLE) == FLASHIF_OK)
              {
                BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Write Protection enable   ");
                BSP_LCD_DisplayStringAtLine(10, (uint8_t*)" Press tamper button   ");
                BSP_LCD_DisplayStringAtLine(11, (uint8_t*)" System will now restart    ");
#ifdef STM32F072xB                
                while (BSP_PB_GetState(BUTTON_TAMPER) == SET);
#else   /* STM32F072xB */
                while (BSP_PB_GetState(BUTTON_TAMPER) == RESET);
#endif  /* STM32F072xB */                
                {}
                /* Launch the option byte loading */
                HAL_FLASH_OB_Launch();
              }
              else
              {
                BSP_LCD_DisplayStringAtLine(9, (uint8_t *)"Error: Flash write    ");
                BSP_LCD_DisplayStringAtLine(10, (uint8_t*)"  protection failed    ");
              }
            }
            break;

          default:
            /* Clear the LCD */
            BSP_LCD_DisplayStringAtLine(9, (uint8_t*)" Selection Error       ");
            break;
        }

        /* Restore Main menu */
        BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)" ");
        SelButtonPressed = 0;


        BSP_LCD_DisplayStringAtLine(10, (uint8_t*)" Press tamper button         ");
        BSP_LCD_DisplayStringAtLine(11, (uint8_t*)"   to continue                ");

#ifdef STM32F072xB
        while (BSP_PB_GetState(BUTTON_TAMPER) == SET);
#else /* STM32F072xB */
        while (BSP_PB_GetState(BUTTON_TAMPER) == RESET);
#endif /* STM32F072xB */

        /* Reactivate main menu */
        BSP_LCD_DisplayStringAtLine(9, (uint8_t*)"                        ");
        BSP_LCD_DisplayStringAtLine(10, (uint8_t*)"                       ");
        BSP_LCD_DisplayStringAtLine(11, (uint8_t*)"                       ");

        MainMenuActive = MENU_ACTIVE;
        BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)">");

      }
    }
  }
}


#ifdef STM32F072xB
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1

  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;


  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}
#else /* STM32F072xB */
/**
* @brief  System Clock Configuration
*         The system Clock is configured as follows :
*            System Clock source            = PLL (HSE)
*            SYSCLK(Hz)                     = 48000000
*            HCLK(Hz)                       = 48000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 1
*            APB2 Prescaler                 = 2
*            HSE Frequency(Hz)              = 8000000
*            PLL_M                          = 1
*            PLL_N                          = 24
*            PLL_P                          = 7
*            PLL_Q                          = 2
*            PLL_R                          = 4
*            Flash Latency(WS)              = 4
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.MSIState = RCC_MSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }


  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }    
}
#endif /* STM32F072xB */

/**
  * @brief  This function runs a previously loaded application
  * @param  None
  * @retval None
  */
void RunApplication(void)
{
  /* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
  if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
  {
    /* Clear the LCD */
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    /*  Error message */
    BSP_LCD_DisplayStringAtLine(1, (uint8_t*)" The user application");
    BSP_LCD_DisplayStringAtLine(2, (uint8_t*)" running ");

    /* Jump to user application */
    JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
    JumpToApplication = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
#if   (defined ( __GNUC__ ))
    /* Compensation as the Stack Pointer is placed at the very end of RAM */
    __set_MSP((*(__IO uint32_t*) APPLICATION_ADDRESS) - 64);
#else  /* (defined  (__GNUC__ )) */
    __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
#endif /* (defined  (__GNUC__ )) */

    JumpToApplication();
  }
  else
  {
    /* Clear the LCD */
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    /*  Error message */
    BSP_LCD_DisplayStringAtLine(1, (uint8_t*)" The user application");
    BSP_LCD_DisplayStringAtLine(2, (uint8_t*)" isn't loaded properly ");
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while (1)
  {}
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */

#ifdef STM32F072xB
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  if (GPIO_Pin == SD_DETECT_PIN)
  {
    /* Check SD card detect pin */
    BSP_SD_IsDetected();
  }
  if (GPIO_Pin == DOWN_JOY_PIN)
  {
    /* Toggle LED1 */
    BSP_LED_Toggle(LED1);

    BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)" ");

    if (MenuIndex == MAX_MENU_INDEX)
      MenuIndex = 0;
    else
      MenuIndex++;

    BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)">");

  }
  if (GPIO_Pin == UP_JOY_PIN)
  {
    /* Toggle LED2 */

    BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)" ");

    BSP_LED_Toggle(LED2);
    if (MenuIndex == 0)
      MenuIndex = MAX_MENU_INDEX;
    else
      MenuIndex--;

    BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)">");

  }
  if (GPIO_Pin == SEL_JOY_PIN)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
    SelButtonPressed = 1;
  }
}
#else /* STM32F072xB */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static JOYState_TypeDef joystick_state = JOY_NONE;

  if (GPIO_Pin == JOYSTICK_PIN)
  {
    /* Get the Joystick State */
    joystick_state = BSP_JOY_GetState();

    if ( MainMenuActive == MENU_ACTIVE)
    {

      switch (joystick_state)
      {
        case   JOY_DOWN:
          /* Toggle LED1 */
          BSP_LED_Toggle(LED1);

          BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)" ");

          if (MenuIndex == MAX_MENU_INDEX)
            MenuIndex = 0;
          else
            MenuIndex++;

          BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)">");
          break;

        case   JOY_UP:

          /* Toggle LED2 */

          BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)" ");

          BSP_LED_Toggle(LED2);
          if (MenuIndex == 0)
            MenuIndex = MAX_MENU_INDEX;
          else
            MenuIndex--;

          BSP_LCD_DisplayStringAtLine((MenuIndex + 3), (uint8_t*)">");

          break;

        case  JOY_SEL:
          /* Toggle LED2 */
          BSP_LED_Toggle(LED2);
          SelButtonPressed = 1;
          break;

        case JOY_LEFT:
        case JOY_RIGHT:
        default:
          break;
      }
    }

    /* Clear joystick interrupt pending bits */
    BSP_IO_ITClear(JOY_ALL_PINS);
  }
}
#endif /* STM32F072xB */


#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}

#endif


/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
