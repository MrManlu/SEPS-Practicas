/**
  ******************************************************************************
  * @file    SEPS/P03-Video/Src/main.c
  * @author  MCD Application Team
  * @brief   SEPS-P03 main file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "camerastream.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBH_HandleTypeDef hUSBHost;
VIDEO_ApplicationTypeDef appli_state = APPLICATION_IDLE; 


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void init_buffer_screen(uint32_t* start, uint32_t color);
static void VIDEO_InitApplication(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  
  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */   
  HAL_Init();
  
  /* Configure the system clock to 200 MHz */
  SystemClock_Config(); 
		
  /* Init Audio Application */
  VIDEO_InitApplication();
	appli_state = APPLICATION_READY;
	  
  /* Init TS module */
  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	
	/* Configure KEY Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);  
  
  /* Run Application (infinite loop)*/
  while (1)
  {    
    /* VIDEO Menu Process */
    VIDEO_MenuProcess(VIDEO_DEMO_IDLE);
  }
}


/*******************************************************************************
                            Static Function
*******************************************************************************/

/**
  * @brief  Video Application Init.
  * @param  None
  * @retval None
  */
static void VIDEO_InitApplication(void)
{
	// Variables auxiliares

 // uint32_t  *ptrLcd = NULL;
	uint8_t	  cam_error = CAMERA_OK;
	
   /* Configure LED1 */
  BSP_LED_Init(LED1);
	
  /* Initialize the LCD */
  BSP_LCD_Init();
		
	// Inicializacion de los dos buffers de pantalla
	init_buffer_screen((uint32_t*)LCD_FRAME_BUFFER, LCD_COLOR_MAGENTA);
	init_buffer_screen((uint32_t*)LCD_FRAME_BUFFER_LAYER1, LCD_COLOR_ORANGE);
	
	/* Enable the LCD */
  BSP_LCD_DisplayOn();
	
	/* LCD Layer Initialization */
	BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);
  BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER_LAYER1); 
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	
	BSP_LCD_SetLayerWindow(0, 0, 0, 480, 272);
	
	BSP_LCD_SelectLayer(1);
	BSP_LCD_SetLayerVisible(0,DISABLE);
	BSP_LCD_SetLayerVisible(1,ENABLE);
	
	LCD_LOG_Init();
  LCD_LOG_SetHeader((uint8_t *)"4-GITT -> SEPS -> Practica 3");   
  
  /* Start Audio interface */
  USBH_UsrLog("Starting Video Demo");
	USBH_UsrLog("Waiting for User Interaction...");
  
	cam_error = BSP_CAMERA_Init(resolution);
	if (cam_error == CAMERA_OK) {
		USBH_UsrLog("Camera module init OK at %u x %u", xsize, ysize);
	}
	else{
		USBH_ErrLog("Camera module failed! code=%u", cam_error);
	}

}

static void init_buffer_screen(uint32_t* start, uint32_t color) {
	uint32_t  ind = 0;
	for (ind=0; ind<(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()); ind++) {
    start[ind]=color;
  }
	return;
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 200 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4; 
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}



/**
  * @brief  Clock Config.
  * @param  hltdc: LTDC handle
  * @note   This API is called by BSP_LCD_Init()
  * @retval None
  */
//void BSP_LCD_ClockConfig(LTDC_HandleTypeDef *hltdc, void *Params)
//{
//  static RCC_PeriphCLKInitTypeDef  periph_clk_init_struct;
//  
//  /* RK043FN48H LCD clock configuration */
//  /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
//  /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
//  /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/5 = 38.4 Mhz */
//  /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_4 = 38.4/4 = 9.6Mhz */
//  periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
//  periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
//  periph_clk_init_struct.PLLSAI.PLLSAIR = RK043FN48H_FREQUENCY_DIVIDER;
//  periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
//  HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
//}

#ifdef  USE_FULL_ASSERT
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
  {
  }
}
#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
