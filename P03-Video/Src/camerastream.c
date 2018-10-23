/**
  ******************************************************************************
  * @file    SEPS/P03-Video/Src/camerastream.c 
  * @author  MCD Application Team
  * @brief   This file provides the Camera interface API
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
#include "stm32f746xx.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t camera_status = CAMERA_STATUS_IDLE;
uint8_t capture_mode = CAMERA_CONTINUOUS;
// Brillo de la camara. Posibles valores: -2, -1, 0 (normal), +1, +2
int8_t camera_brightness_level = 0;
// Contraste de la camara. Posibles valores: -2, -1, 0 (normal), +1, +2
int8_t camera_contrast_level = 0;
// Configuracion del temporizador TIM3
TIM_HandleTypeDef htim3;

/* Private function prototypes -----------------------------------------------*/

static void VIDEO_ShowStatus(void);
static void VIDEO_AcquireUserButton(void);
static void LCD_LL_ConvertLineToARGB8888(void *pSrc, void *pDst);

uint8_t TIMER_Config(TIM_HandleTypeDef* timer_handler);
uint8_t TIMER_Start(TIM_HandleTypeDef* timer_handler);

// A definir por el alumno
void VIDEO_CAMERA_Brightness_Down(void);
void VIDEO_CAMERA_Brightness_Up(void);
void VIDEO_CAMERA_Contrast_Down(void);
void VIDEO_CAMERA_Contrast_Up(void);
void VIDEO_CAMERA_Antique_Effect(uint8_t state);
void VIDEO_CAMERA_BW_Effect(uint8_t state);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Manages Video process (state machine)
  * @param  None
  * @retval None
  */
void VIDEO_CameraProcess(void) {
	
	VIDEO_ShowStatus();	// Muestra informacion en pantalla (sobre la imagen)
	
	switch(camera_status) {
		
		case CAMERA_STATUS_IDLE:
			TIMER_Config(&htim3);
			/* Start the Camera Capture */
			if (capture_mode == CAMERA_CONTINUOUS) {
				BSP_CAMERA_ContinuousStart((uint8_t *)CAMERA_FRAME_BUFFER);
			}
			else {
				CAMERA_Delay(1000);
				BSP_CAMERA_SnapshotStart((uint8_t *)CAMERA_FRAME_BUFFER);
			}
			TIMER_Start(&htim3);
			camera_status = CAMERA_STATUS_CAPTURE;
			break;
		
		case CAMERA_STATUS_CAPTURE:
			//BSP_LED_On(LED1);
			camera_status = CAMERA_STATUS_CAPTURE;
			VIDEO_AcquireUserButton();
		break;
		
		case CAMERA_STATUS_PAUSE:
			// Hay que suspender la captura de imagenes
			BSP_CAMERA_Suspend();
			//BSP_LED_Off(LED1);
			camera_status = CAMERA_STATUS_PAUSE;
			VIDEO_AcquireUserButton();
		break;
		
		case CAMERA_STATUS_RESUME:
			// Hay que reanudar la captura de imagenes
			BSP_CAMERA_Resume();
			camera_status = CAMERA_STATUS_CAPTURE;
		break;
		
		
		default:
			break;
		
	}

	return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer_handler)
{
	
	if (timer_handler->Instance == TIM3) {
		// Your code here
		BSP_LED_Toggle(LED1);
	}
}

/**
  * @brief  Disminuye el brillo de la camara un nivel
  * @param  None
  * @retval None
  */
void VIDEO_CAMERA_Brightness_Down(void) {
	camera_brightness_level = (camera_brightness_level>-2)?(camera_brightness_level-1):(-2);
	BSP_CAMERA_ContrastBrightnessConfig((uint32_t)(camera_contrast_level+7),(uint32_t)(camera_brightness_level+2));
	return;
}

/**
  * @brief  Aumenta el brillo de la camara un nivel
  * @param  None
  * @retval None
  */
void VIDEO_CAMERA_Brightness_Up(void) {
	camera_brightness_level = (camera_brightness_level<2)?(camera_brightness_level+1):(2);
	BSP_CAMERA_ContrastBrightnessConfig((uint32_t)(camera_contrast_level+7),(uint32_t)(camera_brightness_level+2));
	return;
}

/**
  * @brief  Disminuye el contraste de la camara un nivel
  * @param  None
  * @retval None
  */
void VIDEO_CAMERA_Contrast_Down(void) {
	camera_contrast_level = (camera_contrast_level>-2)?(camera_contrast_level-1):(-2);
	BSP_CAMERA_ContrastBrightnessConfig((uint32_t)(camera_contrast_level+7),(uint32_t)(camera_brightness_level+2));	
	return;
}


/**
  * @brief  Aumenta el contraste de la camara un nivel
  * @param  None
  * @retval None
  */
void VIDEO_CAMERA_Contrast_Up(void) {
	camera_contrast_level = (camera_contrast_level<2)?(camera_contrast_level+1):(2);
	BSP_CAMERA_ContrastBrightnessConfig((uint32_t)(camera_contrast_level+7),(uint32_t)(camera_brightness_level+2));	
	return;
}

/**
  * @brief  Aplica o elimina un efecto de envejecimiento sobre la imagen
  * @param  state: indica si se activa (TRUE) o desactiva (FALSE) el efecto
  * @retval None
  */
void VIDEO_CAMERA_Antique_Effect(uint8_t state) {
	if(state)
		BSP_CAMERA_ColorEffectConfig(CAMERA_COLOR_EFFECT_ANTIQUE);
	else
		BSP_CAMERA_ColorEffectConfig(CAMERA_COLOR_EFFECT_NONE);
	return;
}


/**
  * @brief  Aplica o elimina un efecto de blanco y negro sobre la imagen
  * @param  state: indica si se activa (TRUE) o desactiva (FALSE) el efecto
  * @retval None
  */
void VIDEO_CAMERA_BW_Effect(uint8_t state) {
	if(state)
		BSP_CAMERA_BlackWhiteConfig(CAMERA_BLACK_WHITE_BW);
	else
		BSP_CAMERA_BlackWhiteConfig(CAMERA_BLACK_WHITE_NORMAL);
	return;
}




/*******************************************************************************
                            Static Functions
*******************************************************************************/

static void VIDEO_ShowStatus(void){
	static uint8_t cadena[64];
	static DCMI_TypeDef* camera_regs = DCMI;
	
		sprintf((char*)cadena, "CAM status=%u | Ctrl reg=0x%08X", \
			camera_status, (uint32_t)camera_regs->CR);
		BSP_LCD_SelectLayer(1);
		BSP_LCD_DisplayStringAtLine(21, cadena);
}


/**
  * @brief  Test user button state and modify video state machine according to that
  * @param  None
  * @retval None
  */
static void VIDEO_AcquireUserButton(void)
{
	if(BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_RESET) {
				// Code executed when USER button pressed
		switch (camera_status) {
				case CAMERA_STATUS_PAUSE:
					camera_status = CAMERA_STATUS_RESUME;
					break;

				case CAMERA_STATUS_CAPTURE:
					camera_status = CAMERA_STATUS_PAUSE;
					break;

				default:
					break;
				}
		
				// Antes de abandonar la funcion, se esperan 250ms
				HAL_Delay(250);
  }
	return;
}

/**
  * @brief  Line event callback.
  * @param  None
  * @retval None
  */
void BSP_CAMERA_LineEventCallback(void)
{
  static uint32_t camera_datapointer, lcd_datapointer, line_number;
  
  if(BSP_LCD_GetYSize() > line_number)
  {
    LCD_LL_ConvertLineToARGB8888((uint32_t *)(CAMERA_FRAME_BUFFER + camera_datapointer), (uint32_t *)(LCD_FRAME_BUFFER + lcd_datapointer));
    camera_datapointer = camera_datapointer + BSP_LCD_GetXSize() * (sizeof(uint16_t)); 
    lcd_datapointer = lcd_datapointer + BSP_LCD_GetXSize() * (sizeof(uint32_t));
    line_number++;
  }
  else
  {
    camera_datapointer = 0;
    lcd_datapointer = 0;
    line_number = 0;
  }
}

/**
  * @brief  Converts a line to an ARGB8888 pixel format.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Output color  
  * @retval None
  */
static void LCD_LL_ConvertLineToARGB8888(void *pSrc, void *pDst)
{ 
  static DMA2D_HandleTypeDef hdma2d_eval;  
  
  /* Enable DMA2D clock */
  __HAL_RCC_DMA2D_CLK_ENABLE();
  
  /* Configure the DMA2D Mode, Color Mode and output offset */
  hdma2d_eval.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d_eval.Init.ColorMode    = DMA2D_OUTPUT_ARGB8888;
  hdma2d_eval.Init.OutputOffset = 0;     
  
  /* Foreground Configuration */
  hdma2d_eval.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d_eval.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d_eval.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d_eval.LayerCfg[1].InputOffset = 0;
  
  hdma2d_eval.Instance = DMA2D; 
  
  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d_eval) == HAL_OK) 
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d_eval, 1) == HAL_OK) 
    {
      if (HAL_DMA2D_Start(&hdma2d_eval, (uint32_t)pSrc, (uint32_t)pDst, BSP_LCD_GetXSize(), 1) == HAL_OK)
      {
        /* Polling For DMA transfer */  
        HAL_DMA2D_PollForTransfer(&hdma2d_eval, 10);
      }
    }
  } 
}

uint8_t TIMER_Config(TIM_HandleTypeDef* timer_handler) {
	uint32_t uwPrescalerValue = 0;
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
	
	if (timer_handler == NULL) {
		return 1;
	}

	/* Enable clock for TIM3 */
	__TIM3_CLK_ENABLE();
	
	/* Compute the prescaler value to have TIM3 counter clock equal to 10000 Hz */
	uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 10000) - 1;
	/* Set TIM3 instance */
  timer_handler->Instance = TIM3;
	/* Timer parameters */
	timer_handler->Init.Period            = uwPrescalerValue;
  timer_handler->Init.Prescaler         = 10000 - 1;
  timer_handler->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  timer_handler->Init.CounterMode       = TIM_COUNTERMODE_UP;
  timer_handler->Init.RepetitionCounter = 0;
  timer_handler->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
	if (HAL_TIM_Base_Init(timer_handler) != HAL_OK) {
    /* Initialization Error*/
    return 5;
  }
	
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(timer_handler, &sClockSourceConfig) != HAL_OK) {
		/* Clock source config error */
		return 6;
	}
	
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(timer_handler, &sMasterConfig) != HAL_OK) {
		/* Master sync config error */
		return 7;
	}
	
	/* ALL OK! */
	return 0;
}

uint8_t TIMER_Start(TIM_HandleTypeDef* timer_handler) {
	
	if (timer_handler == NULL) {
		return 1;
	}
	
	if (HAL_TIM_Base_Start_IT(timer_handler) != HAL_OK) {
		/* Starting timer error */
		return 5;
	}
	
	/* ALL OK! */
	
	/* Priority and enabling the TIM3 interruption */
	HAL_NVIC_SetPriority(TIM3_IRQn,4,0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
	return 0;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
