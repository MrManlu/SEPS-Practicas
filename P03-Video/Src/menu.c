/**
	******************************************************************************
	* @file		SEPS/P3-Video/Src/menu.c 
	* @author	MCD Application Team
	* @brief	 This file implements Menu Functions
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
	*		this list of conditions and the following disclaimer.
	* 2. Redistributions in binary form must reproduce the above copyright notice,
	*		this list of conditions and the following disclaimer in the documentation
	*		and/or other materials provided with the distribution.
	* 3. Neither the name of STMicroelectronics nor the names of other 
	*		contributors to this software may be used to endorse or promote products 
	*		derived from this software without specific written permission.
	* 4. This software, including modifications and/or derivative works of this 
	*		software, must execute solely and exclusively on microcontroller or
	*		microprocessor devices manufactured by or for STMicroelectronics.
	* 5. Redistribution and use of this software other than as permitted under 
	*		this license is void and will automatically terminate your rights under 
	*		this license. 
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
#define TOUCH_CAMERASTREAM_XMIN		 225
#define TOUCH_CAMERASTREAM_XMAX		 265
#define TOUCH_CAMERASTREAM_YMIN		 212
#define TOUCH_CAMERASTREAM_YMAX		 252

/* Private macro -------------------------------------------------------------*/
/* Global extern variables ---------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
VIDEO_DEMO_StateMachine	VideoDemo;

/* Private function prototypes -----------------------------------------------*/
static void VIDEO_ChangeSelectMode(VIDEO_DEMO_SelectMode select_mode);
static void LCD_ClearTextZone(void);

/* Private functions ---------------------------------------------------------*/

/**
	* @brief	Manages VIDEO Menu Process.
	* @param	None
	* @retval None
	*/
void VIDEO_MenuProcess(uint8_t initial_state)
{
	TS_StateTypeDef	TS_State;
	Point CameraLogoPoints[] = {{TOUCH_CAMERASTREAM_XMIN, TOUCH_CAMERASTREAM_YMIN},
																{TOUCH_CAMERASTREAM_XMAX, (TOUCH_CAMERASTREAM_YMIN+TOUCH_CAMERASTREAM_YMAX)/2},
																{TOUCH_CAMERASTREAM_XMIN, TOUCH_CAMERASTREAM_YMAX}};

	
	if(appli_state == APPLICATION_READY)
	{ 		
		switch(VideoDemo.state)
		{
		case VIDEO_DEMO_IDLE:
		 
			BSP_LCD_SelectLayer(1);
			
			BSP_LCD_SetFont(&LCD_LOG_HEADER_FONT);
			BSP_LCD_ClearStringLine(13);		 /* Clear touch screen buttons dedicated zone */
			BSP_LCD_ClearStringLine(14);
			BSP_LCD_ClearStringLine(15);
			BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
			BSP_LCD_FillPolygon(CameraLogoPoints, 3);								 /* Camera stream sign */			
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			BSP_LCD_SetFont(&LCD_LOG_TEXT_FONT);
			BSP_LCD_DisplayStringAtLine(15, (uint8_t *)"Use touch screen to enter camera stream module");
		
		
			VideoDemo.state = VIDEO_DEMO_WAIT;
			break;		
			
		case VIDEO_DEMO_WAIT:
			
			BSP_TS_GetState(&TS_State);
			if(TS_State.touchDetected == 1)
			{
				if ((TS_State.touchX[0] > TOUCH_CAMERASTREAM_XMIN) && (TS_State.touchX[0] < TOUCH_CAMERASTREAM_XMAX) &&
								 (TS_State.touchY[0] > TOUCH_CAMERASTREAM_YMIN) && (TS_State.touchY[0] < TOUCH_CAMERASTREAM_YMAX))
				{
					// Se prepara el display para visualizar la captura de camara
					BSP_LCD_SelectLayer(1);
					BSP_LCD_SetFont(&LCD_LOG_TEXT_FONT);
					BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
					for (uint8_t line=4; line<=22; line++) {
						BSP_LCD_ClearStringLine(line);
					}
					BSP_LCD_SetColorKeying(1,LCD_COLOR_BLACK);
					BSP_LCD_SetLayerVisible(0,ENABLE);
					VideoDemo.state = VIDEO_DEMO_CAMERA;
				}				
				
				/* Wait for touch released */
				do
				{
					BSP_TS_GetState(&TS_State);
				} while(TS_State.touchDetected > 0);
			}
			break;	 
			
		case VIDEO_DEMO_CAMERA:
			if(appli_state == APPLICATION_READY) {
				VIDEO_CameraProcess();
			}
			else {
				VideoDemo.state = VIDEO_DEMO_WAIT;
			}
			break;		 
			
		default:
			VideoDemo.state = VIDEO_DEMO_IDLE;
			break;
		}
	}
	
	if(appli_state == APPLICATION_DISCONNECT)
	{
		appli_state = APPLICATION_IDLE;		 
		VIDEO_ChangeSelectMode(VIDEO_SELECT_MENU);				 
	}
}

/*******************************************************************************
														Static Functions
*******************************************************************************/

/**
	* @brief	Changes the selection mode.
	* @param	select_mode: Selection mode
	* @retval None
	*/
static void VIDEO_ChangeSelectMode(VIDEO_DEMO_SelectMode select_mode)
{
	if(select_mode == VIDEO_SELECT_MENU)
	{
		LCD_LOG_UpdateDisplay(); 
		VideoDemo.state = VIDEO_DEMO_IDLE; 
	}
	else if(select_mode == VIDEO_CAMERA_CONTROL)
	{
		LCD_ClearTextZone();	 
	}
}

/**
	* @brief	Clears the text zone.
	* @param	None
	* @retval None
	*/
static void LCD_ClearTextZone(void)
{
	uint8_t i = 0;
	
	for(i= 0; i < 13; i++)
	{
		BSP_LCD_ClearStringLine(i + 3);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
