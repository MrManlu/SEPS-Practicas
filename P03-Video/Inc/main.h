/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Inc/main.h 
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitte, provided that the following conditions are met:
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "usbh_core.h"
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_camera.h"
#include "stm32746g_discovery_ts.h"
#include "lcd_log.h"
#include "ff.h"    
#include "ff_gen_drv.h"
#include "usbh_diskio.h"

/* Exported Defines ----------------------------------------------------------*/
#define AUDIO_OUT_BUFFER_SIZE                      8192
#define AUDIO_IN_PCM_BUFFER_SIZE                   4*2304 /* buffer size in half-word */

#define FILEMGR_LIST_DEPDTH                        24
#define FILEMGR_FILE_NAME_SIZE                     540
#define FILEMGR_FULL_PATH_SIZE                     256
#define FILEMGR_MAX_LEVEL                          4    
#define FILETYPE_DIR                               0
#define FILETYPE_FILE                              1

#if defined(USE_RES_160x120)
#define resolution      RESOLUTION_R160x120
#define xsize           160
#define ysize           120
#define xoffset         240
#define yoffset         180
#endif

#if defined(USE_RES_320x240)
#define resolution      RESOLUTION_R320x240
#define xsize           320
#define ysize           240
#define xoffset         160
#define yoffset         120
#endif

#if defined(USE_RES_480x272)
#define resolution      RESOLUTION_R480x272
#define xsize           480
#define ysize           272
#define xoffset         80
#define yoffset         104
#endif

#if defined(USE_RES_640x480)
#define resolution      RESOLUTION_R640x480
#define xsize           640
#define ysize           480
#define xoffset         0
#define yoffset         0
#endif

/* Exported types ------------------------------------------------------------*/
/* Application State Machine Structure */
typedef enum {
  APPLICATION_IDLE = 0,  
  APPLICATION_START,   
  APPLICATION_READY,
  APPLICATION_DISCONNECT,
}VIDEO_ApplicationTypeDef;
    
/* Video Demo State Structure */    
typedef enum {
  VIDEO_DEMO_IDLE = 0,
  VIDEO_DEMO_WAIT,
	VIDEO_DEMO_CAMERA,
  AUDIO_DEMO_EXPLORE,
  AUDIO_DEMO_PLAYBACK,
  AUDIO_DEMO_IN, 
}VIDEO_Demo_State;

/* Video Demo State Machine Structure */
typedef struct _DemoStateMachine {
  __IO VIDEO_Demo_State state;
  __IO uint8_t select;  
}VIDEO_DEMO_StateMachine;

typedef enum {
	CAMERA_STATUS_IDLE = 0,
	CAMERA_STATUS_CAPTURE,
	CAMERA_STATUS_PAUSE,
	CAMERA_STATUS_RESUME,
  VIDEO_STATUS_IDLE,
	VIDEO_STATUS_CONTINUOUS,
  VIDEO_STATUS_WAIT,    
  VIDEO_STATUS_INIT,    
  VIDEO_STATUS_PLAY,  
  VIDEO_STATUS_STOP,   
  VIDEO_STATUS_PAUSE,
  VIDEO_STATUS_RESUME,  
	VIDEO_STATUS_EFFECT,
  VIDEO_STATUS_ERROR
}VIDEO_CAMERA_StateTypeDef;

typedef enum {
  VIDEO_SELECT_MENU = 0,
  VIDEO_CAMERA_CONTROL,  
}VIDEO_DEMO_SelectMode;

typedef enum {
  BUFFER_OFFSET_NONE = 0,  
  BUFFER_OFFSET_HALF,  
  BUFFER_OFFSET_FULL,     
}BUFFER_StateTypeDef;

typedef enum {
  BUFFER_EMPTY = 0,  
  BUFFER_FULL,     
}WR_BUFFER_StateTypeDef;

typedef struct _FILELIST_LineTypeDef {
  uint8_t type;
  uint8_t name[FILEMGR_FILE_NAME_SIZE];
}FILELIST_LineTypeDef;

typedef struct _FILELIST_FileTypeDef {
  FILELIST_LineTypeDef  file[FILEMGR_LIST_DEPDTH] ;
  uint16_t              ptr; 
}FILELIST_FileTypeDef;

typedef enum {
  VIDEO_ERROR_NONE = 0,  
  VIDEO_ERROR_IO,
  AUDIO_ERROR_EOF,
  AUDIO_ERROR_INVALID_VALUE,     
}VIDEO_ErrorTypeDef;

extern USBH_HandleTypeDef hUSBHost;
extern VIDEO_ApplicationTypeDef appli_state;
extern VIDEO_CAMERA_StateTypeDef VideoState;
extern FATFS USBH_fatfs;
extern FIL WavFile;

/* Exported constants --------------------------------------------------------*/
/* LCD Frame Buffer address */
#define CAMERA_FRAME_BUFFER               0xC0260000
#define LCD_FRAME_BUFFER                  0xC0130000
#define LCD_FRAME_BUFFER_LAYER1           0xC0000000
#define CONVERTED_FRAME_BUFFER            0xC0390000

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Menu API */
void VIDEO_MenuProcess(uint8_t initial_state);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
