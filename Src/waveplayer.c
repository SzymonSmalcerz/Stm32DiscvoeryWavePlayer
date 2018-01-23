/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/waveplayer.c 
  * @author  MCD Application Team
  * @version V1.3.5
  * @date    17-February-2017
  * @brief   I2S Audio player program. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "../Drivers/BSP/STM32F4-Discovery/stm32f4_discovery_audio.h"
#define AUDIO_BUFFER_SIZE             4096

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
* @{
*/ 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */

extern __IO uint32_t RepeatState, PauseResumeStatus, PressCount;

/* Audio wave data length to be played */
static uint32_t WaveDataLength = 0;

/* Audio wave remaining data length to be played */
static __IO uint32_t AudioRemSize = 0;

__IO uint32_t AudioPlayStart = 0;

/* Ping-Pong buffer used for audio play */
uint8_t Audio_Buffer[AUDIO_BUFFER_SIZE];

/* Position in the audio play buffer */
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
static uint8_t Volume = 70;

/* Variable used by FatFs*/
FIL FileRead;
DIR Directory;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
  buffer_offset = BUFFER_OFFSET_HALF;
}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  buffer_offset = BUFFER_OFFSET_FULL;
  BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE / 2);
}

void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1)
  {}
  
  /* Could also generate a system reset to recover from the error */
  /* .... */
}

int WavePlayerInit(uint32_t AudioFreq)
{
  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
  return(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, AudioFreq));  
}

/**
  * @brief  Plays Wave from a mass storage.
  * @param  AudioFreq: Audio Sampling Frequency
  * @retval None
*/
void WavePlayBack(uint32_t AudioFreq) 
{	
  UINT bytesread = 0;
	
  printf("in WavePlayBack\n");
	
  /* Start playing */
  AudioPlayStart = 1;

  /* Initialize Wave player (Codec, DMA, I2C) */
  if(WavePlayerInit(AudioFreq) != 0) 
  {
    Error_Handler();
  }

  /* Get Data from USB Flash Disk */
  f_lseek(&FileRead, 0);
  f_read (&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
  AudioRemSize = WaveDataLength - bytesread;

  printf("Starting playing wave\n");
    
  /* Start playing Wave */
  BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);

  /* Check if the device is connected.*/
  while(AudioRemSize != 0) 
  {
    bytesread = 0;
    if(buffer_offset == BUFFER_OFFSET_HALF) 
	{
      f_read(&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE/2, (void *)&bytesread);
      buffer_offset = BUFFER_OFFSET_NONE;
    }

    if(buffer_offset == BUFFER_OFFSET_FULL) 
	{
      f_read(&FileRead, &Audio_Buffer[AUDIO_BUFFER_SIZE/2], AUDIO_BUFFER_SIZE/2, (void *)&bytesread);
      buffer_offset = BUFFER_OFFSET_NONE;
    }

    if(AudioRemSize > (AUDIO_BUFFER_SIZE / 2)) 
	{
      AudioRemSize -= bytesread;
    } 
	else 
	{
      AudioRemSize = 0;
    }
  }
  /* Stop playing Wave */
  AudioPlayStart = 0;
  WavePlayerStop();
    
  printf("Finished playing wave\n");
	
  /* Close file */
  f_close(&FileRead);
  AudioRemSize = 0;
}

/**
  * @brief  Stops playing Wave.
  * @param  None
  * @retval None
  */
void WavePlayerStop(void) 
{
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
}

void WavePlayerStart(const char* path, char* wavefilename)
{
  UINT bytesread = 0;
  WAVE_FormatTypeDef waveformat; /* special structure to read wave file */
  
  /* Get the read out protection status */
  if(f_opendir(&Directory, path) == FR_OK)
  {
	printf("Successfully opened directory\n");
    
	/* Open the Wave file to be played */
    if(f_open(&FileRead, wavefilename , FA_READ) == FR_OK)
    {    
	  printf("Successfully opened wave file\n");
      
	  /* Read sizeof(WaveFormat) from the selected file */
      f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
      
      /* Set WaveDataLenght to the Speech Wave length */
      WaveDataLength = waveformat.FileSize;
    
      /* Play the Wave */
      WavePlayBack(waveformat.SampleRate);
    }
    else
	{
	  printf("Opening wave file failed\n");
	}
  }
  else
  {
	printf("Opening directory failed\n");
  }
}

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/