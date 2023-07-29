/**
  ******************************************************************************
  * @file    Audio.h 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    8-Nov-2016
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  __AUDIO_H 
#define  __AUDIO_H 
/* Includes ------------------------------------------------------------------*/
#include "utils.h"
#include "mp3dec.h"
#include "assembly.h"
#include "diskio.h"
#include "LScan.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//----------------------------------------------------------------------------//
typedef bool Aud_ty_cb(short* buffer,void* context);//callback for audio timer
//----------------------------------------------------------------------------//
extern bool disableAudio;
extern float SoundGain;

#define _AudioOn()  TIM_Cmd(TIM5,ENABLE)   // запускаем ШИМ
#define _AudioOff() TIM_Cmd(TIM5,DISABLE)  //остановить ШИМ
#define _SetAudioVolume(int_volume) SetSoundGain((float)(int_volume/255.0f))//0-255

//----------------------------------------------------------------------------//
void OutputAudioSample(int16_t sample);
void OutputAudioSampleWithoutBlocking(int16_t sample);
void StopAudio();

FRESULT PlayMP3File(TCHAR* filename);//воспроизвести файл (предыдущий будет остановлен)
//чтение из файла [с предварительным смещением,если не 0]
FRESULT ReadFromFileWithAbsOffset(FIL* fp,uint8_t* outBuffer,UINT abs_offset,UINT* rd_bytes);
//----------------------------------------------------------------------------//
void TIM4_IRQHandler(); //прерывания таймера периода звука/семпловый PCM 
//----------------------------------------------------------------------------//
#define SOUND_PWM_LEN     875           //длина ШИМ (для 96 КГц 84МГц/96КГц=875)
#define SOUND_PWM_CENTER  (SOUND_PWM_LEN/2) //центр ШИМ
#define AUDIO_16BIT_DIV   (32768/SOUND_PWM_CENTER+1) //делитель аудио для 16-битных данных
#define SetSoundGain(fgain) SoundGain=(fgain) //задать усиление(возможны искажения при переполнении ШИМ)
void SetSoundVal(float val);//задать текущую амплитуду выборки 0.0f- +/-1.0f от громкости
void Sound_Config(void);//конфигурация звука , PA3,TIM5 CH4 PWM
void Beep(uint16_t tone,uint16_t timeMs);
void BeepPause(uint16_t timeMs);//вывести звуковую паузу длительностью timeMks миллисекунд
void BeepMelody(uint16_t* MelodyArray,uint16_t notesTotal);//проиграть мелодию
//----------------------------------------------------------------------------//
#endif /* __AUDIO_H */
/******************* (C) COPYRIGHT 2016 dem1305 *****END OF FILE****/